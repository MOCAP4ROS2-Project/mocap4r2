// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>

#include "mocap_robot_gt/gt_component.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mocap_robot_gt
{

using std::placeholders::_1;

GTNode::GTNode(const rclcpp::NodeOptions & options)
: Node("mocap_gt", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  rigid_body_sub_ = create_subscription<mocap_msgs::msg::RigidBody>(
    "rigid_bodies", rclcpp::SensorDataQoS(), std::bind(&GTNode::rigid_body_callback, this, _1));

  declare_parameter<std::string>("root_frame", "odom");
  declare_parameter<std::string>("robot_frame", "base_footprint");
  declare_parameter<std::vector<double>>("init_mocap_xyzrpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  get_parameter("root_frame", root_frame_);
  get_parameter("robot_frame", robot_frame_);

  std::vector<double> init_mocap_coordinates;
  get_parameter("init_mocap_xyzrpy", init_mocap_coordinates);

  if (init_mocap_coordinates.size() == 6u) {
    mocap2root_ = get_tf_from_vector(init_mocap_coordinates);
  } else {
    RCLCPP_ERROR(get_logger(), "Error in init_mocap_xyzrpy coordinates - setting all values to 0");
    mocap2root_ = get_tf_from_vector({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    geometry_msgs::msg::TransformStamped mocap2root_msg;
    mocap2root_msg.header.frame_id = "mocap";
    mocap2root_msg.header.stamp = now();
    mocap2root_msg.transform = tf2::toMsg(mocap2root_);

    static_tf_broadcaster_->sendTransform(mocap2root_msg);
  }
}

void
GTNode::rigid_body_callback(const mocap_msgs::msg::RigidBody::SharedPtr msg)
{
  if (!valid_gtbody2robot_) {
    try {
      auto gtbody2robot_msg = tf_buffer_.lookupTransform(
        "base_mocap", robot_frame_, tf2::TimePointZero);
      tf2::fromMsg(gtbody2robot_msg.transform, gtbody2robot_);
      valid_gtbody2robot_ = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(
        get_logger(), "Transform base_mocap->%s exception: [%s]", robot_frame_.c_str(), e.what());
    }
  } else {
    tf2::Transform mocap2gtbody;
    mocap2gtbody.setOrigin(
      tf2::Vector3(
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    mocap2gtbody.setRotation(
      tf2::Quaternion(
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
        msg->pose.orientation.w));

    tf2::Transform root2robotgt;
    root2robotgt = mocap2root_.inverse() * mocap2gtbody * gtbody2robot_;

    geometry_msgs::msg::TransformStamped root2robotgt_msg;
    root2robotgt_msg.header.frame_id = root_frame_;
    root2robotgt_msg.header.stamp = msg->header.stamp;
    root2robotgt_msg.child_frame_id = robot_frame_ + "_gt";
    root2robotgt_msg.transform = tf2::toMsg(root2robotgt);

    tf_broadcaster_->sendTransform(root2robotgt_msg);
  }
}

tf2::Transform
GTNode::get_tf_from_vector(const std::vector<double> & init_pos)
{
  tf2::Transform ret;
  ret.setOrigin(tf2::Vector3(init_pos[0], init_pos[1], init_pos[2]));

  tf2::Quaternion q;
  q.setEuler(init_pos[3], init_pos[4], init_pos[5]);
  ret.setRotation(q);

  return ret;
}

}  // namespace mocap_robot_gt

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap_robot_gt::GTNode)
