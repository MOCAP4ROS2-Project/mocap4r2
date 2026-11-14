// Copyright 2026 Intelligent Robotics Lab
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

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mocap4r2_robot_gt/gt_math.hpp"
#include "mocap4r2_robot_gt/robot_gt.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mocap4r2_robot_gt
{

RobotGT::RobotGT(const rclcpp::NodeOptions & options)
: Node("robot_gt", options)
{
  rigid_body_ = declare_parameter<std::string>("rigid_body", "");
  robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
  body_frame_ = declare_parameter<std::string>("body_frame", "");
  origin_frame_ = declare_parameter<std::string>("origin_frame", "gt_origin");
  gt_frame_ = declare_parameter<std::string>("gt_frame", "");
  if (gt_frame_.empty()) {
    gt_frame_ = robot_frame_ + "_gt";
  }
  const auto body_to_robot = declare_parameter<std::vector<double>>(
    "body_to_robot", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  const auto origin = declare_parameter<std::vector<double>>(
    "origin", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  velocity_alpha_ = declare_parameter<double>("velocity_alpha", 0.5);
  if (velocity_alpha_ <= 0.0 || velocity_alpha_ > 1.0) {
    throw std::invalid_argument("velocity_alpha must be in (0, 1]");
  }

  // Throws on malformed vectors: a wrong ground truth is worse than none.
  offset_ = from_xyzrpy(origin).inverse();
  if (body_frame_.empty()) {
    body_to_robot_ = from_xyzrpy(body_to_robot);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sub_ = create_subscription<mocap4r2_interfaces::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(10).best_effort(),
    [this](mocap4r2_interfaces::msg::RigidBodies::ConstSharedPtr msg) {bodies_callback(msg);});
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", rclcpp::QoS(10));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(10));
  origin_srv_ = create_service<mocap4r2_interfaces::srv::SetOrigin>(
    "~/set_origin",
    [this](
      const std::shared_ptr<mocap4r2_interfaces::srv::SetOrigin::Request> req,
      std::shared_ptr<mocap4r2_interfaces::srv::SetOrigin::Response> resp) {
      set_origin(req, resp);
    });
}

bool
RobotGT::resolve_body_to_robot()
{
  if (body_to_robot_) {
    return true;
  }
  try {
    // Pose of the robot frame expressed in the marker body frame.
    const auto t = tf_buffer_->lookupTransform(body_frame_, robot_frame_, tf2::TimePointZero);
    tf2::Transform tr;
    tf2::fromMsg(t.transform, tr);
    body_to_robot_ = tr;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for TF %s -> %s: %s",
      body_frame_.c_str(), robot_frame_.c_str(), e.what());
    return false;
  }
}

void
RobotGT::bodies_callback(const mocap4r2_interfaces::msg::RigidBodies::ConstSharedPtr msg)
{
  const mocap4r2_interfaces::msg::RigidBody * body = nullptr;
  if (rigid_body_.empty()) {
    if (msg->rigid_bodies.size() == 1u) {
      body = &msg->rigid_bodies[0];
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%zu rigid bodies received: set parameter rigid_body", msg->rigid_bodies.size());
      return;
    }
  } else {
    for (const auto & rb : msg->rigid_bodies) {
      if (rb.name == rigid_body_) {
        body = &rb;
        break;
      }
    }
    if (!body) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "rigid body '%s' not in %s",
        rigid_body_.c_str(), sub_->get_topic_name());
      return;
    }
  }
  if (!body->tracking_valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "rigid body '%s' not tracked", body->name.c_str());
    return;
  }
  if (!resolve_body_to_robot()) {
    return;
  }

  tf2::Transform mocap_body;
  try {
    mocap_body = from_pose(body->pose);
  } catch (const std::invalid_argument &) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid orientation, frame skipped");
    return;
  }
  current_ = mocap_body * (*body_to_robot_);

  geometry_msgs::msg::TransformStamped origin_tf;
  origin_tf.header = msg->header;
  origin_tf.child_frame_id = origin_frame_;
  origin_tf.transform = tf2::toMsg(offset_.inverse());
  geometry_msgs::msg::TransformStamped gt_tf;
  gt_tf.header.stamp = msg->header.stamp;
  gt_tf.header.frame_id = origin_frame_;
  gt_tf.child_frame_id = gt_frame_;
  const tf2::Transform gt = offset_ * (*current_);
  gt_tf.transform = tf2::toMsg(gt);
  tf_broadcaster_->sendTransform({origin_tf, gt_tf});

  geometry_msgs::msg::PoseStamped pose;
  pose.header = gt_tf.header;
  pose.pose = to_pose(gt);
  pose_pub_->publish(pose);
  publish_odometry(gt, gt_tf.header);
}

void
RobotGT::publish_odometry(const tf2::Transform & gt, const std_msgs::msg::Header & header)
{
  const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
  if (previous_gt_) {
    const double dt = (stamp - previous_stamp_).seconds();
    if (dt <= 0.0) {
      return;  // repeated or out-of-order frame
    }
    // Velocities in the robot frame (REP-105 odometry twist).
    const tf2::Transform delta = previous_gt_->inverse() * gt;
    const tf2::Vector3 v = delta.getOrigin() / dt;
    const tf2::Quaternion dq = delta.getRotation();
    const tf2::Vector3 w = dq.getAxis() * (dq.getAngle() > M_PI ? dq.getAngle() - 2.0 * M_PI :
      dq.getAngle()) / dt;
    linear_ = linear_ * (1.0 - velocity_alpha_) + v * velocity_alpha_;
    angular_ = angular_ * (1.0 - velocity_alpha_) + w * velocity_alpha_;
  }
  previous_gt_ = gt;
  previous_stamp_ = stamp;

  nav_msgs::msg::Odometry odom;
  odom.header = header;
  odom.child_frame_id = gt_frame_;
  odom.pose.pose = to_pose(gt);
  odom.twist.twist.linear.x = linear_.x();
  odom.twist.twist.linear.y = linear_.y();
  odom.twist.twist.linear.z = linear_.z();
  odom.twist.twist.angular.x = angular_.x();
  odom.twist.twist.angular.y = angular_.y();
  odom.twist.twist.angular.z = angular_.z();
  odom_pub_->publish(odom);
}

void
RobotGT::set_origin(
  const std::shared_ptr<mocap4r2_interfaces::srv::SetOrigin::Request> req,
  std::shared_ptr<mocap4r2_interfaces::srv::SetOrigin::Response> resp)
{
  if (!current_) {
    resp->success = false;
    resp->message = "no valid pose of the robot received yet";
    return;
  }
  tf2::Transform wish = tf2::Transform::getIdentity();
  if (!req->use_current) {
    try {
      wish = from_pose(req->origin);
    } catch (const std::invalid_argument & e) {
      resp->success = false;
      resp->message = std::string("invalid origin: ") + e.what();
      return;
    }
  }
  offset_ = origin_offset(*current_, wish);
  resp->success = true;
}

}  // namespace mocap4r2_robot_gt

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_robot_gt::RobotGT)
