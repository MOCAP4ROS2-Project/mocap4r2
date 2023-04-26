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

// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#ifndef MOCAP_ROBOT_GT__GTNODE_HPP_
#define MOCAP_ROBOT_GT__GTNODE_HPP_


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

#include "mocap_msgs/msg/rigid_bodies.hpp"
#include "mocap_robot_gt_msgs/srv/set_gt_origin.hpp"

#include "rclcpp/rclcpp.hpp"


namespace mocap_robot_gt
{
class GTNode : public rclcpp::Node
{
public:
  explicit GTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void rigid_bodies_callback(const mocap_msgs::msg::RigidBodies::SharedPtr msg);
  void set_gt_origin_callback(
    const std::shared_ptr<mocap_robot_gt_msgs::srv::SetGTOrigin::Request> req,
    std::shared_ptr<mocap_robot_gt_msgs::srv::SetGTOrigin::Response> resp);

  geometry_msgs::msg::Pose get_pose_from_vector(const std::vector<double> & init_pos);

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr rigid_body_sub_;
  rclcpp::Service<mocap_robot_gt_msgs::srv::SetGTOrigin>::SharedPtr set_gt_origin_srv_;

  std::string root_frame_;
  std::string robot_frame_;

  tf2::Transform offset_;
  tf2::Transform gtbody2robot_;
  tf2::Transform mocap2gtbody_;

  bool valid_gtbody2robot_{false};

};

}  // namespace mocap_robot_gt

#endif  // MOCAP_ROBOT_GT__GTNODE_HPP_
