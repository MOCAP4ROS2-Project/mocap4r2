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

#ifndef MOCAP4R2_ROBOT_GT__ROBOT_GT_HPP_
#define MOCAP4R2_ROBOT_GT__ROBOT_GT_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_interfaces/srv/set_origin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mocap4r2_robot_gt
{

// Ground truth of a robot from a mocap rigid body, as a TF tree parallel to the
// robot's own (never touching map/odom/base_link):
//
//   <mocap frame> -> <origin_frame> -> <gt_frame>     (default gt_frame: <robot_frame>_gt)
//
// mocap -> origin_frame is the origin offset (identity until ~/set_origin is called),
// so mocap -> gt_frame is always the robot pose measured by the mocap system.
// The body -> robot transform comes from TF (body_frame) or from body_to_robot.
class RobotGT : public rclcpp::Node
{
public:
  explicit RobotGT(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Last robot pose in the mocap frame, if any.
  std::optional<tf2::Transform> current() const {return current_;}

private:
  void publish_odometry(const tf2::Transform & gt, const std_msgs::msg::Header & header);
  void bodies_callback(const mocap4r2_interfaces::msg::RigidBodies::ConstSharedPtr msg);
  bool resolve_body_to_robot();
  void set_origin(
    const std::shared_ptr<mocap4r2_interfaces::srv::SetOrigin::Request> req,
    std::shared_ptr<mocap4r2_interfaces::srv::SetOrigin::Response> resp);

  std::string rigid_body_;
  std::string robot_frame_;
  std::string body_frame_;
  std::string origin_frame_;
  std::string gt_frame_;
  std::optional<tf2::Transform> body_to_robot_;
  tf2::Transform offset_;
  std::optional<tf2::Transform> current_;
  std::optional<tf2::Transform> previous_gt_;
  rclcpp::Time previous_stamp_;
  double velocity_alpha_{0.5};
  tf2::Vector3 linear_{0, 0, 0};
  tf2::Vector3 angular_{0, 0, 0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<mocap4r2_interfaces::msg::RigidBodies>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<mocap4r2_interfaces::srv::SetOrigin>::SharedPtr origin_srv_;
};

}  // namespace mocap4r2_robot_gt

#endif  // MOCAP4R2_ROBOT_GT__ROBOT_GT_HPP_
