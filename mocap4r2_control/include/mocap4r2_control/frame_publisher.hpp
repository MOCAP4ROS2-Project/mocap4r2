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

#ifndef MOCAP4R2_CONTROL__FRAME_PUBLISHER_HPP_
#define MOCAP4R2_CONTROL__FRAME_PUBLISHER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_interfaces/msg/markers.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_interfaces/msg/skeletons.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace mocap4r2_control
{

// One frame of a mocap system, already in the driver frame (Z-up, metres).
struct MocapFrame
{
  uint64_t frame_number{0};
  // Time at which the frame was captured (receive time minus system latency).
  rclcpp::Time capture_time;
  std::vector<mocap4r2_interfaces::msg::Marker> markers;
  std::vector<mocap4r2_interfaces::msg::RigidBody> rigid_bodies;
  std::vector<mocap4r2_interfaces::msg::Skeleton> skeletons;
};

// Publishes frames of a MocapSystemNode consistently: one stamp and frame number
// per frame, statistics, and optional TF of valid rigid bodies.
//
// Parameters (declared on the node): publish_markers, publish_rigid_bodies,
// publish_skeletons, publish_tf, tf_prefix.
// Create it in on_system_configure; publish() may be called from any thread.
class FramePublisher
{
public:
  explicit FramePublisher(MocapSystemNode & node);

  // Publish if the node is active; returns false if the frame was discarded.
  bool publish(MocapFrame && frame);

private:
  MocapSystemNode & node_;
  std::string tf_prefix_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_interfaces::msg::Markers>::SharedPtr markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_interfaces::msg::RigidBodies>::SharedPtr
    bodies_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_interfaces::msg::Skeletons>::SharedPtr
    skeletons_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

// Declare the FramePublisher parameters (called by MocapSystemNode subclasses once).
void declare_frame_publisher_parameters(MocapSystemNode & node);

}  // namespace mocap4r2_control

#endif  // MOCAP4R2_CONTROL__FRAME_PUBLISHER_HPP_
