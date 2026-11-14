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

#ifndef MOCAP4R2_MARKER_VIZ__MARKER_VIZ_NODE_HPP_
#define MOCAP4R2_MARKER_VIZ__MARKER_VIZ_NODE_HPP_

#include <memory>

#include "mocap4r2_interfaces/msg/markers.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_interfaces/msg/skeletons.hpp"
#include "mocap4r2_marker_viz/viz_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mocap4r2_marker_viz
{

// Shows markers, rigid bodies and skeletons in RViz (topic "mocap_viz"),
// throttled to max_rate because mocap rates exceed what RViz can draw.
// The scene is cleared when no data arrives for stale_timeout seconds.
class MarkerVizNode : public rclcpp::Node
{
public:
  explicit MarkerVizNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish();

  std::unique_ptr<VizBuilder> builder_;
  mocap4r2_interfaces::msg::Markers::ConstSharedPtr markers_;
  mocap4r2_interfaces::msg::RigidBodies::ConstSharedPtr bodies_;
  mocap4r2_interfaces::msg::Skeletons::ConstSharedPtr skeletons_;
  bool dirty_{false};
  rclcpp::Time last_rx_;
  double stale_timeout_{1.0};

  rclcpp::Subscription<mocap4r2_interfaces::msg::Markers>::SharedPtr markers_sub_;
  rclcpp::Subscription<mocap4r2_interfaces::msg::RigidBodies>::SharedPtr bodies_sub_;
  rclcpp::Subscription<mocap4r2_interfaces::msg::Skeletons>::SharedPtr skeletons_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mocap4r2_marker_viz

#endif  // MOCAP4R2_MARKER_VIZ__MARKER_VIZ_NODE_HPP_
