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

#include <chrono>
#include <memory>
#include <stdexcept>

#include "mocap4r2_marker_viz/marker_viz_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mocap4r2_marker_viz
{

MarkerVizNode::MarkerVizNode(const rclcpp::NodeOptions & options)
: Node("mocap4r2_marker_viz", options)
{
  VizConfig config;
  config.marker_size = declare_parameter<double>("marker_size", 0.014);
  config.axis_length = declare_parameter<double>("axis_length", 0.1);
  config.show_names = declare_parameter<bool>("show_names", true);
  const double max_rate = declare_parameter<double>("max_rate", 30.0);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 1.0);
  last_rx_ = now();
  if (stale_timeout_ <= 0.0 || max_rate <= 0.0 || config.marker_size <= 0.0 ||
    config.axis_length <= 0.0)
  {
    throw std::invalid_argument(
            "max_rate, stale_timeout, marker_size and axis_length must be > 0");
  }
  builder_ = std::make_unique<VizBuilder>(config);

  // Keep only the newest message: drawing is throttled anyway.
  const auto qos = rclcpp::QoS(1).best_effort();
  markers_sub_ = create_subscription<mocap4r2_interfaces::msg::Markers>(
    "markers", qos, [this](mocap4r2_interfaces::msg::Markers::ConstSharedPtr m) {
      markers_ = m;
      last_rx_ = now();
      dirty_ = true;
    });
  bodies_sub_ = create_subscription<mocap4r2_interfaces::msg::RigidBodies>(
    "rigid_bodies", qos, [this](mocap4r2_interfaces::msg::RigidBodies::ConstSharedPtr m) {
      bodies_ = m;
      last_rx_ = now();
      dirty_ = true;
    });
  skeletons_sub_ = create_subscription<mocap4r2_interfaces::msg::Skeletons>(
    "skeletons", qos, [this](mocap4r2_interfaces::msg::Skeletons::ConstSharedPtr m) {
      skeletons_ = m;
      last_rx_ = now();
      dirty_ = true;
    });
  pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("mocap_viz", rclcpp::QoS(1));
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / max_rate), [this] {publish();});
}

void
MarkerVizNode::publish()
{
  const bool has_data = markers_ || bodies_ || skeletons_;
  if (has_data && (now() - last_rx_).seconds() > stale_timeout_) {
    // Source stopped: clear the scene once.
    markers_.reset();
    bodies_.reset();
    skeletons_.reset();
    dirty_ = true;
  }
  if (!dirty_ || pub_->get_subscription_count() == 0) {
    return;
  }
  dirty_ = false;
  pub_->publish(builder_->build(markers_.get(), bodies_.get(), skeletons_.get()));
}

}  // namespace mocap4r2_marker_viz

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_marker_viz::MarkerVizNode)
