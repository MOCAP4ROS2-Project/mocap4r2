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

#ifndef MOCAP4R2_MARKER_VIZ__VIZ_BUILDER_HPP_
#define MOCAP4R2_MARKER_VIZ__VIZ_BUILDER_HPP_

#include "mocap4r2_interfaces/msg/markers.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_interfaces/msg/skeletons.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mocap4r2_marker_viz
{

struct VizConfig
{
  double marker_size{0.014};  // sphere diameter, metres
  double axis_length{0.1};    // rigid body axes, metres
  bool show_names{true};
};

// Builds RViz markers from mocap messages. Every array starts with DELETEALL,
// so stale objects never remain, and ids are stable from frame to frame.
class VizBuilder
{
public:
  explicit VizBuilder(const VizConfig & config = VizConfig())
  : config_(config) {}

  // Any of the inputs may be null.
  visualization_msgs::msg::MarkerArray build(
    const mocap4r2_interfaces::msg::Markers * markers,
    const mocap4r2_interfaces::msg::RigidBodies * bodies,
    const mocap4r2_interfaces::msg::Skeletons * skeletons) const;

private:
  void add_markers(
    const mocap4r2_interfaces::msg::Markers & msg,
    visualization_msgs::msg::MarkerArray & out) const;
  void add_bodies(
    const mocap4r2_interfaces::msg::RigidBodies & msg,
    visualization_msgs::msg::MarkerArray & out) const;
  void add_skeletons(
    const mocap4r2_interfaces::msg::Skeletons & msg,
    visualization_msgs::msg::MarkerArray & out) const;

  VizConfig config_;
};

}  // namespace mocap4r2_marker_viz

#endif  // MOCAP4R2_MARKER_VIZ__VIZ_BUILDER_HPP_
