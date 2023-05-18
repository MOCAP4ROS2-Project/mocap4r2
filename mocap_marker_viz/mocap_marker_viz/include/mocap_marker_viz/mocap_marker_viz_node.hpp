// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#ifndef MOCAP_MARKER_VIZ__MOCAP_MARKER_VIZ_NODE_HPP_
#define MOCAP_MARKER_VIZ__MOCAP_MARKER_VIZ_NODE_HPP_

#include <chrono>
#include <memory>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mocap_marker_viz_srvs/srv/set_marker_color.hpp"
#include "mocap_marker_viz_srvs/srv/reset_marker_color.hpp"

typedef mocap_marker_viz_srvs::srv::SetMarkerColor SetMarkerColor;
typedef mocap_marker_viz_srvs::srv::ResetMarkerColor ResetMarkerColor;
typedef std::shared_ptr<mocap_marker_viz_srvs::srv::SetMarkerColor::Request> SetRequest;
typedef std::shared_ptr<mocap_marker_viz_srvs::srv::SetMarkerColor::Response> SetResponse;
typedef std::shared_ptr<mocap_marker_viz_srvs::srv::ResetMarkerColor::Request> ResetRequest;
typedef std::shared_ptr<mocap_marker_viz_srvs::srv::ResetMarkerColor::Response> ResetResponse;

class MarkerVisualizer : public rclcpp::Node
{
public:
  MarkerVisualizer();

private:
  void marker_callback(const mocap_msgs::msg::Markers::SharedPtr msg) const;
  void rb_callback(const mocap_msgs::msg::RigidBodies::SharedPtr msg) const;

  visualization_msgs::msg::Marker marker2visual(
    int index, const geometry_msgs::msg::Point & translation) const;

  visualization_msgs::msg::Marker rb2visual(
    int index, const geometry_msgs::msg::Pose & poserb) const;

  geometry_msgs::msg::Pose mocap2rviz(const geometry_msgs::msg::Pose mocap_pose) const;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<mocap_msgs::msg::Markers>::SharedPtr markers_subscription_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_rb_;
  rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr markers_subscription_rb_;

  geometry_msgs::msg::Vector3 marker_scale_;
  float marker_lifetime_;
  std::string marker_frame_;
  std::string namespace_;
  std::string mocap_system_;
  std_msgs::msg::ColorRGBA default_marker_color_;
  std::map<int, std_msgs::msg::ColorRGBA> marker_color_;
};

#endif  // MOCAP_MARKER_VIZ__MOCAP_MARKER_VIZ_NODE_HPP_
