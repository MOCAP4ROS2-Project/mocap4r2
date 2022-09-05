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

#include <string>

#include "mocap_marker_viz/mocap_marker_viz_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkerVisualizer::MarkerVisualizer()
: Node("marker_visualizer")
{
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization_marker", 1000);

  declare_parameter<float>("default_marker_color_r", 0.0f);
  declare_parameter<float>("default_marker_color_g", 1.0f);
  declare_parameter<float>("default_marker_color_b", 0.0f);
  declare_parameter<float>("default_marker_color_a", 1.0f);
  declare_parameter<double>("marker_scale_x", 0.014f);
  declare_parameter<double>("marker_scale_y", 0.014f);
  declare_parameter<double>("marker_scale_z", 0.014f);
  declare_parameter<float>("marker_lifetime", 0.01f);
  declare_parameter<std::string>("marker_frame", "mocap");
  declare_parameter<std::string>("namespace", "mocap_markers");

  get_parameter<float>("default_marker_color_r", default_marker_color_.r);
  get_parameter<float>("default_marker_color_g", default_marker_color_.g);
  get_parameter<float>("default_marker_color_b", default_marker_color_.b);
  get_parameter<float>("default_marker_color_a", default_marker_color_.a);
  get_parameter<double>("marker_scale_x", marker_scale_.x);
  get_parameter<double>("marker_scale_y", marker_scale_.y);
  get_parameter<double>("marker_scale_z", marker_scale_.z);
  get_parameter<float>("marker_lifetime", marker_lifetime_);
  get_parameter<std::string>("marker_frame", marker_frame_);
  get_parameter<std::string>("namespace", namespace_);


  markers_subscription_ = this->create_subscription<mocap_msgs::msg::Markers>(
    "markers", 1000, std::bind(&MarkerVisualizer::marker_callback, this, _1));

  // Rigid bodies
  markers_subscription_rb_ = this->create_subscription<mocap_msgs::msg::RigidBody>(
    "rigid_bodies", 1000, std::bind(&MarkerVisualizer::rb_callback, this, _1));

  publisher_rb_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "visualization_marker_rb", 1000);
}


void
MarkerVisualizer::marker_callback(const mocap_msgs::msg::Markers::SharedPtr msg) const
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  static int counter = 0;
  visualization_msgs::msg::MarkerArray visual_markers;
  for (const mocap_msgs::msg::Marker & marker : msg->markers) {
    visual_markers.markers.push_back(marker2visual(counter++, marker.translation));
  }
  publisher_->publish(visual_markers);
}

visualization_msgs::msg::Marker
MarkerVisualizer::marker2visual(int index, const geometry_msgs::msg::Point & translation) const
{
  visualization_msgs::msg::Marker viz_marker;
  viz_marker.header.frame_id = marker_frame_;
  viz_marker.header.stamp = rclcpp::Clock().now();
  viz_marker.ns = namespace_;
  viz_marker.color = default_marker_color_;
  viz_marker.id = index;
  viz_marker.type = visualization_msgs::msg::Marker::SPHERE;
  viz_marker.action = visualization_msgs::msg::Marker::ADD;
  viz_marker.pose.position.x = translation.x;
  viz_marker.pose.position.y = translation.y;
  viz_marker.pose.position.z = translation.z;
  viz_marker.pose.orientation.x = 0.0f;
  viz_marker.pose.orientation.y = 0.0f;
  viz_marker.pose.orientation.z = 0.0f;
  viz_marker.pose.orientation.w = 1.0f;
  viz_marker.scale = marker_scale_;
  viz_marker.lifetime = rclcpp::Duration(1s);
  return viz_marker;
}


void
MarkerVisualizer::rb_callback(const mocap_msgs::msg::RigidBody::SharedPtr msg) const
{
  if (publisher_rb_->get_subscription_count() == 0) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped visual_rb;
  visual_rb.header = msg->header;
  visual_rb.header.frame_id = marker_frame_;
  visual_rb.pose.pose = msg->pose;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      visual_rb.pose.covariance[i * 6 + j] = 0.0;
    }
  }
  publisher_rb_->publish(visual_rb);
}
