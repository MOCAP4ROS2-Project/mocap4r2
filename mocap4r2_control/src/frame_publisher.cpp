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

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mocap4r2_control/frame_publisher.hpp"

namespace mocap4r2_control
{

namespace
{

template<typename T>
T get_or_declare(MocapSystemNode & node, const std::string & name, const T & def)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, def);
  }
  return node.get_parameter(name).get_value<T>();
}

}  // namespace

void
declare_frame_publisher_parameters(MocapSystemNode & node)
{
  get_or_declare<bool>(node, "publish_markers", true);
  get_or_declare<bool>(node, "publish_rigid_bodies", true);
  get_or_declare<bool>(node, "publish_skeletons", true);
  get_or_declare<bool>(node, "publish_tf", false);
  get_or_declare<std::string>(node, "tf_prefix", "");
}

FramePublisher::FramePublisher(MocapSystemNode & node)
: node_(node)
{
  declare_frame_publisher_parameters(node);
  using mocap4r2_interfaces::msg::Markers;
  using mocap4r2_interfaces::msg::RigidBodies;
  using mocap4r2_interfaces::msg::Skeletons;
  if (node.get_parameter("publish_markers").as_bool()) {
    markers_pub_ = node.create_data_publisher<Markers>("markers");
  }
  if (node.get_parameter("publish_rigid_bodies").as_bool()) {
    bodies_pub_ = node.create_data_publisher<RigidBodies>("rigid_bodies");
  }
  if (node.get_parameter("publish_skeletons").as_bool()) {
    skeletons_pub_ = node.create_data_publisher<Skeletons>("skeletons");
  }
  if (node.get_parameter("publish_tf").as_bool()) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
  tf_prefix_ = node.get_parameter("tf_prefix").as_string();
}

bool
FramePublisher::publish(MocapFrame && frame)
{
  if (!node_.is_active()) {
    return false;
  }

  const auto now = node_.now();
  const double latency =
    frame.capture_time.get_clock_type() == now.get_clock_type() ?
    (now - frame.capture_time).seconds() : 0.0;
  node_.stats().add_frame(frame.frame_number, latency);
  for (const auto & rb : frame.rigid_bodies) {
    node_.stats().add_tracking(rb.name, rb.tracking_valid);
  }
  for (const auto & sk : frame.skeletons) {
    node_.stats().add_tracking(sk.name, sk.tracking_valid);
  }

  std_msgs::msg::Header header;
  header.stamp = frame.capture_time;
  header.frame_id = node_.frame_id();

  if (tf_broadcaster_) {
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    for (const auto & rb : frame.rigid_bodies) {
      if (!rb.tracking_valid || rb.name.empty()) {
        continue;
      }
      geometry_msgs::msg::TransformStamped t;
      t.header = header;
      t.child_frame_id = tf_prefix_ + rb.name;
      t.transform.translation.x = rb.pose.position.x;
      t.transform.translation.y = rb.pose.position.y;
      t.transform.translation.z = rb.pose.position.z;
      t.transform.rotation = rb.pose.orientation;
      tfs.push_back(t);
    }
    if (!tfs.empty()) {
      tf_broadcaster_->sendTransform(tfs);
    }
  }

  if (markers_pub_) {
    mocap4r2_interfaces::msg::Markers msg;
    msg.header = header;
    msg.frame_number = frame.frame_number;
    msg.markers = std::move(frame.markers);
    markers_pub_->publish(msg);
  }
  if (bodies_pub_) {
    mocap4r2_interfaces::msg::RigidBodies msg;
    msg.header = header;
    msg.frame_number = frame.frame_number;
    msg.rigid_bodies = std::move(frame.rigid_bodies);
    bodies_pub_->publish(msg);
  }
  if (skeletons_pub_ && !frame.skeletons.empty()) {
    mocap4r2_interfaces::msg::Skeletons msg;
    msg.header = header;
    msg.frame_number = frame.frame_number;
    msg.skeletons = std::move(frame.skeletons);
    skeletons_pub_->publish(msg);
  }
  return true;
}

}  // namespace mocap4r2_control
