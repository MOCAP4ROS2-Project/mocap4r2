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

#include <map>
#include <string>

#include "mocap4r2_marker_viz/viz_builder.hpp"

namespace mocap4r2_marker_viz
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker base(const std_msgs::msg::Header & header, const std::string & ns, int id, int type)
{
  Marker m;
  m.header = header;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  return m;
}

// Rotate v by quaternion q.
geometry_msgs::msg::Point rotate(
  const geometry_msgs::msg::Quaternion & q, double x, double y, double z)
{
  // v' = v + 2 w (u x v) + 2 u x (u x v)
  const double ux = q.x, uy = q.y, uz = q.z, w = q.w;
  const double cx = uy * z - uz * y, cy = uz * x - ux * z, cz = ux * y - uy * x;
  geometry_msgs::msg::Point p;
  p.x = x + 2.0 * (w * cx + uy * cz - uz * cy);
  p.y = y + 2.0 * (w * cy + uz * cx - ux * cz);
  p.z = z + 2.0 * (w * cz + ux * cy - uy * cx);
  return p;
}

}  // namespace

MarkerArray
VizBuilder::build(
  const mocap4r2_interfaces::msg::Markers * markers,
  const mocap4r2_interfaces::msg::RigidBodies * bodies,
  const mocap4r2_interfaces::msg::Skeletons * skeletons) const
{
  MarkerArray out;
  Marker clear;
  clear.action = Marker::DELETEALL;
  if (markers) {
    clear.header = markers->header;
  } else if (bodies) {
    clear.header = bodies->header;
  } else if (skeletons) {
    clear.header = skeletons->header;
  }
  out.markers.push_back(clear);

  if (markers) {
    add_markers(*markers, out);
  }
  if (bodies) {
    add_bodies(*bodies, out);
  }
  if (skeletons) {
    add_skeletons(*skeletons, out);
  }
  return out;
}

void
VizBuilder::add_markers(const mocap4r2_interfaces::msg::Markers & msg, MarkerArray & out) const
{
  // One sphere list per category keeps the array small at any marker count.
  Marker labeled = base(msg.header, "markers", 0, Marker::SPHERE_LIST);
  Marker unlabeled = base(msg.header, "markers", 1, Marker::SPHERE_LIST);
  Marker occluded = base(msg.header, "markers", 2, Marker::SPHERE_LIST);
  for (auto * m : {&labeled, &unlabeled, &occluded}) {
    m->scale.x = m->scale.y = m->scale.z = config_.marker_size;
  }
  labeled.color = color(0.1f, 0.9f, 0.2f);
  unlabeled.color = color(0.9f, 0.9f, 0.9f);
  occluded.color = color(0.9f, 0.2f, 0.2f, 0.4f);

  for (const auto & mk : msg.markers) {
    if (mk.occluded) {
      occluded.points.push_back(mk.position);
    } else if (mk.id == mocap4r2_interfaces::msg::Marker::UNKNOWN_ID && mk.name.empty()) {
      unlabeled.points.push_back(mk.position);
    } else {
      labeled.points.push_back(mk.position);
    }
  }
  for (auto * m : {&labeled, &unlabeled, &occluded}) {
    if (!m->points.empty()) {
      out.markers.push_back(*m);
    }
  }
}

void
VizBuilder::add_bodies(const mocap4r2_interfaces::msg::RigidBodies & msg, MarkerArray & out) const
{
  int id = 0;
  for (const auto & rb : msg.rigid_bodies) {
    // Axes as a line list: x red, y green, z blue; gray when not tracked.
    Marker axes = base(msg.header, "rigid_bodies", id++, Marker::LINE_LIST);
    axes.scale.x = config_.axis_length * 0.1;
    const auto & o = rb.pose.position;
    const double L = config_.axis_length;
    const geometry_msgs::msg::Point dirs[3] = {
      rotate(rb.pose.orientation, L, 0, 0), rotate(rb.pose.orientation, 0, L, 0),
      rotate(rb.pose.orientation, 0, 0, L)};
    const std_msgs::msg::ColorRGBA cols[3] = {
      color(0.9f, 0.1f, 0.1f), color(0.1f, 0.9f, 0.1f), color(0.1f, 0.3f, 1.0f)};
    for (int k = 0; k < 3; k++) {
      geometry_msgs::msg::Point tip;
      tip.x = o.x + dirs[k].x;
      tip.y = o.y + dirs[k].y;
      tip.z = o.z + dirs[k].z;
      axes.points.push_back(o);
      axes.points.push_back(tip);
      const auto c = rb.tracking_valid ? cols[k] : color(0.5f, 0.5f, 0.5f, 0.6f);
      axes.colors.push_back(c);
      axes.colors.push_back(c);
    }
    out.markers.push_back(axes);

    if (config_.show_names && !rb.name.empty()) {
      Marker text = base(msg.header, "rigid_body_names", id - 1, Marker::TEXT_VIEW_FACING);
      text.pose.position = o;
      text.pose.position.z += L * 1.3;
      text.scale.z = L * 0.5;
      text.text = rb.tracking_valid ? rb.name : rb.name + " (lost)";
      text.color = rb.tracking_valid ? color(1.0f, 1.0f, 1.0f) : color(0.9f, 0.3f, 0.3f);
      out.markers.push_back(text);
    }
  }
}

void
VizBuilder::add_skeletons(const mocap4r2_interfaces::msg::Skeletons & msg, MarkerArray & out) const
{
  int id = 0;
  for (const auto & sk : msg.skeletons) {
    std::map<int32_t, const geometry_msgs::msg::Point *> by_id;
    for (const auto & seg : sk.segments) {
      by_id[seg.id] = &seg.pose.position;
    }
    Marker bones = base(msg.header, "skeletons", id++, Marker::LINE_LIST);
    bones.scale.x = config_.marker_size;
    bones.color = sk.tracking_valid ? color(0.2f, 0.6f, 1.0f) : color(0.5f, 0.5f, 0.5f, 0.6f);
    Marker joints = base(msg.header, "skeletons", id++, Marker::SPHERE_LIST);
    joints.scale.x = joints.scale.y = joints.scale.z = config_.marker_size * 2.0;
    joints.color = bones.color;
    for (const auto & seg : sk.segments) {
      joints.points.push_back(seg.pose.position);
      const auto parent = by_id.find(seg.parent_id);
      if (seg.parent_id != mocap4r2_interfaces::msg::Segment::NO_PARENT &&
        parent != by_id.end())
      {
        bones.points.push_back(*parent->second);
        bones.points.push_back(seg.pose.position);
      }
    }
    if (!bones.points.empty()) {
      out.markers.push_back(bones);
    }
    if (!joints.points.empty()) {
      out.markers.push_back(joints);
    }
  }
}

}  // namespace mocap4r2_marker_viz
