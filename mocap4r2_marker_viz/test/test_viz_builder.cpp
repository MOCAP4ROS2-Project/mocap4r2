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

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <utility>

#include "mocap4r2_marker_viz/viz_builder.hpp"

using mocap4r2_interfaces::msg::Marker;
using mocap4r2_interfaces::msg::Markers;
using mocap4r2_interfaces::msg::RigidBodies;
using mocap4r2_interfaces::msg::RigidBody;
using mocap4r2_interfaces::msg::Segment;
using mocap4r2_interfaces::msg::Skeleton;
using mocap4r2_interfaces::msg::Skeletons;
using mocap4r2_marker_viz::VizBuilder;
using VizMarker = visualization_msgs::msg::Marker;

namespace
{

Marker mk(int32_t id, const std::string & name, double x, bool occluded = false)
{
  Marker m;
  m.id = id;
  m.name = name;
  m.position.x = x;
  m.occluded = occluded;
  return m;
}

const VizMarker * find(
  const visualization_msgs::msg::MarkerArray & a, const std::string & ns, int id)
{
  for (const auto & m : a.markers) {
    if (m.ns == ns && m.id == id && m.action == VizMarker::ADD) {
      return &m;
    }
  }
  return nullptr;
}

}  // namespace

TEST(VizBuilder, empty_input_only_clears)
{
  VizBuilder b;
  const auto a = b.build(nullptr, nullptr, nullptr);
  ASSERT_EQ(a.markers.size(), 1u);
  EXPECT_EQ(a.markers[0].action, VizMarker::DELETEALL);
}

TEST(VizBuilder, markers_grouped_by_category)
{
  Markers msg;
  msg.header.frame_id = "mocap";
  msg.markers = {mk(1, "a:1", 0.0), mk(2, "a:2", 1.0), mk(Marker::UNKNOWN_ID, "", 2.0),
    mk(3, "a:3", 3.0, true)};
  VizBuilder b;
  const auto a = b.build(&msg, nullptr, nullptr);
  EXPECT_EQ(a.markers[0].action, VizMarker::DELETEALL);
  EXPECT_EQ(a.markers[0].header.frame_id, "mocap");

  const auto * labeled = find(a, "markers", 0);
  const auto * unlabeled = find(a, "markers", 1);
  const auto * occluded = find(a, "markers", 2);
  ASSERT_TRUE(labeled && unlabeled && occluded);
  EXPECT_EQ(labeled->type, VizMarker::SPHERE_LIST);
  EXPECT_EQ(labeled->points.size(), 2u);
  EXPECT_EQ(unlabeled->points.size(), 1u);
  EXPECT_EQ(occluded->points.size(), 1u);
  EXPECT_EQ(labeled->header.frame_id, "mocap");
  EXPECT_DOUBLE_EQ(labeled->scale.x, 0.014);
  EXPECT_LT(occluded->color.a, 1.0f);
}

TEST(VizBuilder, empty_categories_are_omitted)
{
  Markers msg;
  msg.markers = {mk(1, "a", 0.0)};
  VizBuilder b;
  const auto a = b.build(&msg, nullptr, nullptr);
  EXPECT_EQ(a.markers.size(), 2u);  // DELETEALL + labeled
  EXPECT_EQ(find(a, "markers", 1), nullptr);
}

TEST(VizBuilder, ids_are_stable_between_frames)
{
  RigidBodies msg;
  RigidBody rb;
  rb.name = "robot";
  rb.tracking_valid = true;
  rb.pose.orientation.w = 1.0;
  msg.rigid_bodies = {rb, rb};
  msg.rigid_bodies[1].name = "wand";
  VizBuilder b;
  const auto a1 = b.build(nullptr, &msg, nullptr);
  const auto a2 = b.build(nullptr, &msg, nullptr);
  ASSERT_EQ(a1.markers.size(), a2.markers.size());
  std::set<std::pair<std::string, int>> keys;
  for (size_t i = 0; i < a1.markers.size(); i++) {
    EXPECT_EQ(a1.markers[i].id, a2.markers[i].id);
    EXPECT_EQ(a1.markers[i].ns, a2.markers[i].ns);
    if (a1.markers[i].action == VizMarker::ADD) {
      EXPECT_TRUE(keys.insert({a1.markers[i].ns, a1.markers[i].id}).second) << "duplicate id";
      EXPECT_EQ(a1.markers[i].lifetime.sec, 0);  // persistent until next DELETEALL
    }
  }
}

TEST(VizBuilder, body_axes_follow_orientation)
{
  RigidBodies msg;
  RigidBody rb;
  rb.name = "robot";
  rb.tracking_valid = true;
  rb.pose.position.x = 1.0;
  // 90 degrees around z
  rb.pose.orientation.z = std::sin(M_PI_4);
  rb.pose.orientation.w = std::cos(M_PI_4);
  msg.rigid_bodies = {rb};
  mocap4r2_marker_viz::VizConfig cfg;
  cfg.axis_length = 0.5;
  VizBuilder b(cfg);
  const auto a = b.build(nullptr, &msg, nullptr);
  const auto * axes = find(a, "rigid_bodies", 0);
  ASSERT_NE(axes, nullptr);
  ASSERT_EQ(axes->points.size(), 6u);
  ASSERT_EQ(axes->colors.size(), 6u);
  // x axis of the body points to +y of the world
  EXPECT_NEAR(axes->points[1].x, 1.0, 1e-9);
  EXPECT_NEAR(axes->points[1].y, 0.5, 1e-9);
  // y axis points to -x
  EXPECT_NEAR(axes->points[3].x, 0.5, 1e-9);
  EXPECT_NEAR(axes->points[3].y, 0.0, 1e-9);
  // z axis unchanged
  EXPECT_NEAR(axes->points[5].z, 0.5, 1e-9);
  const auto * name = find(a, "rigid_body_names", 0);
  ASSERT_NE(name, nullptr);
  EXPECT_EQ(name->text, "robot");
}

TEST(VizBuilder, lost_bodies_are_marked)
{
  RigidBodies msg;
  RigidBody rb;
  rb.name = "robot";
  rb.tracking_valid = false;
  rb.pose.orientation.w = 1.0;
  msg.rigid_bodies = {rb};
  VizBuilder b;
  const auto a = b.build(nullptr, &msg, nullptr);
  const auto * name = find(a, "rigid_body_names", 0);
  ASSERT_NE(name, nullptr);
  EXPECT_EQ(name->text, "robot (lost)");
  const auto * axes = find(a, "rigid_bodies", 0);
  EXPECT_FLOAT_EQ(axes->colors[0].r, axes->colors[0].g);  // gray
}

TEST(VizBuilder, names_can_be_hidden_and_unnamed_bodies_have_no_text)
{
  RigidBodies msg;
  RigidBody rb;
  rb.tracking_valid = true;
  rb.pose.orientation.w = 1.0;
  msg.rigid_bodies = {rb};
  VizBuilder b;
  EXPECT_EQ(find(b.build(nullptr, &msg, nullptr), "rigid_body_names", 0), nullptr);
  msg.rigid_bodies[0].name = "x";
  mocap4r2_marker_viz::VizConfig cfg;
  cfg.show_names = false;
  VizBuilder hidden(cfg);
  EXPECT_EQ(find(hidden.build(nullptr, &msg, nullptr), "rigid_body_names", 0), nullptr);
}

TEST(VizBuilder, skeleton_bones_connect_to_parents)
{
  Skeletons msg;
  Skeleton sk;
  sk.name = "subject";
  sk.tracking_valid = true;
  auto seg = [](int32_t id, int32_t parent, double z) {
      Segment s;
      s.id = id;
      s.parent_id = parent;
      s.pose.position.z = z;
      s.pose.orientation.w = 1.0;
      return s;
    };
  // Root, child, grandchild and a segment whose parent is missing.
  sk.segments = {seg(1, Segment::NO_PARENT, 1.0), seg(2, 1, 1.5), seg(3, 2, 1.8), seg(4, 99, 0.2)};
  msg.skeletons = {sk};
  VizBuilder b;
  const auto a = b.build(nullptr, nullptr, &msg);
  const auto * bones = find(a, "skeletons", 0);
  const auto * joints = find(a, "skeletons", 1);
  ASSERT_TRUE(bones && joints);
  EXPECT_EQ(bones->type, VizMarker::LINE_LIST);
  EXPECT_EQ(bones->points.size(), 4u);   // 1-2 and 2-3
  EXPECT_EQ(joints->points.size(), 4u);
  EXPECT_DOUBLE_EQ(bones->points[0].z, 1.0);
  EXPECT_DOUBLE_EQ(bones->points[1].z, 1.5);
}
