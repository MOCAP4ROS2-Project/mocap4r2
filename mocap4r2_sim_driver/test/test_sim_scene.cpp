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
#include <stdexcept>
#include <string>

#include "mocap4r2_sim_driver/sim_scene.hpp"

using mocap4r2_sim_driver::BodyConfig;
using mocap4r2_sim_driver::SceneConfig;
using mocap4r2_sim_driver::SimScene;

namespace
{

SceneConfig two_bodies()
{
  SceneConfig c;
  BodyConfig a;
  a.name = "robot";
  a.motion = "circle";
  a.center = {1.0, 2.0, 0.1};
  a.radius = 1.0;
  a.period = 4.0;
  BodyConfig b;
  b.name = "wand";
  b.motion = "static";
  b.center = {0.0, 0.0, 1.0};
  c.bodies = {a, b};
  return c;
}

double dist(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & q)
{
  return std::hypot(p.x - q.x, p.y - q.y, p.z - q.z);
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return 2.0 * std::atan2(q.z, q.w);
}

}  // namespace

TEST(SimScene, circle_pose_and_heading)
{
  BodyConfig b;
  b.motion = "circle";
  b.center = {1.0, 2.0, 0.5};
  b.radius = 2.0;
  b.period = 4.0;
  auto p = SimScene::body_pose(b, 0.0);
  EXPECT_NEAR(p[0], 3.0, 1e-12);
  EXPECT_NEAR(p[1], 2.0, 1e-12);
  EXPECT_NEAR(p[2], 0.5, 1e-12);
  EXPECT_NEAR(p[3], M_PI_2, 1e-12);  // moving towards +y
  p = SimScene::body_pose(b, 1.0);   // quarter turn
  EXPECT_NEAR(p[0], 1.0, 1e-12);
  EXPECT_NEAR(p[1], 4.0, 1e-12);
  EXPECT_NEAR(std::abs(p[3]), M_PI, 1e-12);  // moving towards -x
}

TEST(SimScene, figure8_and_static)
{
  BodyConfig b;
  b.motion = "figure8";
  b.center = {0.0, 0.0, 1.0};
  b.radius = 1.0;
  b.period = 2.0 * M_PI;  // w = 1
  auto p = SimScene::body_pose(b, 0.0);
  EXPECT_NEAR(p[0], 0.0, 1e-12);
  EXPECT_NEAR(p[1], 0.0, 1e-12);
  EXPECT_NEAR(p[3], M_PI_4, 1e-12);
  b.motion = "static";
  p = SimScene::body_pose(b, 123.0);
  EXPECT_EQ(p[0], 0.0);
  EXPECT_EQ(p[2], 1.0);
  EXPECT_EQ(p[3], 0.0);
}

TEST(SimScene, bodies_markers_and_ids)
{
  auto c = two_bodies();
  c.unlabeled_markers = 3;
  SimScene scene(c);
  const auto f = scene.generate(0.5);
  ASSERT_EQ(f.rigid_bodies.size(), 2u);
  EXPECT_EQ(f.rigid_bodies[0].name, "robot");
  EXPECT_EQ(f.rigid_bodies[0].id, 1);
  EXPECT_EQ(f.rigid_bodies[1].id, 2);
  EXPECT_TRUE(f.rigid_bodies[0].tracking_valid);
  ASSERT_EQ(f.markers.size(), 2u * 4u + 3u);

  std::set<int32_t> ids;
  int unlabeled = 0;
  for (const auto & m : f.markers) {
    if (m.id == mocap4r2_interfaces::msg::Marker::UNKNOWN_ID) {
      unlabeled++;
      EXPECT_TRUE(m.name.empty());
    } else {
      EXPECT_TRUE(ids.insert(m.id).second) << "duplicated id " << m.id;
    }
  }
  EXPECT_EQ(unlabeled, 3);
  // Ids are stable from frame to frame.
  const auto g = scene.generate(0.6);
  for (size_t k = 0; k < 8; k++) {
    EXPECT_EQ(f.markers[k].id, g.markers[k].id);
    EXPECT_EQ(f.markers[k].name, g.markers[k].name);
  }
}

TEST(SimScene, markers_move_rigidly_with_the_body)
{
  SimScene scene(two_bodies());
  const auto a = scene.generate(0.0);
  const auto b = scene.generate(1.3);
  const auto & ma = a.rigid_bodies[0].markers;
  const auto & mb = b.rigid_bodies[0].markers;
  ASSERT_EQ(ma.size(), 4u);
  for (size_t i = 0; i < ma.size(); i++) {
    for (size_t j = i + 1; j < ma.size(); j++) {
      EXPECT_NEAR(dist(ma[i].position, ma[j].position), dist(mb[i].position, mb[j].position), 1e-9);
    }
  }
  // Pose orientation is a unit quaternion consistent with the heading.
  const auto & q = b.rigid_bodies[0].pose.orientation;
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
  const auto p = SimScene::body_pose(two_bodies().bodies[0], 1.3);
  EXPECT_NEAR(std::remainder(yaw_of(q) - p[3], 2.0 * M_PI), 0.0, 1e-9);
}

TEST(SimScene, deterministic_given_seed)
{
  auto c = two_bodies();
  c.noise = 0.01;
  c.unlabeled_markers = 2;
  c.occlusion_probability = 0.3;
  SimScene s1(c);
  SimScene s2(c);
  for (int i = 0; i < 50; i++) {
    const auto a = s1.generate(i * 0.01);
    const auto b = s2.generate(i * 0.01);
    ASSERT_EQ(a.markers.size(), b.markers.size());
    for (size_t k = 0; k < a.markers.size(); k++) {
      EXPECT_EQ(a.markers[k].position.x, b.markers[k].position.x);
    }
    EXPECT_EQ(a.rigid_bodies[0].tracking_valid, b.rigid_bodies[0].tracking_valid);
  }
}

TEST(SimScene, occlusion_keeps_last_pose_and_flags_invalid)
{
  auto c = two_bodies();
  c.occlusion_duration = 10.0;
  SimScene scene(c);
  const auto valid = scene.generate(0.0);
  ASSERT_TRUE(valid.rigid_bodies[0].tracking_valid);

  scene.set_occlusion_probability(1.0);
  const auto f = scene.generate(0.5);
  const auto & rb = f.rigid_bodies[0];
  EXPECT_FALSE(rb.tracking_valid);
  EXPECT_EQ(rb.name, "robot");
  EXPECT_EQ(rb.id, 1);
  EXPECT_LT(rb.mean_error, 0.0f);
  // Last valid pose and markers, flagged.
  EXPECT_EQ(rb.pose, valid.rigid_bodies[0].pose);
  ASSERT_EQ(rb.markers.size(), valid.rigid_bodies[0].markers.size());
  for (size_t k = 0; k < rb.markers.size(); k++) {
    EXPECT_TRUE(rb.markers[k].occluded);
    EXPECT_EQ(rb.markers[k].position, valid.rigid_bodies[0].markers[k].position);
  }

  // Occluded from the very first frame: invalid, no markers known.
  SimScene fresh(c);
  fresh.set_occlusion_probability(1.0);
  const auto g = fresh.generate(0.0);
  EXPECT_FALSE(g.rigid_bodies[0].tracking_valid);
  EXPECT_TRUE(g.rigid_bodies[0].markers.empty());

  EXPECT_THROW(scene.set_occlusion_probability(-0.1), std::invalid_argument);
}

TEST(SimScene, occlusion_bursts_end)
{
  auto c = two_bodies();
  c.occlusion_probability = 0.05;
  c.occlusion_duration = 0.1;
  SimScene scene(c);
  int valid = 0;
  int invalid = 0;
  bool recovered = false;
  bool was_invalid = false;
  for (int i = 0; i < 2000; i++) {
    const auto f = scene.generate(i * 0.01);
    const auto & rb = f.rigid_bodies[0];
    if (rb.tracking_valid) {
      valid++;
      recovered |= was_invalid;
    } else {
      invalid++;
      for (const auto & m : rb.markers) {
        EXPECT_TRUE(m.occluded);
      }
    }
    was_invalid = !rb.tracking_valid;
  }
  EXPECT_GT(valid, 0);
  EXPECT_GT(invalid, 0);
  EXPECT_TRUE(recovered);
}

TEST(SimScene, invalid_configs_throw)
{
  auto c = two_bodies();
  c.markers_per_body = SimScene::kMaxMarkersPerBody + 1;
  EXPECT_THROW(SimScene{c}, std::invalid_argument);
  c = two_bodies();
  c.markers_per_body = -1;
  EXPECT_THROW(SimScene{c}, std::invalid_argument);
  c = two_bodies();
  c.noise = -0.1;
  EXPECT_THROW(SimScene{c}, std::invalid_argument);
  c = two_bodies();
  c.occlusion_probability = 1.5;
  EXPECT_THROW(SimScene{c}, std::invalid_argument);
  c = two_bodies();
  c.bodies[0].motion = "spiral";
  EXPECT_THROW(SimScene{c}, std::invalid_argument);
  c = two_bodies();
  c.bodies[0].period = 0.0;
  EXPECT_THROW(SimScene{c}, std::invalid_argument);
  c = two_bodies();
  c.bodies[1].period = 0.0;  // static bodies do not need a period
  EXPECT_NO_THROW(SimScene{c});
  c = two_bodies();
  c.markers_per_body = 0;
  SimScene empty(c);
  EXPECT_TRUE(empty.generate(0.0).markers.empty());
}

TEST(SimScene, no_bodies)
{
  SceneConfig c;
  c.unlabeled_markers = 1;
  SimScene scene(c);
  const auto f = scene.generate(0.0);
  EXPECT_TRUE(f.rigid_bodies.empty());
  EXPECT_EQ(f.markers.size(), 1u);
}
