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
#include <random>
#include <stdexcept>
#include <vector>

#include "mocap4r2_robot_gt/gt_math.hpp"

using mocap4r2_robot_gt::from_pose;
using mocap4r2_robot_gt::from_xyzrpy;
using mocap4r2_robot_gt::origin_offset;
using mocap4r2_robot_gt::to_pose;

namespace
{

void expect_vec(const tf2::Vector3 & v, double x, double y, double z)
{
  EXPECT_NEAR(v.x(), x, 1e-9);
  EXPECT_NEAR(v.y(), y, 1e-9);
  EXPECT_NEAR(v.z(), z, 1e-9);
}

void expect_tf_eq(const tf2::Transform & a, const tf2::Transform & b)
{
  expect_vec(a.getOrigin(), b.getOrigin().x(), b.getOrigin().y(), b.getOrigin().z());
  // q and -q are the same rotation
  EXPECT_NEAR(std::abs(a.getRotation().dot(b.getRotation())), 1.0, 1e-9);
}

}  // namespace

// Regression of the old setEuler(roll, pitch, yaw) bug: each angle is about its own axis.
TEST(GtMath, rpy_angles_rotate_about_fixed_axes)
{
  const tf2::Vector3 x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
  auto yaw = from_xyzrpy({0, 0, 0, 0, 0, M_PI_2});
  expect_vec(yaw * x, 0, 1, 0);
  expect_vec(yaw * z, 0, 0, 1);
  auto roll = from_xyzrpy({0, 0, 0, M_PI_2, 0, 0});
  expect_vec(roll * y, 0, 0, 1);
  expect_vec(roll * x, 1, 0, 0);
  auto pitch = from_xyzrpy({0, 0, 0, 0, M_PI_2, 0});
  expect_vec(pitch * x, 0, 0, -1);
  expect_vec(pitch * y, 0, 1, 0);
}

TEST(GtMath, rpy_composition_order_is_z_y_x)
{
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const double r = 0.3, p = -0.4, yw = 1.1;
  auto t = from_xyzrpy({1, 2, 3, r, p, yw});
  tf2::Quaternion qr, qp, qy;
  qr.setRotation(tf2::Vector3(1, 0, 0), r);
  qp.setRotation(tf2::Vector3(0, 1, 0), p);
  qy.setRotation(tf2::Vector3(0, 0, 1), yw);
  expect_tf_eq(t, tf2::Transform(qy * qp * qr, tf2::Vector3(1, 2, 3)));
}

TEST(GtMath, bad_inputs_throw)
{
  EXPECT_THROW(from_xyzrpy({}), std::invalid_argument);
  EXPECT_THROW(from_xyzrpy({1, 2, 3, 4, 5}), std::invalid_argument);
  EXPECT_THROW(from_xyzrpy({1, 2, 3, 4, 5, 6, 7}), std::invalid_argument);
  geometry_msgs::msg::Pose zero;
  zero.orientation.w = 0.0;
  EXPECT_THROW(from_pose(zero), std::invalid_argument);
}

TEST(GtMath, pose_round_trip_and_normalization)
{
  geometry_msgs::msg::Pose p;
  p.position.x = 1.5;
  p.position.z = -2.0;
  p.orientation.z = 2.0;  // not normalized
  p.orientation.w = 2.0;
  const auto t = from_pose(p);
  EXPECT_NEAR(t.getRotation().length(), 1.0, 1e-12);
  const auto back = to_pose(t);
  EXPECT_DOUBLE_EQ(back.position.x, 1.5);
  EXPECT_DOUBLE_EQ(back.position.z, -2.0);
  EXPECT_NEAR(back.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(back.orientation.w, std::sqrt(0.5), 1e-12);
}

// Regression of the old current.inverse() * wish bug: offset * current must equal wish.
TEST(GtMath, origin_offset_maps_current_to_wish)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  for (int i = 0; i < 100; i++) {
    const auto current = from_xyzrpy({u(rng), u(rng), u(rng), u(rng), u(rng), u(rng)});
    const auto wish = from_xyzrpy({u(rng), u(rng), u(rng), u(rng), u(rng), u(rng)});
    const auto offset = origin_offset(current, wish);
    expect_tf_eq(offset * current, wish);
    // Identity wish puts the origin at the current pose.
    expect_tf_eq(origin_offset(current, tf2::Transform::getIdentity()) * current,
      tf2::Transform::getIdentity());
  }
}

TEST(GtMath, offset_keeps_relative_motion)
{
  // After setting the origin, moving the robot 1 m forward in its own frame
  // moves the ground truth 1 m forward in the origin frame.
  const auto current = from_xyzrpy({2, 1, 0, 0, 0, M_PI_2});
  const auto offset = origin_offset(current, tf2::Transform::getIdentity());
  const auto moved = current * from_xyzrpy({1, 0, 0, 0, 0, 0});
  expect_tf_eq(offset * moved, from_xyzrpy({1, 0, 0, 0, 0, 0}));
}
