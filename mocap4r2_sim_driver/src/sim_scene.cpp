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

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "mocap4r2_sim_driver/sim_scene.hpp"

namespace mocap4r2_sim_driver
{

namespace
{

// Asymmetric marker pattern in the body frame (no two bodies need the same).
constexpr std::array<std::array<double, 3>, SimScene::kMaxMarkersPerBody> kPattern{{
  {0.10, 0.00, 0.00}, {-0.05, 0.07, 0.00}, {-0.05, -0.04, 0.02},
  {0.00, 0.00, 0.06}, {0.03, 0.05, 0.03}, {-0.08, 0.00, 0.04}}};

}  // namespace

SimScene::SimScene(const SceneConfig & config)
: config_(config), rng_(config.seed),
  occluded_until_(config.bodies.size(), -1.0),
  last_bodies_(config.bodies.size())
{
  if (config_.markers_per_body < 0 || config_.markers_per_body > kMaxMarkersPerBody) {
    throw std::invalid_argument(
            "markers_per_body must be in [0, " + std::to_string(kMaxMarkersPerBody) + "]");
  }
  if (config_.unlabeled_markers < 0) {
    throw std::invalid_argument("unlabeled_markers must be >= 0");
  }
  if (config_.noise < 0.0) {
    throw std::invalid_argument("noise must be >= 0");
  }
  if (config_.occlusion_probability < 0.0 || config_.occlusion_probability > 1.0) {
    throw std::invalid_argument("occlusion_probability must be in [0, 1]");
  }
  for (const auto & b : config_.bodies) {
    if (b.motion != "circle" && b.motion != "figure8" && b.motion != "static") {
      throw std::invalid_argument("unknown motion '" + b.motion + "' of body " + b.name);
    }
    if (b.motion != "static" && b.period <= 0.0) {
      throw std::invalid_argument("period of body " + b.name + " must be > 0");
    }
  }
}

void
SimScene::set_occlusion_probability(double p)
{
  if (p < 0.0 || p > 1.0) {
    throw std::invalid_argument("occlusion_probability must be in [0, 1]");
  }
  config_.occlusion_probability = p;
}

std::array<double, 4>
SimScene::body_pose(const BodyConfig & body, double t)
{
  const auto & c = body.center;
  if (body.motion == "static") {
    return {c[0], c[1], c[2], 0.0};
  }
  const double w = 2.0 * M_PI / body.period;
  if (body.motion == "circle") {
    return {c[0] + body.radius * std::cos(w * t), c[1] + body.radius * std::sin(w * t), c[2],
      std::atan2(std::sin(w * t + M_PI_2), std::cos(w * t + M_PI_2))};
  }
  // figure8: x = r sin(wt), y = r sin(wt) cos(wt); heading follows the velocity
  const double x = body.radius * std::sin(w * t);
  const double y = body.radius * std::sin(w * t) * std::cos(w * t);
  const double dx = body.radius * w * std::cos(w * t);
  const double dy = body.radius * w * std::cos(2.0 * w * t);
  return {c[0] + x, c[1] + y, c[2], std::atan2(dy, dx)};
}

SceneFrame
SimScene::generate(double t)
{
  SceneFrame frame;
  std::normal_distribution<double> noise(0.0, config_.noise);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  auto jitter = [&]() {return config_.noise > 0.0 ? noise(rng_) : 0.0;};

  for (size_t i = 0; i < config_.bodies.size(); i++) {
    const auto & cfg = config_.bodies[i];
    if (t >= occluded_until_[i] && config_.occlusion_probability > 0.0 &&
      unit(rng_) < config_.occlusion_probability)
    {
      occluded_until_[i] = t + config_.occlusion_duration;
    }
    const bool occluded = t < occluded_until_[i];

    mocap4r2_interfaces::msg::RigidBody rb;
    rb.id = static_cast<int32_t>(i + 1);
    rb.name = cfg.name;
    if (occluded) {
      // Pose of an untracked body is the last valid one, flagged invalid.
      rb = last_bodies_[i];
      rb.id = static_cast<int32_t>(i + 1);
      rb.name = cfg.name;
      rb.tracking_valid = false;
      rb.mean_error = -1.0f;
      for (auto & m : rb.markers) {
        m.occluded = true;
        m.residual = -1.0f;
      }
    } else {
      const auto p = body_pose(cfg, t);
      const double yaw = p[3];
      rb.tracking_valid = true;
      rb.pose.position.x = p[0];
      rb.pose.position.y = p[1];
      rb.pose.position.z = p[2];
      rb.pose.orientation.z = std::sin(yaw / 2.0);
      rb.pose.orientation.w = std::cos(yaw / 2.0);
      rb.mean_error = static_cast<float>(config_.noise);
      for (int k = 0; k < config_.markers_per_body; k++) {
        const auto & o = kPattern[k];
        mocap4r2_interfaces::msg::Marker m;
        m.id = static_cast<int32_t>((i + 1) * 100 + k + 1);
        m.name = cfg.name + ":m" + std::to_string(k + 1);
        m.position.x = p[0] + std::cos(yaw) * o[0] - std::sin(yaw) * o[1] + jitter();
        m.position.y = p[1] + std::sin(yaw) * o[0] + std::cos(yaw) * o[1] + jitter();
        m.position.z = p[2] + o[2] + jitter();
        m.occluded = false;
        m.residual = static_cast<float>(config_.noise);
        rb.markers.push_back(m);
      }
      last_bodies_[i] = rb;
    }
    frame.markers.insert(frame.markers.end(), rb.markers.begin(), rb.markers.end());
    frame.rigid_bodies.push_back(rb);
  }

  std::uniform_real_distribution<double> xy(-2.0, 2.0);
  std::uniform_real_distribution<double> z(0.0, 2.0);
  for (int k = 0; k < config_.unlabeled_markers; k++) {
    mocap4r2_interfaces::msg::Marker m;
    m.id = mocap4r2_interfaces::msg::Marker::UNKNOWN_ID;
    m.position.x = xy(rng_);
    m.position.y = xy(rng_);
    m.position.z = z(rng_);
    m.residual = -1.0f;
    frame.markers.push_back(m);
  }
  return frame;
}

}  // namespace mocap4r2_sim_driver
