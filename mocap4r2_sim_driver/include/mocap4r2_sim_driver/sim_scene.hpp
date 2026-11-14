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

#ifndef MOCAP4R2_SIM_DRIVER__SIM_SCENE_HPP_
#define MOCAP4R2_SIM_DRIVER__SIM_SCENE_HPP_

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "mocap4r2_interfaces/msg/marker.hpp"
#include "mocap4r2_interfaces/msg/rigid_body.hpp"

namespace mocap4r2_sim_driver
{

struct BodyConfig
{
  std::string name;
  std::string motion{"circle"};  // circle | figure8 | static
  std::array<double, 3> center{0.0, 0.0, 0.5};
  double radius{1.0};
  double period{10.0};
};

struct SceneConfig
{
  std::vector<BodyConfig> bodies;
  int markers_per_body{4};
  int unlabeled_markers{0};
  double noise{0.0};                  // marker noise std, metres
  double occlusion_probability{0.0};  // per body and frame, start of an occlusion
  double occlusion_duration{0.2};     // seconds
  unsigned int seed{42};
};

struct SceneFrame
{
  std::vector<mocap4r2_interfaces::msg::Marker> markers;
  std::vector<mocap4r2_interfaces::msg::RigidBody> rigid_bodies;
};

// Deterministic (given the seed) synthetic mocap scene.
class SimScene
{
public:
  static constexpr int kMaxMarkersPerBody = 6;

  explicit SimScene(const SceneConfig & config);

  // Frame at time t (seconds since start).
  SceneFrame generate(double t);

  // Ground truth pose of a body at time t: x, y, z, yaw.
  static std::array<double, 4> body_pose(const BodyConfig & body, double t);

  // Change the occlusion probability (e.g. from a parameter update).
  void set_occlusion_probability(double p);

  const SceneConfig & config() const {return config_;}

private:
  SceneConfig config_;
  std::mt19937 rng_;
  std::vector<double> occluded_until_;
  std::vector<mocap4r2_interfaces::msg::RigidBody> last_bodies_;
};

}  // namespace mocap4r2_sim_driver

#endif  // MOCAP4R2_SIM_DRIVER__SIM_SCENE_HPP_
