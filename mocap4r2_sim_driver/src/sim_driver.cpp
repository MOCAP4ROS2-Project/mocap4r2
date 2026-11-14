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

#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "mocap4r2_sim_driver/sim_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mocap4r2_sim_driver
{

SimDriver::SimDriver(const rclcpp::NodeOptions & options)
: MocapSystemNode("sim_driver", "sim", options)
{
  declare_parameter<double>("rate", 120.0);
  declare_parameter<double>("latency", 0.004);
  declare_parameter<double>("frame_drop_probability", 0.0);
  declare_parameter<std::vector<std::string>>("bodies", {"robot", "wand"});
  declare_parameter<int>("markers_per_body", 4);
  declare_parameter<int>("unlabeled_markers", 0);
  declare_parameter<double>("noise", 0.0005);
  declare_parameter<double>("occlusion_probability", 0.0);
  declare_parameter<double>("occlusion_duration", 0.2);
  declare_parameter<int>("seed", 42);
  mocap4r2_control::declare_frame_publisher_parameters(*this);
}

SimDriver::~SimDriver()
{
  stop_thread();
}

SimDriver::CallbackReturn
SimDriver::on_system_configure()
{
  rate_ = get_parameter("rate").as_double();
  latency_ = get_parameter("latency").as_double();
  drop_probability_ = get_parameter("frame_drop_probability").as_double();
  if (rate_ <= 0.0 || rate_ > 10000.0) {
    RCLCPP_ERROR(get_logger(), "rate must be in (0, 10000] Hz");
    return CallbackReturn::FAILURE;
  }
  if (latency_ < 0.0 || drop_probability_ < 0.0 || drop_probability_ > 1.0) {
    RCLCPP_ERROR(get_logger(), "latency must be >= 0 and frame_drop_probability in [0, 1]");
    return CallbackReturn::FAILURE;
  }

  SceneConfig config;
  config.markers_per_body = static_cast<int>(get_parameter("markers_per_body").as_int());
  config.unlabeled_markers = static_cast<int>(get_parameter("unlabeled_markers").as_int());
  config.noise = get_parameter("noise").as_double();
  config.occlusion_probability = get_parameter("occlusion_probability").as_double();
  config.occlusion_duration = get_parameter("occlusion_duration").as_double();
  config.seed = static_cast<unsigned int>(get_parameter("seed").as_int());

  // Per-body parameters: bodies.<name>.{motion,center,radius,period}
  const auto names = get_parameter("bodies").as_string_array();
  for (size_t i = 0; i < names.size(); i++) {
    BodyConfig b;
    b.name = names[i];
    const std::string p = "bodies_config." + b.name + ".";
    auto declare = [this](const std::string & n, auto def) {
        if (!has_parameter(n)) {
          declare_parameter(n, def);
        }
        return get_parameter(n);
      };
    b.motion = declare(p + "motion", std::string(i % 2 ? "figure8" : "circle")).as_string();
    const auto center = declare(
      p + "center", std::vector<double>{0.0, 0.0, 0.3 + 0.5 * i}).as_double_array();
    if (center.size() != 3) {
      RCLCPP_ERROR(get_logger(), "%scenter must have 3 values", p.c_str());
      return CallbackReturn::FAILURE;
    }
    b.center = {center[0], center[1], center[2]};
    b.radius = declare(p + "radius", 1.0 + 0.5 * i).as_double();
    b.period = declare(p + "period", 10.0 + 5.0 * i).as_double();
    config.bodies.push_back(b);
  }

  scene_ = std::make_unique<SimScene>(config);  // throws on invalid values
  publisher_ = std::make_unique<mocap4r2_control::FramePublisher>(*this);
  return CallbackReturn::SUCCESS;
}

SimDriver::CallbackReturn
SimDriver::on_system_activate()
{
  running_ = true;
  thread_ = std::thread(&SimDriver::acquisition_loop, this);
  return CallbackReturn::SUCCESS;
}

SimDriver::CallbackReturn
SimDriver::on_system_deactivate()
{
  stop_thread();
  return CallbackReturn::SUCCESS;
}

SimDriver::CallbackReturn
SimDriver::on_system_cleanup()
{
  stop_thread();
  publisher_.reset();
  scene_.reset();
  return CallbackReturn::SUCCESS;
}

void
SimDriver::stop_thread()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void
SimDriver::acquisition_loop()
{
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clock::duration>(
    std::chrono::duration<double>(1.0 / rate_));
  const auto start = clock::now();
  auto next = start;
  uint64_t frame_number = 0;
  std::mt19937 rng(scene_->config().seed + 1);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  while (running_) {
    next += period;
    std::this_thread::sleep_until(next);
    if (!running_) {
      break;
    }
    frame_number++;
    const double t = std::chrono::duration<double>(clock::now() - start).count();
    auto scene_frame = scene_->generate(t);
    if (drop_probability_ > 0.0 && unit(rng) < drop_probability_) {
      continue;
    }
    mocap4r2_control::MocapFrame frame;
    frame.frame_number = frame_number;
    frame.capture_time = now() - rclcpp::Duration::from_seconds(latency_);
    frame.markers = std::move(scene_frame.markers);
    frame.rigid_bodies = std::move(scene_frame.rigid_bodies);
    publisher_->publish(std::move(frame));
  }
}

}  // namespace mocap4r2_sim_driver

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_sim_driver::SimDriver)
