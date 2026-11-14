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

#ifndef MOCAP4R2_SIM_DRIVER__SIM_DRIVER_HPP_
#define MOCAP4R2_SIM_DRIVER__SIM_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_sim_driver/sim_scene.hpp"

namespace mocap4r2_sim_driver
{

// Simulated mocap system: synthetic bodies and markers produced by an
// acquisition thread, like a real SDK, with optional noise, occlusions,
// dropped frames and latency.
class SimDriver : public mocap4r2_control::MocapSystemNode
{
public:
  explicit SimDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SimDriver() override;

protected:
  CallbackReturn on_system_configure() override;
  CallbackReturn on_system_activate() override;
  CallbackReturn on_system_deactivate() override;
  CallbackReturn on_system_cleanup() override;
  double nominal_frame_rate() const override {return rate_;}
  std::string server() const override {return "simulated scene";}
  bool connected() const override {return scene_ != nullptr;}

private:
  void acquisition_loop();
  void stop_thread();

  double rate_{120.0};
  double latency_{0.0};
  double drop_probability_{0.0};
  std::unique_ptr<SimScene> scene_;
  std::unique_ptr<mocap4r2_control::FramePublisher> publisher_;
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace mocap4r2_sim_driver

#endif  // MOCAP4R2_SIM_DRIVER__SIM_DRIVER_HPP_
