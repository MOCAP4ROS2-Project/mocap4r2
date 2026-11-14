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

#ifndef MOCAP4R2_CONTROL__AUXILIAR_SYSTEM_HPP_
#define MOCAP4R2_CONTROL__AUXILIAR_SYSTEM_HPP_

#include "mocap4r2_control/mocap_system_node.hpp"

namespace mocap4r2_control
{

// A system without data of its own (EMG, cameras, ...) that joins capture sessions.
// Parameter "topics": entries "topic" or "topic:pkg/msg/Type" announced as its data.
class AuxiliarSystem : public MocapSystemNode
{
public:
  explicit AuxiliarSystem(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_system_configure() override;
  void on_control_start(const Control & request) override;
  void on_control_stop(const Control & request) override;
};

}  // namespace mocap4r2_control

#endif  // MOCAP4R2_CONTROL__AUXILIAR_SYSTEM_HPP_
