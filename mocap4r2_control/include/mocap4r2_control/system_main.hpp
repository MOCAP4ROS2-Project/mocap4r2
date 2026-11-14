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

#ifndef MOCAP4R2_CONTROL__SYSTEM_MAIN_HPP_
#define MOCAP4R2_CONTROL__SYSTEM_MAIN_HPP_

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_control
{

// Common main of mocap systems. Parameter "autostart":
//   "none"      stay unconfigured,
//   "configure" configure and wait for START (default),
//   "activate"  configure and activate.
template<typename SystemT>
int system_main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  {
    auto node = std::make_shared<SystemT>(rclcpp::NodeOptions());
    const auto autostart = node->template declare_parameter<std::string>("autostart", "configure");
    using lifecycle_msgs::msg::State;
    using lifecycle_msgs::msg::Transition;
    if (autostart == "configure" || autostart == "activate") {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
      if (autostart == "activate" &&
        node->get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
      {
        node->trigger_transition(Transition::TRANSITION_ACTIVATE);
      }
    } else if (autostart != "none") {
      RCLCPP_ERROR(
        node->get_logger(), "autostart must be none, configure or activate (got '%s')",
        autostart.c_str());
      ret = 1;
    }
    if (ret == 0) {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      executor.spin();
    }
    if (node->get_current_state().id() != State::PRIMARY_STATE_FINALIZED) {
      node->shutdown();
    }
  }
  rclcpp::shutdown();
  return ret;
}

}  // namespace mocap4r2_control

#endif  // MOCAP4R2_CONTROL__SYSTEM_MAIN_HPP_
