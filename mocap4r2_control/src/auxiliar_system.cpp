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

#include <string>
#include <vector>

#include "mocap4r2_control/auxiliar_system.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mocap4r2_control
{

AuxiliarSystem::AuxiliarSystem(const rclcpp::NodeOptions & options)
: MocapSystemNode("auxiliar_system", "auxiliar", options)
{
  declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>());
}

AuxiliarSystem::CallbackReturn
AuxiliarSystem::on_system_configure()
{
  const auto entries = get_parameter("topics").as_string_array();
  for (const auto & entry : entries) {
    const auto sep = entry.find(':');
    const auto topic = entry.substr(0, sep);
    const auto type = sep == std::string::npos ? std::string() : entry.substr(sep + 1);
    if (topic.empty()) {
      RCLCPP_ERROR(get_logger(), "empty topic in parameter topics");
      return CallbackReturn::FAILURE;
    }
    register_topic(get_node_topics_interface()->resolve_topic_name(topic), type);
  }
  return CallbackReturn::SUCCESS;
}

void
AuxiliarSystem::on_control_start(const Control & request)
{
  RCLCPP_INFO(get_logger(), "started (session '%s')", request.session_id.c_str());
}

void
AuxiliarSystem::on_control_stop(const Control & request)
{
  RCLCPP_INFO(get_logger(), "stopped (session '%s')", request.session_id.c_str());
}

}  // namespace mocap4r2_control

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_control::AuxiliarSystem)
