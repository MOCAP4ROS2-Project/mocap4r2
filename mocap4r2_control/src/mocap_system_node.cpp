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

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"

namespace mocap4r2_control
{

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

double steady_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

State make_state(uint8_t id, const char * label)
{
  State s;
  s.id = id;
  s.label = label;
  return s;
}

}  // namespace

MocapSystemNode::MocapSystemNode(
  const std::string & node_name, const std::string & system_type,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  system_type_(system_type),
  system_id_(get_node_base_interface()->get_fully_qualified_name())
{
  frame_id_ = declare_parameter<std::string>("frame_id", "mocap");
  declare_parameter<std::string>("qos.reliability", "reliable");
  declare_parameter<int>("qos.depth", 10);

  control_sub_ = create_subscription<Control>(
    kControlTopic, rclcpp::QoS(10).reliable(),
    std::bind(&MocapSystemNode::control_callback, this, std::placeholders::_1));
  ack_pub_ = rclcpp::create_publisher<ControlAck>(
    *this, kControlAckTopic, rclcpp::QoS(10).reliable());
  info_pub_ = rclcpp::create_publisher<MocapSystemInfo>(
    *this, kSystemsTopic, rclcpp::QoS(1).reliable().transient_local());

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID(system_type_);
  diagnostics_->add("frames", this, &MocapSystemNode::produce_diagnostics);

  stats_.reset(steady_now());
  publish_info(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
}

std::string
MocapSystemNode::session_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_id_;
}

rclcpp::QoS
MocapSystemNode::data_qos() const
{
  const auto reliability = get_parameter("qos.reliability").as_string();
  const auto depth = get_parameter("qos.depth").as_int();
  if (depth < 1) {
    throw std::invalid_argument("qos.depth must be >= 1");
  }
  rclcpp::QoS qos(static_cast<size_t>(depth));
  if (reliability == "reliable") {
    qos.reliable();
  } else if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    throw std::invalid_argument("qos.reliability must be 'reliable' or 'best_effort'");
  }
  return qos;
}

void
MocapSystemNode::register_topic(
  const std::string & topic, const std::string & type, std::function<void(bool)> toggle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  topics_.push_back({topic, type, std::move(toggle)});
}

void
MocapSystemNode::announce()
{
  const auto & s = get_current_state();
  publish_info(s.id(), s.label().c_str());
}

void
MocapSystemNode::publish_info(uint8_t state_id, const char * state_label)
{
  MocapSystemInfo info;
  info.system = system_id();
  info.type = system_type_;
  info.frame_id = frame_id_;
  info.frame_rate = nominal_frame_rate();
  info.state = make_state(state_id, state_label);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    info.session_id = session_id_;
    for (const auto & t : topics_) {
      info.topics.push_back(t.name);
      info.topic_types.push_back(t.type);
    }
  }
  // After Ctrl-C the context is gone but the shutdown transition still runs.
  if (!get_node_base_interface()->get_context()->is_valid()) {
    return;
  }
  try {
    info_pub_->publish(info);
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_DEBUG(get_logger(), "cannot announce: %s", e.what());
  }
}

MocapSystemNode::CallbackReturn
MocapSystemNode::guarded(const char * what, const std::function<CallbackReturn()> & hook)
{
  try {
    const auto ret = hook();
    if (ret != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s failed", what);
    }
    return ret;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s failed: %s", what, e.what());
    return CallbackReturn::FAILURE;
  }
}

void
MocapSystemNode::set_publishers_active(bool active)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & t : topics_) {
    if (t.toggle) {
      t.toggle(active);
    }
  }
}

MocapSystemNode::CallbackReturn
MocapSystemNode::on_configure(const rclcpp_lifecycle::State &)
{
  frame_id_ = get_parameter("frame_id").as_string();
  const auto ret = guarded(
    "configure", [this] {
      data_qos();  // validate QoS parameters early
      return on_system_configure();
    });
  if (ret != CallbackReturn::SUCCESS) {
    // Partial configuration is released so that configure can be retried.
    guarded("cleanup after failed configure", [this] {return on_system_cleanup();});
    std::lock_guard<std::mutex> lock(mutex_);
    topics_.clear();
    return ret;
  }
  publish_info(State::PRIMARY_STATE_INACTIVE, "inactive");
  return ret;
}

MocapSystemNode::CallbackReturn
MocapSystemNode::on_activate(const rclcpp_lifecycle::State &)
{
  stats_.reset(steady_now());
  set_publishers_active(true);
  active_ = true;
  const auto ret = guarded("activate", [this] {return on_system_activate();});
  if (ret != CallbackReturn::SUCCESS) {
    active_ = false;
    set_publishers_active(false);
    return ret;
  }
  publish_info(State::PRIMARY_STATE_ACTIVE, "active");
  return ret;
}

MocapSystemNode::CallbackReturn
MocapSystemNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  const auto ret = guarded("deactivate", [this] {return on_system_deactivate();});
  if (ret != CallbackReturn::SUCCESS) {
    active_ = true;
    return ret;
  }
  set_publishers_active(false);
  publish_info(State::PRIMARY_STATE_INACTIVE, "inactive");
  return ret;
}

MocapSystemNode::CallbackReturn
MocapSystemNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  const auto ret = guarded("cleanup", [this] {return on_system_cleanup();});
  if (ret == CallbackReturn::SUCCESS) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      topics_.clear();
    }
    publish_info(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  }
  return ret;
}

void
MocapSystemNode::release_system(uint8_t from_state)
{
  if (active_ || from_state == State::PRIMARY_STATE_ACTIVE) {
    active_ = false;
    guarded("deactivate", [this] {return on_system_deactivate();});
    set_publishers_active(false);
  }
  if (from_state != State::PRIMARY_STATE_UNCONFIGURED) {
    guarded("cleanup", [this] {return on_system_cleanup();});
  }
  std::lock_guard<std::mutex> lock(mutex_);
  topics_.clear();
}

MocapSystemNode::CallbackReturn
MocapSystemNode::on_shutdown(const rclcpp_lifecycle::State & previous)
{
  release_system(previous.id());
  try {
    on_system_shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "shutdown failed: %s", e.what());
  }
  publish_info(State::PRIMARY_STATE_FINALIZED, "finalized");
  return CallbackReturn::SUCCESS;
}

MocapSystemNode::CallbackReturn
MocapSystemNode::on_error(const rclcpp_lifecycle::State & previous)
{
  RCLCPP_ERROR(get_logger(), "error in state %s, releasing system", previous.label().c_str());
  // The failing transition may come from any state: release everything.
  release_system(State::PRIMARY_STATE_INACTIVE);
  publish_info(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  return CallbackReturn::SUCCESS;
}

bool
MocapSystemNode::addressed(const std::vector<std::string> & systems) const
{
  if (systems.empty()) {
    return true;
  }
  const std::string name = get_name();
  return std::any_of(
    systems.begin(), systems.end(),
    [&](const std::string & s) {return s == system_id_ || s == name;});
}

void
MocapSystemNode::control_callback(const Control::ConstSharedPtr msg)
{
  if (!addressed(msg->systems)) {
    return;
  }

  ControlAck ack;
  ack.request_id = msg->request_id;
  ack.session_id = msg->session_id;
  ack.system = system_id();
  ack.command = msg->command;

  if (msg->command == Control::START) {
    // Set the session first so that the ACTIVE announcement carries it.
    std::string previous_session;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      previous_session = session_id_;
      session_id_ = msg->session_id;
    }
    const bool was_active = get_current_state().id() == State::PRIMARY_STATE_ACTIVE;
    if (get_current_state().id() == State::PRIMARY_STATE_UNCONFIGURED) {
      trigger_transition(Transition::TRANSITION_CONFIGURE);
    }
    if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
      trigger_transition(Transition::TRANSITION_ACTIVATE);
    }
    ack.success = get_current_state().id() == State::PRIMARY_STATE_ACTIVE;
    if (!ack.success) {
      std::lock_guard<std::mutex> lock(mutex_);
      session_id_ = previous_session;
    }
    if (was_active && previous_session != msg->session_id) {
      announce();  // no transition happened: announce the new session
    }
  } else if (msg->command == Control::STOP) {
    if (get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
      trigger_transition(Transition::TRANSITION_DEACTIVATE);
    }
    const auto id = get_current_state().id();
    ack.success = id == State::PRIMARY_STATE_INACTIVE || id == State::PRIMARY_STATE_UNCONFIGURED;
  } else {
    ack.success = false;
    ack.message = "unknown command " + std::to_string(msg->command);
  }

  if (!ack.success && ack.message.empty()) {
    ack.message = "system is in state " + get_current_state().label();
  }
  ack.stamp = now();
  ack_pub_->publish(ack);

  if (ack.success) {
    try {
      if (msg->command == Control::START) {
        on_control_start(*msg);
      } else {
        on_control_stop(*msg);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "control hook failed: %s", e.what());
    }
  }
}

void
MocapSystemNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  status.add("system", system_id_);
  status.add("server", server());
  status.add("connected", connected() ? "true" : "false");
  if (!active_) {
    stats_.take(steady_now());
    status.summary(DiagnosticStatus::OK, "inactive");
    return;
  }

  const auto s = stats_.take(steady_now());
  const double nominal = nominal_frame_rate();
  status.add("rate_hz", s.rate);
  status.add("nominal_rate_hz", nominal);
  status.add("frames", s.frames);
  status.add("dropped", s.dropped);
  status.add("latency_mean_ms", s.latency_mean * 1e3);
  status.add("latency_max_ms", s.latency_max * 1e3);
  status.add("total_frames", s.total_frames);
  status.add("total_dropped", s.total_dropped);
  for (const auto & [body, ratio] : s.tracking_ratio) {
    status.add("tracking/" + body, ratio);
  }

  const uint64_t expected = s.frames + s.dropped;
  if (s.frames == 0) {
    status.summary(DiagnosticStatus::ERROR, "no frames received");
  } else if (expected > 0 && s.dropped * 100 > expected) {
    status.summary(DiagnosticStatus::WARN, "dropping frames");
  } else if (nominal > 0.0 && s.rate < 0.9 * nominal) {
    status.summary(DiagnosticStatus::WARN, "rate below nominal");
  } else {
    status.summary(DiagnosticStatus::OK, "streaming");
  }
}

}  // namespace mocap4r2_control
