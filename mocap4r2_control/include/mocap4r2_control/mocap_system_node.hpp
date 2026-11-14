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

#ifndef MOCAP4R2_CONTROL__MOCAP_SYSTEM_NODE_HPP_
#define MOCAP4R2_CONTROL__MOCAP_SYSTEM_NODE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "mocap4r2_control/frame_stats.hpp"
#include "mocap4r2_interfaces/msg/control.hpp"
#include "mocap4r2_interfaces/msg/control_ack.hpp"
#include "mocap4r2_interfaces/msg/mocap_system_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mocap4r2_control
{

class FramePublisher;

// Well-known topics of the MOCAP4ROS2 control plane.
constexpr char kControlTopic[] = "/mocap4r2/control";
constexpr char kControlAckTopic[] = "/mocap4r2/control_ack";
constexpr char kSystemsTopic[] = "/mocap4r2/systems";

// Base class of every mocap system (driver or auxiliary recorder).
//
// It owns the lifecycle callbacks: drivers implement the on_system_* hooks.
// START/STOP requests on /mocap4r2/control drive activate/deactivate and are
// answered on /mocap4r2/control_ack. The system is announced on /mocap4r2/systems
// at every transition, and frame statistics are published on /diagnostics.
//
// Threading: is_active(), stats() and publish() on data publishers may be used
// from SDK threads. Drivers must stop their SDK threads in on_system_deactivate
// or on_system_cleanup (and in their destructor) before publishers are released.
class MocapSystemNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Control = mocap4r2_interfaces::msg::Control;
  using ControlAck = mocap4r2_interfaces::msg::ControlAck;
  using MocapSystemInfo = mocap4r2_interfaces::msg::MocapSystemInfo;

  MocapSystemNode(
    const std::string & node_name, const std::string & system_type,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // True between a successful activation and the next deactivation.
  bool is_active() const {return active_.load();}

  // Frame of the published data (parameter frame_id).
  const std::string & frame_id() const {return frame_id_;}

  // System id: fully qualified node name.
  const std::string & system_id() const {return system_id_;}

  // Session of the last START request.
  std::string session_id() const;

  FrameStats & stats() {return stats_;}

  // QoS for data topics (parameters qos.reliability and qos.depth).
  rclcpp::QoS data_qos() const;

protected:
  // Hooks for subclasses. Returning FAILURE (or throwing) aborts the transition.
  virtual CallbackReturn on_system_configure() {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_system_activate() {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_system_deactivate() {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_system_cleanup() {return CallbackReturn::SUCCESS;}
  virtual void on_system_shutdown() {}

  // Called after a START/STOP request succeeded.
  virtual void on_control_start(const Control & request) {(void)request;}
  virtual void on_control_stop(const Control & request) {(void)request;}

  // Nominal frame rate announced in MocapSystemInfo (0 if unknown).
  virtual double nominal_frame_rate() const {return 0.0;}

  // Mocap software the system talks to (e.g. "Motive 3.1 at 192.168.1.10"), shown by
  // the tools; empty if not applicable.
  virtual std::string server() const {return "";}

  // Whether the link with that software is up (reported in /diagnostics).
  virtual bool connected() const {return false;}

  // Create a data publisher with data_qos(); it is announced and (de)activated
  // with the node. Call it from on_system_configure.
  template<typename MessageT>
  typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr
  create_data_publisher(const std::string & topic)
  {
    auto pub = rclcpp_lifecycle::LifecycleNode::create_publisher<MessageT>(topic, data_qos());
    std::weak_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT>> weak = pub;
    register_topic(
      pub->get_topic_name(), rosidl_generator_traits::name<MessageT>(),
      [weak](bool activate) {
        if (auto p = weak.lock()) {
          activate ? p->on_activate() : p->on_deactivate();
        }
      });
    return pub;
  }

  // Announce a topic not created through create_data_publisher (e.g. auxiliary systems).
  void register_topic(
    const std::string & topic, const std::string & type,
    std::function<void(bool)> toggle = nullptr);

  // Publish the current MocapSystemInfo.
  void announce();

private:
  friend class FramePublisher;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) final;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) final;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) final;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) final;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) final;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) final;

  void publish_info(uint8_t state_id, const char * state_label);
  CallbackReturn guarded(const char * what, const std::function<CallbackReturn()> & hook);
  void set_publishers_active(bool active);
  void release_system(uint8_t from_state);
  void control_callback(const Control::ConstSharedPtr msg);
  bool addressed(const std::vector<std::string> & systems) const;
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

  struct DataTopic
  {
    std::string name;
    std::string type;
    std::function<void(bool)> toggle;
  };

  std::string system_type_;
  std::string system_id_;
  std::string frame_id_;
  std::atomic<bool> active_{false};
  FrameStats stats_;

  mutable std::mutex mutex_;
  std::vector<DataTopic> topics_;
  std::string session_id_;

  rclcpp::Subscription<Control>::SharedPtr control_sub_;
  rclcpp::Publisher<ControlAck>::SharedPtr ack_pub_;
  rclcpp::Publisher<MocapSystemInfo>::SharedPtr info_pub_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
};

}  // namespace mocap4r2_control

#endif  // MOCAP4R2_CONTROL__MOCAP_SYSTEM_NODE_HPP_
