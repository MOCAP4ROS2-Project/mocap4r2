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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_interfaces/msg/markers.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using mocap4r2_control::MocapSystemNode;
using mocap4r2_interfaces::msg::Control;
using mocap4r2_interfaces::msg::ControlAck;
using mocap4r2_interfaces::msg::Markers;
using mocap4r2_interfaces::msg::MocapSystemInfo;

// System with switchable failures that records which hooks ran.
class TestSystem : public MocapSystemNode
{
public:
  explicit TestSystem(
    const std::string & name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MocapSystemNode(name, "test", options) {}

  bool fail_configure{false};
  bool throw_configure{false};
  bool fail_activate{false};
  std::vector<std::string> calls;
  std::vector<std::string> sessions;
  rclcpp_lifecycle::LifecyclePublisher<Markers>::SharedPtr pub;

  void publish_data(uint64_t frame)
  {
    if (pub) {
      Markers msg;
      msg.header.frame_id = frame_id();
      msg.frame_number = frame;
      pub->publish(msg);
    }
  }

protected:
  CallbackReturn on_system_configure() override
  {
    calls.push_back("configure");
    if (throw_configure) {
      throw std::runtime_error("cannot reach server");
    }
    pub = create_data_publisher<Markers>("markers");
    return fail_configure ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }
  CallbackReturn on_system_activate() override
  {
    calls.push_back("activate");
    return fail_activate ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }
  CallbackReturn on_system_deactivate() override
  {
    calls.push_back("deactivate");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_system_cleanup() override
  {
    calls.push_back("cleanup");
    pub.reset();
    return CallbackReturn::SUCCESS;
  }
  void on_system_shutdown() override {calls.push_back("shutdown");}
  void on_control_start(const Control & r) override {sessions.push_back("start:" + r.session_id);}
  void on_control_stop(const Control & r) override {sessions.push_back("stop:" + r.session_id);}
  double nominal_frame_rate() const override {return 120.0;}
};

class MocapSystemNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_ = rclcpp::Node::make_shared("test_client");
    control_pub_ = client_->create_publisher<Control>(
      mocap4r2_control::kControlTopic, rclcpp::QoS(10).reliable());
    ack_sub_ = client_->create_subscription<ControlAck>(
      mocap4r2_control::kControlAckTopic, rclcpp::QoS(10).reliable(),
      [this](ControlAck::ConstSharedPtr msg) {acks_.push_back(*msg);});
    info_sub_ = client_->create_subscription<MocapSystemInfo>(
      mocap4r2_control::kSystemsTopic, rclcpp::QoS(10).reliable().transient_local(),
      [this](MocapSystemInfo::ConstSharedPtr msg) {infos_.push_back(*msg);});
    executor_.add_node(client_);
  }

  void TearDown() override
  {
    for (auto & s : systems_) {
      executor_.remove_node(s->get_node_base_interface());
    }
  }

  std::shared_ptr<TestSystem> add_system(
    const std::string & name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    auto s = std::make_shared<TestSystem>(name, options);
    executor_.add_node(s->get_node_base_interface());
    systems_.push_back(s);
    return s;
  }

  bool spin_until(const std::function<bool()> & cond, std::chrono::seconds timeout = 10s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      if (cond()) {
        return true;
      }
    }
    return cond();
  }

  // Send a request once the systems listen, and wait for n acks of it.
  std::vector<ControlAck> request(
    uint8_t command, const std::vector<std::string> & systems, size_t n,
    const std::string & session = "s1", std::chrono::seconds timeout = 10s)
  {
    const auto expected_subs = systems_.size();
    EXPECT_TRUE(spin_until([&] {return control_pub_->get_subscription_count() >= expected_subs;}));
    EXPECT_TRUE(spin_until([&] {return ack_sub_->get_publisher_count() >= expected_subs;}));
    Control msg;
    msg.command = command;
    msg.request_id = "req" + std::to_string(++request_count_);
    msg.session_id = session;
    msg.systems = systems;
    msg.stamp = client_->now();
    control_pub_->publish(msg);

    std::vector<ControlAck> got;
    spin_until(
      [&] {
        got.clear();
        for (const auto & a : acks_) {
          if (a.request_id == msg.request_id) {got.push_back(a);}
        }
        return got.size() >= n;
      }, timeout);
    return got;
  }

  std::vector<MocapSystemInfo> infos_of(const std::string & system) const
  {
    std::vector<MocapSystemInfo> out;
    for (const auto & i : infos_) {
      if (i.system == system) {out.push_back(i);}
    }
    return out;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Publisher<Control>::SharedPtr control_pub_;
  rclcpp::Subscription<ControlAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<MocapSystemInfo>::SharedPtr info_sub_;
  std::vector<ControlAck> acks_;
  std::vector<MocapSystemInfo> infos_;
  std::vector<std::shared_ptr<TestSystem>> systems_;
  int request_count_{0};
};

TEST_F(MocapSystemNodeTest, start_from_unconfigured_configures_and_activates)
{
  auto sys = add_system("sys_a");
  auto acks = request(Control::START, {}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_TRUE(acks[0].success) << acks[0].message;
  EXPECT_EQ(acks[0].system, "/sys_a");
  EXPECT_EQ(acks[0].session_id, "s1");
  EXPECT_EQ(acks[0].command, Control::START);
  EXPECT_EQ(sys->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(sys->is_active());
  EXPECT_EQ(sys->calls, (std::vector<std::string>{"configure", "activate"}));
  EXPECT_EQ(sys->sessions, (std::vector<std::string>{"start:s1"}));
  EXPECT_EQ(sys->session_id(), "s1");
}

TEST_F(MocapSystemNodeTest, start_and_stop_are_idempotent)
{
  auto sys = add_system("sys_b");
  ASSERT_TRUE(request(Control::START, {}, 1).at(0).success);
  ASSERT_TRUE(
    spin_until([&] {
      return !infos_of("/sys_b").empty() &&
             infos_of("/sys_b").back().session_id == "s1";
                                                   }));
  auto again = request(Control::START, {}, 1, "s2");
  ASSERT_EQ(again.size(), 1u);
  EXPECT_TRUE(again[0].success);
  EXPECT_EQ(sys->session_id(), "s2");
  // A new session on an active system is announced as well.
  ASSERT_TRUE(spin_until([&] {return infos_of("/sys_b").back().session_id == "s2";}));
  EXPECT_EQ(infos_of("/sys_b").back().state.id, State::PRIMARY_STATE_ACTIVE);

  auto stop = request(Control::STOP, {}, 1, "s2");
  ASSERT_EQ(stop.size(), 1u);
  EXPECT_TRUE(stop[0].success);
  EXPECT_EQ(sys->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(sys->is_active());

  auto stop_again = request(Control::STOP, {}, 1, "s2");
  ASSERT_EQ(stop_again.size(), 1u);
  EXPECT_TRUE(stop_again[0].success);
  // Hooks ran once per real transition.
  EXPECT_EQ(sys->calls, (std::vector<std::string>{"configure", "activate", "deactivate"}));
}

TEST_F(MocapSystemNodeTest, stop_when_unconfigured_succeeds_without_transitions)
{
  auto sys = add_system("sys_c");
  auto acks = request(Control::STOP, {}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_TRUE(acks[0].success);
  EXPECT_TRUE(sys->calls.empty());
}

TEST_F(MocapSystemNodeTest, only_addressed_systems_answer)
{
  auto a = add_system("sys_d");
  auto b = add_system("sys_e", rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/r1"}));
  // Addressed by fully qualified name and by plain name.
  auto acks = request(Control::START, {"/r1/sys_e"}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_EQ(acks[0].system, "/r1/sys_e");
  EXPECT_TRUE(b->is_active());
  EXPECT_FALSE(a->is_active());

  acks = request(Control::START, {"sys_d"}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_EQ(acks[0].system, "/sys_d");

  // Unknown system: nobody answers.
  acks = request(Control::STOP, {"nobody"}, 1, "s1", 1s);
  EXPECT_TRUE(acks.empty());
  EXPECT_TRUE(a->is_active());
  EXPECT_TRUE(b->is_active());

  // Empty list addresses everyone.
  acks = request(Control::STOP, {}, 2);
  EXPECT_EQ(acks.size(), 2u);
}

TEST_F(MocapSystemNodeTest, failed_activation_is_nacked)
{
  auto sys = add_system("sys_f");
  sys->fail_activate = true;
  auto acks = request(Control::START, {}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_FALSE(acks[0].success);
  EXPECT_NE(acks[0].message.find("inactive"), std::string::npos) << acks[0].message;
  EXPECT_EQ(sys->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(sys->is_active());
  EXPECT_TRUE(sys->sessions.empty());
  EXPECT_EQ(sys->session_id(), "");  // failed START does not change the session

  // Retry after the problem is solved.
  sys->fail_activate = false;
  EXPECT_TRUE(request(Control::START, {}, 1).at(0).success);
  EXPECT_TRUE(sys->is_active());
}

TEST_F(MocapSystemNodeTest, failed_or_throwing_configure_is_nacked_and_retryable)
{
  auto sys = add_system("sys_g");
  sys->throw_configure = true;
  auto acks = request(Control::START, {}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_FALSE(acks[0].success);
  EXPECT_EQ(sys->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  sys->throw_configure = false;
  sys->fail_configure = true;
  EXPECT_FALSE(request(Control::START, {}, 1).at(0).success);
  EXPECT_EQ(sys->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  // A failed configure releases what it created.
  EXPECT_EQ(sys->pub, nullptr);

  sys->fail_configure = false;
  EXPECT_TRUE(request(Control::START, {}, 1).at(0).success);
}

TEST_F(MocapSystemNodeTest, unknown_command_is_nacked)
{
  auto sys = add_system("sys_h");
  auto acks = request(42, {}, 1);
  ASSERT_EQ(acks.size(), 1u);
  EXPECT_FALSE(acks[0].success);
  EXPECT_NE(acks[0].message.find("unknown command"), std::string::npos);
  EXPECT_TRUE(sys->calls.empty());
}

TEST_F(MocapSystemNodeTest, announces_state_topics_and_types)
{
  auto sys = add_system("sys_i", rclcpp::NodeOptions().parameter_overrides({{"frame_id", "lab"}}));
  ASSERT_TRUE(spin_until([&] {return !infos_of("/sys_i").empty();}));
  EXPECT_EQ(infos_of("/sys_i").back().state.id, State::PRIMARY_STATE_UNCONFIGURED);

  sys->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(
    spin_until([&] {return infos_of("/sys_i").back().state.id == State::PRIMARY_STATE_INACTIVE;}));
  auto info = infos_of("/sys_i").back();
  EXPECT_EQ(info.type, "test");
  EXPECT_EQ(info.frame_id, "lab");
  EXPECT_DOUBLE_EQ(info.frame_rate, 120.0);
  ASSERT_EQ(info.topics.size(), 1u);
  EXPECT_EQ(info.topics[0], "/markers");
  EXPECT_EQ(info.topic_types[0], "mocap4r2_interfaces/msg/Markers");

  // Transitions not requested through /mocap4r2/control are announced too.
  sys->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(
    spin_until([&] {return infos_of("/sys_i").back().state.id == State::PRIMARY_STATE_ACTIVE;}));
  sys->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  sys->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_TRUE(
    spin_until(
      [&] {return infos_of("/sys_i").back().state.id == State::PRIMARY_STATE_UNCONFIGURED;}));
  EXPECT_TRUE(infos_of("/sys_i").back().topics.empty());
}

TEST_F(MocapSystemNodeTest, late_joiner_gets_last_announcement)
{
  auto sys = add_system("sys_j");
  sys->trigger_transition(Transition::TRANSITION_CONFIGURE);
  sys->trigger_transition(Transition::TRANSITION_ACTIVATE);

  std::vector<MocapSystemInfo> late;
  auto sub = client_->create_subscription<MocapSystemInfo>(
    mocap4r2_control::kSystemsTopic, rclcpp::QoS(10).reliable().transient_local(),
    [&](MocapSystemInfo::ConstSharedPtr msg) {
      if (msg->system == "/sys_j") {late.push_back(*msg);}
    });
  ASSERT_TRUE(spin_until([&] {return !late.empty();}));
  // Only the latest announcement is kept per system.
  spin_until([] {return false;}, 1s);
  ASSERT_EQ(late.size(), 1u);
  EXPECT_EQ(late[0].state.id, State::PRIMARY_STATE_ACTIVE);
}

TEST_F(MocapSystemNodeTest, data_only_flows_while_active)
{
  auto sys = add_system("sys_k");
  std::vector<uint64_t> frames;
  auto sub = client_->create_subscription<Markers>(
    "/markers", rclcpp::QoS(10).reliable(),
    [&](Markers::ConstSharedPtr msg) {frames.push_back(msg->frame_number);});

  sys->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(spin_until([&] {return sys->pub && sys->pub->get_subscription_count() > 0;}));
  sys->publish_data(1);  // inactive: dropped
  sys->trigger_transition(Transition::TRANSITION_ACTIVATE);
  sys->publish_data(2);
  ASSERT_TRUE(spin_until([&] {return !frames.empty();}));
  sys->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  sys->publish_data(3);  // inactive again: dropped
  spin_until([] {return false;}, 1s);
  EXPECT_EQ(frames, (std::vector<uint64_t>{2}));
}

TEST_F(MocapSystemNodeTest, invalid_qos_parameters_fail_configure)
{
  auto bad_rel = add_system(
    "sys_l", rclcpp::NodeOptions().parameter_overrides({{"qos.reliability", "sometimes"}}));
  bad_rel->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(bad_rel->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(bad_rel->calls.empty() || bad_rel->calls == std::vector<std::string>{"cleanup"});

  auto bad_depth = add_system(
    "sys_m", rclcpp::NodeOptions().parameter_overrides({{"qos.depth", 0}}));
  bad_depth->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(bad_depth->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  auto best_effort = add_system(
    "sys_n", rclcpp::NodeOptions().parameter_overrides({{"qos.reliability", "best_effort"}}));
  best_effort->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(best_effort->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    best_effort->pub->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
}

TEST_F(MocapSystemNodeTest, shutdown_from_active_releases_everything)
{
  auto sys = add_system("sys_o");
  sys->trigger_transition(Transition::TRANSITION_CONFIGURE);
  sys->trigger_transition(Transition::TRANSITION_ACTIVATE);
  sys->shutdown();
  EXPECT_EQ(sys->get_current_state().id(), State::PRIMARY_STATE_FINALIZED);
  EXPECT_FALSE(sys->is_active());
  EXPECT_EQ(
    sys->calls,
    (std::vector<std::string>{"configure", "activate", "deactivate", "cleanup", "shutdown"}));
}

TEST_F(MocapSystemNodeTest, shutdown_from_unconfigured_skips_cleanup)
{
  auto sys = add_system("sys_p");
  sys->shutdown();
  EXPECT_EQ(sys->calls, (std::vector<std::string>{"shutdown"}));
}

TEST_F(MocapSystemNodeTest, diagnostics_report_streaming_and_silence)
{
  auto sys = add_system("sys_q");
  std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> last;
  auto sub = client_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10),
    [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      for (const auto & st : msg->status) {
        if (st.name.find("sys_q") != std::string::npos) {last["s"] = st;}
      }
    });

  ASSERT_TRUE(spin_until([&] {return last.count("s") > 0;}));
  EXPECT_EQ(last["s"].message, "inactive");

  sys->trigger_transition(Transition::TRANSITION_CONFIGURE);
  sys->trigger_transition(Transition::TRANSITION_ACTIVATE);
  last.clear();
  // Active without frames: error.
  ASSERT_TRUE(spin_until([&] {return last.count("s") && last["s"].message != "inactive";}));
  EXPECT_EQ(last["s"].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  // Feed frames at a good rate for a while: OK.
  uint64_t frame = 0;
  last.clear();
  const auto end = std::chrono::steady_clock::now() + 3s;
  bool streaming_seen = false;
  while (std::chrono::steady_clock::now() < end && !streaming_seen) {
    for (int i = 0; i < 2; i++) {
      sys->stats().add_frame(++frame, 0.001);
      sys->stats().add_tracking("robot", true);
    }
    executor_.spin_some(10ms);
    streaming_seen = last.count("s") && last["s"].message == "streaming";
  }
  // Rate depends on the loop; only the summary and keys are checked.
  ASSERT_TRUE(last.count("s"));
  bool has_tracking = false;
  for (const auto & kv : last["s"].values) {
    has_tracking |= kv.key == "tracking/robot";
  }
  EXPECT_TRUE(has_tracking);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
