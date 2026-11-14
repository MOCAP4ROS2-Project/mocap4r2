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

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mocap4r2_interfaces/msg/markers.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_sim_driver/sim_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using mocap4r2_interfaces::msg::Markers;
using mocap4r2_interfaces::msg::RigidBodies;
using mocap4r2_sim_driver::SimDriver;

class SimDriverTest : public ::testing::Test
{
protected:
  std::shared_ptr<SimDriver> make(const std::vector<rclcpp::Parameter> & params)
  {
    driver_ = std::make_shared<SimDriver>(rclcpp::NodeOptions().parameter_overrides(params));
    client_ = rclcpp::Node::make_shared("sim_test_client");
    bodies_sub_ = client_->create_subscription<RigidBodies>(
      "/rigid_bodies", rclcpp::QoS(100).reliable(),
      [this](RigidBodies::ConstSharedPtr m) {bodies_.push_back(*m);});
    markers_sub_ = client_->create_subscription<Markers>(
      "/markers", rclcpp::QoS(100).reliable(),
      [this](Markers::ConstSharedPtr m) {markers_.push_back(*m);});
    executor_.add_node(driver_->get_node_base_interface());
    executor_.add_node(client_);
    return driver_;
  }

  void TearDown() override
  {
    if (driver_) {
      driver_->shutdown();
    }
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

  void spin_for(std::chrono::milliseconds d)
  {
    spin_until([] {return false;}, std::chrono::duration_cast<std::chrono::seconds>(d + 999ms));
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<SimDriver> driver_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Subscription<RigidBodies>::SharedPtr bodies_sub_;
  rclcpp::Subscription<Markers>::SharedPtr markers_sub_;
  std::vector<RigidBodies> bodies_;
  std::vector<Markers> markers_;
};

TEST_F(SimDriverTest, streams_consistent_frames_while_active)
{
  auto d = make({{"rate", 200.0}, {"latency", 0.01}, {"frame_id", "lab"}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(300ms);
  EXPECT_TRUE(bodies_.empty());  // inactive: nothing

  ASSERT_TRUE(spin_until([&] {return bodies_sub_->get_publisher_count() > 0;}));
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return bodies_.size() >= 50 && markers_.size() >= 50;}));

  for (size_t i = 1; i < bodies_.size(); i++) {
    EXPECT_GT(bodies_[i].frame_number, bodies_[i - 1].frame_number);
  }
  const auto & b = bodies_.back();
  EXPECT_EQ(b.header.frame_id, "lab");
  ASSERT_EQ(b.rigid_bodies.size(), 2u);
  EXPECT_EQ(b.rigid_bodies[0].name, "robot");
  EXPECT_EQ(b.rigid_bodies[1].name, "wand");
  // Same frame number and stamp for the markers of the same frame.
  bool matched = false;
  for (const auto & m : markers_) {
    if (m.frame_number == b.frame_number) {
      EXPECT_EQ(m.header.stamp, b.header.stamp);
      matched = true;
    }
  }
  EXPECT_TRUE(matched);
  // Stamp is the capture time: before now by about the latency.
  const double age = (d->now() - rclcpp::Time(b.header.stamp)).seconds();
  EXPECT_GE(age, 0.009);
  EXPECT_LT(age, 1.0);

  d->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  spin_for(100ms);
  const auto n = bodies_.size();
  spin_for(300ms);
  EXPECT_EQ(bodies_.size(), n);  // stopped
}

TEST_F(SimDriverTest, dropped_frames_show_as_gaps_and_in_stats)
{
  auto d = make({{"rate", 500.0}, {"frame_drop_probability", 0.3}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(spin_until([&] {return bodies_sub_->get_publisher_count() > 0;}));
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return bodies_.size() >= 200;}));
  uint64_t gaps = 0;
  for (size_t i = 1; i < bodies_.size(); i++) {
    gaps += bodies_[i].frame_number - bodies_[i - 1].frame_number - 1;
  }
  EXPECT_GT(gaps, 20u);
  const auto s = d->stats().take(0.0);
  EXPECT_GT(s.total_dropped, 20u);
}

TEST_F(SimDriverTest, occluded_bodies_are_flagged)
{
  auto d = make({{"rate", 500.0}, {"occlusion_probability", 0.05}, {"occlusion_duration", 0.05}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(spin_until([&] {return bodies_sub_->get_publisher_count() > 0;}));
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  bool seen_invalid = false;
  ASSERT_TRUE(
    spin_until(
      [&] {
        for (const auto & b : bodies_) {
          for (const auto & rb : b.rigid_bodies) {
            seen_invalid |= !rb.tracking_valid;
          }
        }
        return seen_invalid;
      }));
}

TEST_F(SimDriverTest, reconfigure_cycle)
{
  auto d = make({{"rate", 200.0}});
  for (int i = 0; i < 3; i++) {
    d->trigger_transition(Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << "cycle " << i;
    d->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
    bodies_.clear();
    ASSERT_TRUE(spin_until([&] {return bodies_.size() >= 5;})) << "cycle " << i;
    d->trigger_transition(Transition::TRANSITION_DEACTIVATE);
    d->trigger_transition(Transition::TRANSITION_CLEANUP);
    ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(SimDriverTest, destroy_while_active_is_clean)
{
  auto d = make({{"rate", 1000.0}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  spin_for(100ms);
  executor_.remove_node(d->get_node_base_interface());
  driver_.reset();
  d.reset();  // destructor joins the acquisition thread
  SUCCEED();
}

TEST_F(SimDriverTest, invalid_parameters_fail_configure)
{
  const std::vector<std::vector<rclcpp::Parameter>> bad = {
    {{"rate", 0.0}},
    {{"rate", -5.0}},
    {{"latency", -0.1}},
    {{"frame_drop_probability", 1.5}},
    {{"markers_per_body", 99}},
    {{"noise", -1.0}},
    {{"bodies", std::vector<std::string>{"x"}}, {"bodies_config.x.motion", "spiral"}},
    {{"bodies", std::vector<std::string>{"x"}},
      {"bodies_config.x.center", std::vector<double>{1.0}}},
  };
  for (size_t i = 0; i < bad.size(); i++) {
    auto d = std::make_shared<SimDriver>(rclcpp::NodeOptions().parameter_overrides(bad[i]));
    d->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED) << "case " << i;
  }
}

TEST_F(SimDriverTest, publishes_tf_of_valid_bodies)
{
  auto d = make({{"rate", 200.0}, {"publish_tf", true}, {"tf_prefix", "mc/"}});
  std::vector<std::string> children;
  auto sub = client_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100),
    [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
      for (const auto & t : m->transforms) {
        children.push_back(t.child_frame_id);
        EXPECT_EQ(t.header.frame_id, "mocap");
      }
    });
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return children.size() >= 10;}));
  EXPECT_NE(std::find(children.begin(), children.end(), "mc/robot"), children.end());
  EXPECT_NE(std::find(children.begin(), children.end(), "mc/wand"), children.end());
}

TEST_F(SimDriverTest, topics_can_be_disabled)
{
  auto d = make({{"rate", 200.0}, {"publish_markers", false}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(spin_until([&] {return bodies_sub_->get_publisher_count() > 0;}));
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return bodies_.size() >= 10;}));
  EXPECT_EQ(markers_sub_->get_publisher_count(), 0u);
  EXPECT_TRUE(markers_.empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
