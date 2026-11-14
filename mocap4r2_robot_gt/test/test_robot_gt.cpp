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
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_interfaces/srv/set_origin.hpp"
#include "mocap4r2_robot_gt/gt_math.hpp"
#include "mocap4r2_robot_gt/robot_gt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

using namespace std::chrono_literals;
using mocap4r2_interfaces::msg::RigidBodies;
using mocap4r2_interfaces::msg::RigidBody;
using mocap4r2_interfaces::srv::SetOrigin;
using mocap4r2_robot_gt::from_xyzrpy;
using mocap4r2_robot_gt::RobotGT;
using mocap4r2_robot_gt::to_pose;

namespace
{

tf2::Transform to_tf(const geometry_msgs::msg::PoseStamped & p)
{
  return mocap4r2_robot_gt::from_pose(p.pose);
}

}  // namespace

class RobotGTTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_ = rclcpp::Node::make_shared("gt_test_client");
    pub_ = client_->create_publisher<RigidBodies>("/rigid_bodies", rclcpp::QoS(10));
    buffer_ = std::make_shared<tf2_ros::Buffer>(client_->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    pose_sub_ = client_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_gt/pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {poses_.push_back(*m);});
    executor_.add_node(client_);
  }

  void TearDown() override
  {
    if (gt_) {
      executor_.remove_node(gt_);
    }
  }

  void make(const std::vector<rclcpp::Parameter> & params)
  {
    gt_ = std::make_shared<RobotGT>(rclcpp::NodeOptions().parameter_overrides(params));
    executor_.add_node(gt_);
    ASSERT_TRUE(spin_until([&] {return pub_->get_subscription_count() > 0;}));
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

  static RigidBody body(const std::string & name, const tf2::Transform & t, bool valid = true)
  {
    RigidBody rb;
    rb.name = name;
    rb.tracking_valid = valid;
    rb.pose = to_pose(t);
    return rb;
  }

  void send(const std::vector<RigidBody> & bodies)
  {
    RigidBodies msg;
    msg.header.frame_id = "mocap";
    msg.header.stamp = client_->now();
    msg.rigid_bodies = bodies;
    pub_->publish(msg);
  }

  // Send until a new pose is produced (or timeout).
  bool send_and_wait(const std::vector<RigidBody> & bodies, std::chrono::seconds t = 5s)
  {
    const auto n = poses_.size();
    const auto end = std::chrono::steady_clock::now() + t;
    while (std::chrono::steady_clock::now() < end) {
      send(bodies);
      if (spin_until([&] {return poses_.size() > n;}, 1s)) {
        return true;
      }
    }
    return false;
  }

  tf2::Transform lookup(const std::string & parent, const std::string & child)
  {
    geometry_msgs::msg::TransformStamped t;
    EXPECT_TRUE(
      spin_until(
        [&] {
          try {
            t = buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            return true;
          } catch (const tf2::TransformException &) {
            return false;
          }
        }));
    tf2::Transform out;
    tf2::fromMsg(t.transform, out);
    return out;
  }

  std::shared_ptr<SetOrigin::Response> call_set_origin(bool use_current, const tf2::Transform & o)
  {
    auto cli = client_->create_client<SetOrigin>("/robot_gt/set_origin");
    EXPECT_TRUE(cli->wait_for_service(5s));
    auto req = std::make_shared<SetOrigin::Request>();
    req->use_current = use_current;
    req->origin = to_pose(o);
    auto fut = cli->async_send_request(req);
    EXPECT_TRUE(
      spin_until([&] {return fut.wait_for(0s) == std::future_status::ready;}));
    return fut.get();
  }

  static void expect_tf_eq(const tf2::Transform & a, const tf2::Transform & b, double tol = 1e-6)
  {
    EXPECT_NEAR(a.getOrigin().x(), b.getOrigin().x(), tol);
    EXPECT_NEAR(a.getOrigin().y(), b.getOrigin().y(), tol);
    EXPECT_NEAR(a.getOrigin().z(), b.getOrigin().z(), tol);
    EXPECT_NEAR(std::abs(a.getRotation().dot(b.getRotation())), 1.0, tol);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr client_;
  std::shared_ptr<RobotGT> gt_;
  rclcpp::Publisher<RigidBodies>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
};

TEST_F(RobotGTTest, ground_truth_is_body_pose_times_body_to_robot)
{
  make({{"rigid_body", "robot"}, {"body_to_robot", std::vector<double>{0, 0, -0.3, 0, 0, M_PI}}});
  const auto mocap_body = from_xyzrpy({1.0, 2.0, 0.4, 0.0, 0.0, 0.5});
  ASSERT_TRUE(send_and_wait({body("other", from_xyzrpy({9, 9, 9, 0, 0, 0})),
      body("robot", mocap_body)}));
  const auto expected = mocap_body * from_xyzrpy({0, 0, -0.3, 0, 0, M_PI});
  expect_tf_eq(lookup("mocap", "base_link_gt"), expected);
  // Origin is the identity until set.
  expect_tf_eq(lookup("mocap", "gt_origin"), tf2::Transform::getIdentity());
  EXPECT_EQ(poses_.back().header.frame_id, "gt_origin");
  ASSERT_TRUE(gt_->current().has_value());
}

TEST_F(RobotGTTest, missing_empty_or_lost_bodies_produce_nothing)
{
  make({{"rigid_body", "robot"}});
  EXPECT_FALSE(send_and_wait({}, 2s));
  EXPECT_FALSE(send_and_wait({body("wand", tf2::Transform::getIdentity())}, 2s));
  EXPECT_FALSE(send_and_wait({body("robot", tf2::Transform::getIdentity(), false)}, 2s));
  RigidBody bad = body("robot", tf2::Transform::getIdentity());
  bad.pose.orientation.w = 0.0;  // zero quaternion
  EXPECT_FALSE(send_and_wait({bad}, 2s));
  EXPECT_FALSE(gt_->current().has_value());
  EXPECT_TRUE(poses_.empty());
}

TEST_F(RobotGTTest, single_body_is_used_when_no_name_is_given)
{
  make({});
  ASSERT_TRUE(send_and_wait({body("anything", from_xyzrpy({1, 0, 0, 0, 0, 0}))}));
  // Two bodies and no name: ambiguous, nothing published.
  const auto n = poses_.size();
  EXPECT_FALSE(
    send_and_wait(
      {body("a", tf2::Transform::getIdentity()), body("b", tf2::Transform::getIdentity())}, 2s));
  EXPECT_EQ(poses_.size(), n);
}

TEST_F(RobotGTTest, set_origin_before_any_pose_fails)
{
  make({{"rigid_body", "robot"}});
  auto resp = call_set_origin(true, tf2::Transform::getIdentity());
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
}

TEST_F(RobotGTTest, set_origin_to_current_and_to_a_pose)
{
  make({{"rigid_body", "robot"}});
  const auto mocap_robot = from_xyzrpy({3.0, -1.0, 0.0, 0.0, 0.0, 1.0});
  ASSERT_TRUE(send_and_wait({body("robot", mocap_robot)}));

  ASSERT_TRUE(call_set_origin(true, tf2::Transform::getIdentity())->success);
  ASSERT_TRUE(send_and_wait({body("robot", mocap_robot)}));
  expect_tf_eq(to_tf(poses_.back()), tf2::Transform::getIdentity());
  // The measured pose in the mocap frame does not change.
  buffer_->clear();
  ASSERT_TRUE(send_and_wait({body("robot", mocap_robot)}));
  expect_tf_eq(lookup("mocap", "base_link_gt"), mocap_robot);

  const auto wish = from_xyzrpy({0.5, 0.5, 0.0, 0.0, 0.0, -0.3});
  ASSERT_TRUE(call_set_origin(false, wish)->success);
  ASSERT_TRUE(send_and_wait({body("robot", mocap_robot)}));
  expect_tf_eq(to_tf(poses_.back()), wish);

  // Moving forward 1 m in the robot frame moves the ground truth the same way.
  const auto moved = mocap_robot * from_xyzrpy({1, 0, 0, 0, 0, 0});
  ASSERT_TRUE(send_and_wait({body("robot", moved)}));
  expect_tf_eq(to_tf(poses_.back()), wish * from_xyzrpy({1, 0, 0, 0, 0, 0}));
}

TEST_F(RobotGTTest, body_to_robot_from_tf)
{
  tf2_ros::StaticTransformBroadcaster st(*client_);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "base_link";
  t.child_frame_id = "mocap_markers";
  t.header.stamp = client_->now();
  // Marker body mounted 0.5 m above base_link.
  t.transform.translation.z = 0.5;
  t.transform.rotation.w = 1.0;
  st.sendTransform(t);

  make({{"rigid_body", "robot"}, {"body_frame", "mocap_markers"}});
  const auto mocap_body = from_xyzrpy({1.0, 1.0, 0.8, 0.0, 0.0, 0.0});
  ASSERT_TRUE(send_and_wait({body("robot", mocap_body)}));
  expect_tf_eq(lookup("mocap", "base_link_gt"), from_xyzrpy({1.0, 1.0, 0.3, 0.0, 0.0, 0.0}));
}

TEST_F(RobotGTTest, custom_frames_and_initial_origin)
{
  make({{"rigid_body", "robot"}, {"robot_frame", "r1/base_footprint"},
      {"origin_frame", "r1/gt_odom"}, {"origin", std::vector<double>{1, 0, 0, 0, 0, 0}}});
  ASSERT_TRUE(send_and_wait({body("robot", from_xyzrpy({3, 0, 0, 0, 0, 0}))}));
  expect_tf_eq(lookup("mocap", "r1/gt_odom"), from_xyzrpy({1, 0, 0, 0, 0, 0}));
  expect_tf_eq(lookup("r1/gt_odom", "r1/base_footprint_gt"), from_xyzrpy({2, 0, 0, 0, 0, 0}));
}

TEST_F(RobotGTTest, odometry_velocities_in_the_robot_frame)
{
  make({{"rigid_body", "robot"}, {"velocity_alpha", 1.0}});
  std::vector<nav_msgs::msg::Odometry> odoms;
  auto sub = client_->create_subscription<nav_msgs::msg::Odometry>(
    "/robot_gt/odom", rclcpp::QoS(10),
    [&](nav_msgs::msg::Odometry::ConstSharedPtr m) {odoms.push_back(*m);});
  // Robot yawed 90 degrees moving along world +y at 1 m/s and turning at 0.5 rad/s:
  // in its own frame it moves forward (+x).
  RigidBodies msg;
  msg.header.frame_id = "mocap";
  const auto t0 = client_->now();
  ASSERT_TRUE(spin_until([&] {return sub->get_publisher_count() > 0;}));
  for (int i = 0; i < 20; i++) {
    const double t = i * 0.1;
    msg.header.stamp = t0 + rclcpp::Duration::from_seconds(t);
    msg.rigid_bodies = {body("robot", from_xyzrpy({0.0, t, 0.0, 0.0, 0.0, M_PI_2 + 0.5 * t}))};
    pub_->publish(msg);
    spin_until([] {return false;}, 1s);
    if (odoms.size() >= 5) {
      break;
    }
  }
  ASSERT_GE(odoms.size(), 3u);
  const auto & o = odoms.back();
  EXPECT_EQ(o.child_frame_id, "base_link_gt");
  EXPECT_EQ(o.header.frame_id, "gt_origin");
  EXPECT_NEAR(o.twist.twist.angular.z, 0.5, 1e-6);
  EXPECT_NEAR(std::hypot(o.twist.twist.linear.x, o.twist.twist.linear.y), 1.0, 0.03);
  EXPECT_GT(o.twist.twist.linear.x, 0.9);  // mostly forward in the robot frame
}

TEST(RobotGTParams, malformed_vectors_are_rejected)
{
  EXPECT_THROW(
    RobotGT(rclcpp::NodeOptions().parameter_overrides(
      {{"body_to_robot", std::vector<double>{1, 2, 3}}})), std::invalid_argument);
  EXPECT_THROW(
    RobotGT(rclcpp::NodeOptions().parameter_overrides(
      {{"origin", std::vector<double>{1}}})), std::invalid_argument);
  EXPECT_THROW(
    RobotGT(rclcpp::NodeOptions().parameter_overrides({{"velocity_alpha", 0.0}})),
    std::invalid_argument);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
