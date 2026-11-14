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

#include <stdexcept>
#include <vector>

#include "mocap4r2_robot_gt/gt_math.hpp"

namespace mocap4r2_robot_gt
{

tf2::Transform
from_xyzrpy(const std::vector<double> & v)
{
  if (v.size() != 6u) {
    throw std::invalid_argument("expected 6 values [x, y, z, roll, pitch, yaw]");
  }
  tf2::Quaternion q;
  q.setRPY(v[3], v[4], v[5]);
  return tf2::Transform(q, tf2::Vector3(v[0], v[1], v[2]));
}

tf2::Transform
from_pose(const geometry_msgs::msg::Pose & p)
{
  tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  if (q.length2() < 1e-12) {
    throw std::invalid_argument("zero quaternion");
  }
  q.normalize();
  return tf2::Transform(q, tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

geometry_msgs::msg::Pose
to_pose(const tf2::Transform & t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  const auto q = t.getRotation();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

tf2::Transform
origin_offset(const tf2::Transform & current, const tf2::Transform & wish)
{
  return wish * current.inverse();
}

}  // namespace mocap4r2_robot_gt
