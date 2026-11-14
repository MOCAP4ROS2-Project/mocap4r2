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

#ifndef MOCAP4R2_ROBOT_GT__GT_MATH_HPP_
#define MOCAP4R2_ROBOT_GT__GT_MATH_HPP_

#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace mocap4r2_robot_gt
{

// Transform from [x, y, z, roll, pitch, yaw] (fixed axes, as in REP-103).
// Throws std::invalid_argument if the vector does not have 6 values.
tf2::Transform from_xyzrpy(const std::vector<double> & v);

tf2::Transform from_pose(const geometry_msgs::msg::Pose & p);
geometry_msgs::msg::Pose to_pose(const tf2::Transform & t);

// Offset O such that O * current == wish (wish = identity puts the origin at current).
tf2::Transform origin_offset(const tf2::Transform & current, const tf2::Transform & wish);

}  // namespace mocap4r2_robot_gt

#endif  // MOCAP4R2_ROBOT_GT__GT_MATH_HPP_
