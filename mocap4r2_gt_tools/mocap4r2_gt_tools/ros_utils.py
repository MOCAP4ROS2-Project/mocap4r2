# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Small conversions between ROS messages and 4x4 matrices."""

from geometry_msgs.msg import Pose, TransformStamped
from mocap4r2_gt_tools import trajectory as tr
import numpy as np


def from_transform(t: TransformStamped) -> np.ndarray:
    tt, q = t.transform.translation, t.transform.rotation
    return tr.pose([tt.x, tt.y, tt.z], [q.x, q.y, q.z, q.w])


def from_pose(p: Pose) -> np.ndarray:
    return tr.pose([p.position.x, p.position.y, p.position.z],
                   [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def to_pose(m: np.ndarray) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in m[:3, 3])
    q = tr.matrix_to_quat(m[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def to_transform(m: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = stamp
    t.child_frame_id = child
    p = to_pose(m)
    t.transform.translation.x = p.position.x
    t.transform.translation.y = p.position.y
    t.transform.translation.z = p.position.z
    t.transform.rotation = p.orientation
    return t


def xyzrpy(m: np.ndarray):
    """[x, y, z, roll, pitch, yaw] of a matrix (fixed axes, as REP-103)."""
    r = m[:3, :3]
    pitch = float(np.arcsin(-np.clip(r[2, 0], -1.0, 1.0)))
    roll = float(np.arctan2(r[2, 1], r[2, 2]))
    yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    return [float(m[0, 3]), float(m[1, 3]), float(m[2, 3]), roll, pitch, yaw]
