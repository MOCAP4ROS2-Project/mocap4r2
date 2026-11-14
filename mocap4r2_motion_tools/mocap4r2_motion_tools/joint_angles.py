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

"""Joint angles of skeletons: rotation of each segment relative to its parent."""

import math
from typing import Dict, Sequence, Tuple

import numpy as np


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cardan_xyz(r: np.ndarray) -> Tuple[float, float, float]:
    """Angles (a, b, c) with r = Rx(a) Ry(b) Rz(c) (mobile axes X, Y', Z'')."""
    b = math.asin(max(-1.0, min(1.0, r[0, 2])))
    if abs(r[0, 2]) > 1.0 - 1e-9:  # gimbal lock: put everything in a
        return math.atan2(r[2, 1], r[1, 1]), b, 0.0
    return math.atan2(-r[1, 2], r[2, 2]), b, math.atan2(-r[0, 1], r[0, 0])


def joint_angles(segments) -> Dict[str, Tuple[float, float, float]]:
    """
    Cardan XYZ angles of every segment with a parent (mocap4r2 Segment messages).

    Segments that are not tracked, or whose parent is missing or untracked, are
    left out.
    """
    by_id = {s.id: s for s in segments}
    out = {}
    for s in segments:
        parent = by_id.get(s.parent_id)
        if parent is None or not s.tracking_valid or not parent.tracking_valid:
            continue
        rp = quat_to_matrix([parent.pose.orientation.x, parent.pose.orientation.y,
                             parent.pose.orientation.z, parent.pose.orientation.w])
        rc = quat_to_matrix([s.pose.orientation.x, s.pose.orientation.y,
                             s.pose.orientation.z, s.pose.orientation.w])
        out[s.name] = cardan_xyz(rp.T @ rc)
    return out
