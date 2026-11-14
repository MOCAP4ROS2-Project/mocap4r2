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

"""
Trajectory math for ground-truth evaluation, independent of ROS.

Poses are 4x4 homogeneous matrices; trajectories are (stamps, poses) with
stamps in seconds, sorted.
"""

import math
from typing import List, Sequence, Tuple

import numpy as np


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """Rotation matrix of a quaternion (x, y, z, w), normalized first."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w) of a rotation matrix."""
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [s / 4, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 1] + r[1, 0]) / s, s / 4, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4, (r[1, 0] - r[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def pose(position: Sequence[float], quat: Sequence[float]) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quat)
    t[:3, 3] = position
    return t


def yaw_of(t: np.ndarray) -> float:
    return float(np.arctan2(t[1, 0], t[0, 0]))


def umeyama(src: np.ndarray, dst: np.ndarray, planar: bool = False) -> np.ndarray:
    """
    Rigid transform T (4x4) minimizing sum |dst_i - T src_i|^2 (no scale).

    src, dst: N x 3 corresponding points. With planar=True the rotation is a yaw
    and the vertical offset is the mean difference (robots on a floor).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.shape[0] < 2 or src.shape[1] != 3:
        raise ValueError('need at least 2 corresponding 3D points')
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    xs, xd = src - mu_s, dst - mu_d
    if planar:
        cov = xd[:, :2].T @ xs[:, :2]
        yaw = np.arctan2(cov[1, 0] - cov[0, 1], cov[0, 0] + cov[1, 1])
        r = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    else:
        if src.shape[0] < 3:
            raise ValueError('need at least 3 points for a 3D alignment')
        u, _, vt = np.linalg.svd(xd.T @ xs)
        d = np.sign(np.linalg.det(u @ vt))
        r = u @ np.diag([1.0, 1.0, d]) @ vt
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = mu_d - r @ mu_s
    return t


def associate(stamps_a: np.ndarray, stamps_b: np.ndarray, max_dt: float) -> List[Tuple[int, int]]:
    """Pairs (i, j) of nearest stamps closer than max_dt (both sorted)."""
    pairs = []
    if len(stamps_b) == 0:
        return pairs
    j = 0
    for i, t in enumerate(stamps_a):
        while j + 1 < len(stamps_b) and abs(stamps_b[j + 1] - t) <= abs(stamps_b[j] - t):
            j += 1
        if abs(stamps_b[j] - t) <= max_dt:
            pairs.append((i, j))
    return pairs


def ate(est: Sequence[np.ndarray], gt: Sequence[np.ndarray], align: bool = True,
        planar: bool = False) -> dict:
    """Absolute trajectory error of associated poses (translation, metres)."""
    p_est = np.array([e[:3, 3] for e in est])
    p_gt = np.array([g[:3, 3] for g in gt])
    t = umeyama(p_est, p_gt, planar) if align and len(p_est) >= 3 else np.eye(4)
    aligned = (t[:3, :3] @ p_est.T).T + t[:3, 3]
    err = np.linalg.norm(aligned - p_gt, axis=1)
    return {'rmse': float(np.sqrt(np.mean(err ** 2))), 'mean': float(err.mean()),
            'max': float(err.max()), 'n': int(len(err)), 'alignment': t}


def rpe(est: Sequence[np.ndarray], gt: Sequence[np.ndarray], delta: int = 1) -> dict:
    """Relative pose error over steps of delta poses (translation m, rotation rad)."""
    trans, rot = [], []
    for i in range(len(est) - delta):
        de = np.linalg.inv(est[i]) @ est[i + delta]
        dg = np.linalg.inv(gt[i]) @ gt[i + delta]
        e = np.linalg.inv(dg) @ de
        trans.append(np.linalg.norm(e[:3, 3]))
        rot.append(np.arccos(np.clip((np.trace(e[:3, :3]) - 1) / 2, -1.0, 1.0)))
    if not trans:
        return {'trans_rmse': 0.0, 'rot_rmse': 0.0, 'n': 0}
    trans, rot = np.array(trans), np.array(rot)
    return {'trans_rmse': float(np.sqrt(np.mean(trans ** 2))),
            'rot_rmse': float(np.sqrt(np.mean(rot ** 2))), 'n': int(len(trans))}


def time_offset(t_a, w_a, t_b, w_b, max_offset: float = 0.5, step: float = 0.001) -> float:
    """
    Offset d such that signal b(t) ~ a(t - d), by cross-correlation.

    Typical use: yaw rate of the ground truth (a) against the robot IMU or odometry
    (b) to find how late the robot stamps are with respect to the mocap ones.
    """
    t_a, w_a, t_b, w_b = (np.asarray(v, dtype=float) for v in (t_a, w_a, t_b, w_b))
    start = max(t_a[0], t_b[0]) + max_offset
    end = min(t_a[-1], t_b[-1]) - max_offset
    if end - start < 4 * max_offset:
        raise ValueError('signals overlap too little for this max_offset')
    grid = np.arange(start, end, step)
    b = np.interp(grid, t_b, w_b)
    b = b - b.mean()
    best, best_d = -np.inf, 0.0
    for d in np.arange(-max_offset, max_offset + step / 2, step):
        a = np.interp(grid - d, t_a, w_a)
        a = a - a.mean()
        c = float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)
        if c > best:
            best, best_d = c, d
    return float(best_d)


def circle_center(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """Least-squares circle (centre xy, radius) through N x 2 points."""
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        raise ValueError('need at least 3 points')
    a = np.c_[2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))]
    b = (p ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    c = sol[:2]
    return c, float(np.sqrt(sol[2] + c @ c))


def write_tum(path: str, stamps: Sequence[float], poses: Sequence[np.ndarray]):
    """TUM format: 'stamp x y z qx qy qz qw' per line (read by evo)."""
    with open(path, 'w') as f:
        for s, t in zip(stamps, poses):
            q = matrix_to_quat(t[:3, :3])
            f.write(f'{s:.9f} {t[0, 3]:.6f} {t[1, 3]:.6f} {t[2, 3]:.6f} '
                    f'{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n')


def read_tum(path: str):
    stamps, poses = [], []
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith('#'):
                continue
            v = [float(x) for x in line.split()]
            stamps.append(v[0])
            poses.append(pose(v[1:4], v[4:8]))
    return np.array(stamps), poses


def body_offset_from_rotation(body_poses: Sequence[np.ndarray], base_height: float = 0.0):
    """
    Position of the robot base in the marker body frame, from a turn in place.

    While the robot spins about its vertical axis, the body traces a circle whose
    centre is the base. base_height is the height of the base origin in the mocap
    frame (0 for base_footprint on the floor). Returns (offset xyz, circle radius).
    """
    if len(body_poses) < 10:
        raise ValueError('need at least 10 poses of the turn')
    yaws = np.unwrap([yaw_of(p) for p in body_poses])
    if abs(yaws[-1] - yaws[0]) < math.radians(90):
        raise ValueError('the robot must turn at least 90 degrees in place')
    centre, radius = circle_center(np.array([p[:2, 3] for p in body_poses]))
    base = np.array([centre[0], centre[1], base_height])
    offsets = [p[:3, :3].T @ (base - p[:3, 3]) for p in body_poses]
    return np.mean(offsets, axis=0), radius


def yaw_from_straight(body_poses: Sequence[np.ndarray], min_distance: float = 0.3) -> float:
    """
    Yaw of the robot x axis in the marker body frame, from a straight drive forward.

    The direction of travel is the robot x axis; expressed in the body frame it
    gives the yaw between both frames.
    """
    p0, p1 = body_poses[0][:3, 3], body_poses[-1][:3, 3]
    d = p1 - p0
    if np.linalg.norm(d[:2]) < min_distance:
        raise ValueError(f'drive at least {min_distance} m straight forward')
    yaws = np.unwrap([yaw_of(p) for p in body_poses])
    if abs(yaws[-1] - yaws[0]) > math.radians(10):
        raise ValueError('the drive must be straight (body turned more than 10 degrees)')
    local = body_poses[0][:3, :3].T @ d
    return float(np.arctan2(local[1], local[0]))
