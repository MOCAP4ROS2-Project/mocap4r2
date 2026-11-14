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

import math

from mocap4r2_gt_tools import trajectory as tr
import numpy as np
import pytest


def rot(axis, angle):
    q = np.r_[np.asarray(axis, float) / np.linalg.norm(axis) * math.sin(angle / 2),
              math.cos(angle / 2)]
    return tr.quat_to_matrix(q)


def make(t, r):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def test_quaternion_round_trip_all_branches():
    rng = np.random.default_rng(0)
    cases = [np.eye(3), rot([1, 0, 0], math.pi), rot([0, 1, 0], math.pi),
             rot([0, 0, 1], math.pi), rot([1, 1, 0], 3.0)]
    cases += [rot(rng.normal(size=3), rng.uniform(-3.1, 3.1)) for _ in range(50)]
    for r in cases:
        q = tr.matrix_to_quat(r)
        assert np.isclose(np.linalg.norm(q), 1.0)
        assert np.allclose(tr.quat_to_matrix(q), r, atol=1e-9)


def test_umeyama_recovers_3d_transform():
    rng = np.random.default_rng(1)
    t = make([1.0, -2.0, 0.5], rot([0.3, 0.2, 1.0], 1.1))
    src = rng.uniform(-3, 3, size=(30, 3))
    dst = (t[:3, :3] @ src.T).T + t[:3, 3]
    assert np.allclose(tr.umeyama(src, dst), t, atol=1e-9)
    noisy = dst + rng.normal(scale=0.002, size=dst.shape)
    assert np.allclose(tr.umeyama(src, noisy), t, atol=0.01)


def test_umeyama_planar_and_reflection_free():
    rng = np.random.default_rng(2)
    t = make([0.5, 0.2, 0.1], rot([0, 0, 1], -2.0))
    src = np.c_[rng.uniform(-2, 2, size=(20, 2)), np.zeros(20)]
    dst = (t[:3, :3] @ src.T).T + t[:3, 3]
    assert np.allclose(tr.umeyama(src, dst, planar=True), t, atol=1e-9)
    # Planar data in 3D mode: still a proper rotation (det +1), no mirror.
    r = tr.umeyama(src, dst)[:3, :3]
    assert np.isclose(np.linalg.det(r), 1.0)


def test_umeyama_bad_inputs():
    with pytest.raises(ValueError):
        tr.umeyama(np.zeros((1, 3)), np.zeros((1, 3)))
    with pytest.raises(ValueError):
        tr.umeyama(np.zeros((2, 3)), np.zeros((2, 3)))  # 3D needs 3 points
    with pytest.raises(ValueError):
        tr.umeyama(np.zeros((4, 3)), np.zeros((5, 3)))


def test_associate():
    a = np.array([0.0, 0.1, 0.2, 0.3, 1.0])
    b = np.array([0.005, 0.098, 0.31, 0.5])
    assert tr.associate(a, b, 0.02) == [(0, 0), (1, 1), (3, 2)]
    assert tr.associate(a, np.array([]), 0.1) == []


def circle_path(n=200):
    stamps = np.linspace(0, 20, n)
    poses = [make([math.cos(0.3 * s), math.sin(0.3 * s), 0.0], rot([0, 0, 1], 0.3 * s + 1.57))
             for s in stamps]
    return stamps, poses


def test_ate_and_rpe_exact_and_offset():
    _, gt = circle_path()
    r = tr.ate(gt, gt)
    assert r['rmse'] < 1e-9 and r['n'] == 200
    # The estimate lives in another frame: alignment removes it.
    off = make([3.0, 1.0, 0.0], rot([0, 0, 1], 0.7))
    est = [np.linalg.inv(off) @ g for g in gt]
    assert tr.ate(est, gt)['rmse'] < 1e-9
    assert tr.ate(est, gt, align=False)['rmse'] > 1.0
    assert tr.rpe(est, gt)['trans_rmse'] < 1e-9  # relative motion is frame free


def test_ate_and_rpe_with_drift():
    _, gt = circle_path()
    est = []
    for i, g in enumerate(gt):
        e = g.copy()
        e[0, 3] += 0.001 * i  # 0.2 m drift at the end
        est.append(e)
    a = tr.ate(est, gt, align=False)
    assert 0.1 < a['rmse'] < 0.2
    assert np.isclose(a['max'], 0.199, atol=1e-6)
    r = tr.rpe(est, gt)
    assert np.isclose(r['trans_rmse'], 0.001, atol=1e-6)
    assert r['rot_rmse'] < 1e-9
    assert tr.rpe(est[:1], gt[:1])['n'] == 0


def test_time_offset_found():
    t = np.arange(0, 20, 0.005)
    w = np.sin(1.3 * t) + 0.5 * np.sin(3.7 * t)
    delay = 0.037
    t_b = np.arange(0, 20, 0.01)
    w_b = np.interp(t_b - delay, t, w)  # b lags a by delay
    assert abs(tr.time_offset(t, w, t_b, w_b) - delay) < 0.002
    with pytest.raises(ValueError):
        tr.time_offset(t[:100], w[:100], t_b[:50], w_b[:50])


def test_circle_center():
    a = np.linspace(0, 2 * math.pi, 50, endpoint=False)
    pts = np.c_[0.3 + 0.12 * np.cos(a), -0.05 + 0.12 * np.sin(a)]
    c, r = tr.circle_center(pts)
    assert np.allclose(c, [0.3, -0.05]) and math.isclose(r, 0.12)
    # Arc of 60 degrees is enough with exact data.
    c, r = tr.circle_center(pts[:9])
    assert np.allclose(c, [0.3, -0.05], atol=1e-9)
    with pytest.raises(ValueError):
        tr.circle_center(pts[:2])


def test_tum_round_trip(tmp_path):
    stamps, poses = circle_path(20)
    path = str(tmp_path / 'x.tum')
    tr.write_tum(path, stamps, poses)
    s2, p2 = tr.read_tum(path)
    assert np.allclose(s2, stamps)
    for a, b in zip(poses, p2):
        assert np.allclose(a, b, atol=1e-5)
    assert math.isclose(tr.yaw_of(poses[0]), 1.57)


def test_body_offset_and_yaw_calibration():
    # Body mounted 0.2 m ahead, 0.05 m left and 0.4 m above base_footprint,
    # yawed 30 degrees with respect to the robot.
    body_in_base = make([0.2, 0.05, 0.4], rot([0, 0, 1], math.radians(30)))
    base_start = make([1.0, -1.0, 0.0], rot([0, 0, 1], 0.4))
    turn = [base_start @ make([0, 0, 0], rot([0, 0, 1], a)) @ body_in_base
            for a in np.linspace(0, 2.5, 60)]
    offset, radius = tr.body_offset_from_rotation(turn, base_height=0.0)
    expected = np.linalg.inv(body_in_base)[:3, 3]  # base in the body frame
    assert np.allclose(offset, expected, atol=1e-6)
    assert math.isclose(radius, math.hypot(0.2, 0.05), rel_tol=1e-6)
    drive = [base_start @ make([x, 0, 0], np.eye(3)) @ body_in_base
             for x in np.linspace(0, 1.0, 30)]
    yaw = tr.yaw_from_straight(drive)
    assert math.isclose(yaw, -math.radians(30), abs_tol=1e-9)  # robot x in the body frame
    with pytest.raises(ValueError):
        tr.body_offset_from_rotation(turn[:5])
    with pytest.raises(ValueError):
        tr.body_offset_from_rotation(turn[:12])  # less than 90 degrees
    with pytest.raises(ValueError):
        tr.yaw_from_straight(drive[:2])  # too short
    with pytest.raises(ValueError):
        tr.yaw_from_straight(turn)  # not straight
