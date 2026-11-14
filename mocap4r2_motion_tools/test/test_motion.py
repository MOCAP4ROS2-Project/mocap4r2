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
import struct

from mocap4r2_interfaces.msg import Segment
from mocap4r2_motion_tools import c3d
from mocap4r2_motion_tools import filters
from mocap4r2_motion_tools.joint_angles import cardan_xyz, joint_angles, quat_to_matrix
import numpy as np
import pytest


def test_butterworth_response():
    rate = 200.0
    t = np.arange(0, 5, 1 / rate)
    slow = np.sin(2 * math.pi * 1.0 * t)
    fast = np.sin(2 * math.pi * 40.0 * t)
    y = filters.filtfilt(6.0, rate, slow + fast)
    core = slice(100, -100)
    assert np.max(np.abs(y[core] - slow[core])) < 0.02  # 1 Hz kept, 40 Hz removed
    # Zero phase: the filtered peak is where the original one is.
    assert abs(np.argmax(y[200:400]) - np.argmax(slow[200:400])) <= 1
    const = filters.filtfilt(6.0, rate, np.full(300, 2.5))
    assert np.allclose(const, 2.5)
    assert filters.filtfilt(6.0, rate, [1.0, 2.0]).tolist() == [1.0, 2.0]
    with pytest.raises(ValueError):
        filters.butter2_lowpass(150.0, 200.0)
    with pytest.raises(ValueError):
        filters.butter2_lowpass(0.0, 200.0)


def test_fill_gaps_and_derivative():
    x = np.array([0.0, 1.0, np.nan, np.nan, 4.0, np.nan, np.nan, np.nan, np.nan, 9.0, np.nan])
    y = filters.fill_gaps(x, 3)
    assert y[:5].tolist() == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert np.isnan(y[5:9]).all()  # gap of 4 > 3 stays
    assert np.isnan(y[10])        # trailing gap cannot be interpolated
    assert np.isnan(filters.fill_gaps([np.nan, 1.0], 5)[0])  # leading gap either
    v = filters.derivative(np.arange(10) * 0.5, 100.0)
    assert np.allclose(v, 50.0)
    assert filters.derivative([1.0], 100.0).tolist() == [0.0]


def read_c3d(path):
    """Tiny reader for the test: header, POINT parameters and data."""
    with open(path, 'rb') as f:
        data = f.read()
    pblock, key = data[0], data[1]
    n_points, _, first, last = struct.unpack_from('<hhhh', data, 2)
    scale, = struct.unpack_from('<f', data, 12)
    start, = struct.unpack_from('<h', data, 16)
    rate, = struct.unpack_from('<f', data, 20)
    p = (pblock - 1) * 512
    assert data[p + 1] == 0x50 and data[p + 3] == 84
    params, groups, i = {}, {}, p + 4
    while True:
        nlen, gid = struct.unpack_from('<bb', data, i)
        if nlen == 0:
            break
        name = data[i + 2:i + 2 + abs(nlen)].decode()
        off, = struct.unpack_from('<h', data, i + 2 + abs(nlen))
        body = i + 4 + abs(nlen)
        if gid < 0:
            groups[-gid] = name
        else:
            kind, ndim = struct.unpack_from('<bB', data, body)
            dims = list(data[body + 2:body + 2 + ndim])
            raw = data[body + 2 + ndim:]
            params[(gid, name)] = (kind, dims, raw)
        if off == 0:
            break
        i = i + 2 + abs(nlen) + off
    return {'key': key, 'n_points': n_points, 'first': first, 'last': last, 'scale': scale,
            'start': start, 'rate': rate, 'groups': groups, 'params': params, 'raw': data}


def test_c3d_round_trip(tmp_path):
    frames, labels = 50, ['robot:front', 'robot:back', 'heel']
    pts = np.random.default_rng(0).uniform(-2, 2, size=(frames, 3, 3))
    pts[10, 1] = np.nan  # a missing sample
    path = str(tmp_path / 'trial.c3d')
    c3d.write_c3d(path, labels, pts, 120.0)
    h = read_c3d(path)
    assert h['key'] == 0x50 and h['n_points'] == 3 and h['first'] == 1 and h['last'] == frames
    assert h['scale'] < 0 and math.isclose(h['rate'], 120.0)
    assert h['groups'][1] == 'POINT'
    kind, dims, raw = h['params'][(1, 'LABELS')]
    assert kind == -1 and dims == [30, 3]
    assert raw[:30].decode().strip() == 'robot:front'
    kind, dims, raw = h['params'][(1, 'DATA_START')]
    assert struct.unpack_from('<h', raw)[0] == h['start']
    values = np.frombuffer(h['raw'][(h['start'] - 1) * 512:], dtype='<f4')[:frames * 3 * 4]
    values = values.reshape(frames, 3, 4)
    assert np.allclose(values[:, :, :3][values[:, :, 3] >= 0],
                       (pts * 1000.0)[values[:, :, 3] >= 0], atol=1e-2)
    assert values[10, 1, 3] == -1.0 and values[11, 1, 3] == 0.0
    kind, dims, raw = h['params'][(3, 'ACTUAL_END_FIELD')]
    assert struct.unpack_from('<HH', raw) == (frames, 0)
    with pytest.raises(ValueError):
        c3d.write_c3d(path, ['a'], pts, 120.0)
    with pytest.raises(ValueError):
        c3d.write_c3d(path, labels, pts, 0.0)


def rot_q(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return [*(a * math.sin(angle / 2)), math.cos(angle / 2)]


def test_cardan_angles():
    rx = quat_to_matrix(rot_q([1, 0, 0], 0.3))
    ry = quat_to_matrix(rot_q([0, 1, 0], -0.4))
    rz = quat_to_matrix(rot_q([0, 0, 1], 1.1))
    a, b, c = cardan_xyz(rx @ ry @ rz)
    assert np.allclose([a, b, c], [0.3, -0.4, 1.1])
    a, b, c = cardan_xyz(quat_to_matrix(rot_q([0, 1, 0], math.pi / 2)))
    assert math.isclose(b, math.pi / 2) and c == 0.0  # gimbal lock handled


def seg(i, parent, q, valid=True):
    s = Segment()
    s.id, s.parent_id, s.name, s.tracking_valid = i, parent, f's{i}', valid
    s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z, s.pose.orientation.w = q
    return s


def test_joint_angles_relative_to_parent():
    hip = seg(1, Segment.NO_PARENT, rot_q([0, 0, 1], 0.5))
    # Knee flexed 0.7 rad about its local X, on top of the hip yaw.
    r = quat_to_matrix(rot_q([0, 0, 1], 0.5)) @ quat_to_matrix(rot_q([1, 0, 0], 0.7))
    w = math.sqrt(1 + np.trace(r)) / 2
    knee_q = [(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w),
              (r[1, 0] - r[0, 1]) / (4 * w), w]
    knee = seg(2, 1, knee_q)
    lost = seg(3, 2, rot_q([1, 0, 0], 0.1), valid=False)
    orphan = seg(4, 99, rot_q([1, 0, 0], 0.1))
    angles = joint_angles([hip, knee, lost, orphan])
    assert set(angles) == {'s2'}
    assert np.allclose(angles['s2'], [0.7, 0.0, 0.0], atol=1e-9)
