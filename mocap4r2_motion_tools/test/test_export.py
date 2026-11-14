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

"""Export of a recorded bag to CSV, TUM and C3D."""

import csv
import os

from mocap4r2_interfaces.msg import Marker, Markers, RigidBodies, RigidBody
from mocap4r2_motion_tools.export import export_session
import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')


def write_bag(path, n=40):
    from rclpy.serialization import serialize_message
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    for name, kind in (('/markers', 'mocap4r2_interfaces/msg/Markers'),
                       ('/rigid_bodies', 'mocap4r2_interfaces/msg/RigidBodies')):
        try:  # Jazzy and newer: topic id first
            meta = rosbag2_py.TopicMetadata(id=0, name=name, type=kind, serialization_format='cdr')
        except TypeError:  # Humble
            meta = rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr')
        w.create_topic(meta)
    for i in range(n):
        t = i / 100.0
        ns = int(t * 1e9)
        m = Markers()
        m.header.stamp.sec, m.header.stamp.nanosec = 0, ns
        m.frame_number = i
        a, b = Marker(), Marker()
        a.name, a.position.x = 'robot:front', t
        b.name, b.occluded = 'robot:back', i == 5
        m.markers = [a, b]
        w.write('/markers', serialize_message(m), ns)
        rbs = RigidBodies()
        rbs.header = m.header
        rb = RigidBody()
        rb.name, rb.tracking_valid = 'robot', i % 10 != 0
        rb.pose.position.x, rb.pose.orientation.w = t, 1.0
        rbs.rigid_bodies = [rb]
        w.write('/rigid_bodies', serialize_message(rbs), ns)
    del w


def test_export_all_formats(tmp_path):
    bag = str(tmp_path / 'session')
    write_bag(bag)
    out = str(tmp_path / 'csv')
    files = export_session(bag, out, 'csv', lowpass=10.0, max_gap=3)
    names = sorted(os.path.basename(f) for f in files)
    assert names == ['markers.csv', 'rigid_bodies_robot.csv']
    with open(os.path.join(out, 'markers.csv')) as f:
        rows = list(csv.reader(f))
    assert rows[0] == ['stamp', 'robot:front_x', 'robot:front_y', 'robot:front_z',
                       'robot:back_x', 'robot:back_y', 'robot:back_z']
    assert len(rows) == 41
    assert rows[6][4] != 'nan'  # occluded sample at frame 5 filled (gap 1 <= 3)
    with open(os.path.join(out, 'rigid_bodies_robot.csv')) as f:
        assert len(list(csv.reader(f))) == 1 + 36  # untracked samples left out

    tum = export_session(bag, str(tmp_path / 'tum'), 'tum')
    assert [os.path.basename(f) for f in tum] == ['rigid_bodies_robot.tum']
    with open(tum[0]) as f:
        first = f.readline().split()
    assert len(first) == 8

    c3d_files = export_session(bag, str(tmp_path / 'c3d'), 'c3d')
    assert [os.path.basename(f) for f in c3d_files] == ['markers.c3d']
    assert os.path.getsize(c3d_files[0]) % 512 == 0
