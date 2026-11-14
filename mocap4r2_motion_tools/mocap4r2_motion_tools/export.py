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
Export of a recorded session (rosbag2) to CSV, TUM or C3D.

Reads the mocap4r2_interfaces Markers and RigidBodies messages of the bag.
"""

import csv
import os
from typing import Dict, List

from mocap4r2_motion_tools import c3d
from mocap4r2_motion_tools.filters import fill_gaps, filtfilt
import numpy as np

MARKERS = 'mocap4r2_interfaces/msg/Markers'
BODIES = 'mocap4r2_interfaces/msg/RigidBodies'


def read_bag(path: str):
    """{topic: (type, [(stamp_s, msg)])} of the mocap topics of a bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {n: t for n, t in types.items() if t in (MARKERS, BODIES)}
    classes = {n: get_message(t) for n, t in wanted.items()}
    out = {n: (t, []) for n, t in wanted.items()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in classes:
            msg = deserialize_message(data, classes[topic])
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            out[topic][1].append((stamp, msg))
    return out


def marker_table(messages) -> (np.ndarray, List[str], np.ndarray):
    """(stamps, labels, frames x labels x 3 with NaN when missing) of named markers."""
    labels: Dict[str, int] = {}
    for _, m in messages:
        for mk in m.markers:
            if mk.name and mk.name not in labels:
                labels[mk.name] = len(labels)
    stamps = np.array([s for s, _ in messages])
    pts = np.full((len(messages), len(labels), 3), np.nan)
    for i, (_, m) in enumerate(messages):
        for mk in m.markers:
            if mk.name and not mk.occluded:
                pts[i, labels[mk.name]] = (mk.position.x, mk.position.y, mk.position.z)
    return stamps, list(labels), pts


def body_tables(messages) -> Dict[str, list]:
    """{body: [(stamp, x, y, z, qx, qy, qz, qw)]} of tracked samples."""
    out: Dict[str, list] = {}
    for stamp, m in messages:
        for rb in m.rigid_bodies:
            if rb.tracking_valid:
                p, q = rb.pose.position, rb.pose.orientation
                out.setdefault(rb.name, []).append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return out


def process(stamps, pts, lowpass: float = 0.0, max_gap: int = 0):
    """Gap filling and zero-phase low-pass of every coordinate."""
    if len(stamps) < 2:
        return pts
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if stamps[-1] > stamps[0] else 0.0
    out = pts.copy()
    for j in range(pts.shape[1]):
        for k in range(3):
            x = fill_gaps(pts[:, j, k], max_gap) if max_gap else pts[:, j, k]
            if lowpass and rate and not np.isnan(x).any():
                x = filtfilt(lowpass, rate, x)
            out[:, j, k] = x
    return out


def export_session(bag: str, out_dir: str, fmt: str, lowpass: float = 0.0,
                   max_gap: int = 0) -> List[str]:
    """Export every mocap topic of the bag; returns the files written."""
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for topic, (kind, messages) in sorted(read_bag(bag).items()):
        if not messages:
            continue
        base = os.path.join(out_dir, topic.strip('/').replace('/', '_'))
        if kind == MARKERS:
            stamps, labels, pts = marker_table(messages)
            pts = process(stamps, pts, lowpass, max_gap)
            if fmt == 'c3d':
                rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 1.0
                c3d.write_c3d(base + '.c3d', labels, pts, rate)
                written.append(base + '.c3d')
            elif fmt == 'csv':
                with open(base + '.csv', 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['stamp'] + [f'{n}_{a}' for n in labels for a in 'xyz'])
                    for s, row in zip(stamps, pts):
                        w.writerow([f'{s:.9f}'] + [f'{v:.6f}' for v in row.reshape(-1)])
                written.append(base + '.csv')
        elif kind == BODIES and fmt in ('csv', 'tum'):
            for body, rows in body_tables(messages).items():
                path = f'{base}_{body}.' + ('tum' if fmt == 'tum' else 'csv')
                with open(path, 'w', newline='') as f:
                    if fmt == 'tum':
                        for r in rows:
                            f.write(' '.join(f'{v:.9f}' for v in r) + '\n')
                    else:
                        w = csv.writer(f)
                        w.writerow(['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                        w.writerows(rows)
                written.append(path)
    return written
