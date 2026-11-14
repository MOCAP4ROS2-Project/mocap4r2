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

"""Evaluator and aligner against synthetic TF trajectories."""

import math
import os
import random
import time

from mocap4r2_gt_tools import trajectory as tr
from mocap4r2_gt_tools.frame_aligner import FrameAligner
from mocap4r2_gt_tools.gt_evaluator import GtEvaluator
from mocap4r2_gt_tools.ros_utils import to_transform, xyzrpy
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import TransformBroadcaster


@pytest.fixture
def ros():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(150, 180))
    rclpy.init()
    yield
    rclpy.try_shutdown()


def drive(executor, node, broadcaster, seconds, map_to_mocap, noise=0.0, error=0.0):
    """Robot on a circle: TF map->base_link (estimate) and mocap->base_link_gt."""
    rng = np.random.default_rng(3)
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        t = time.monotonic() - start
        stamp = node.get_clock().now().to_msg()
        gt_map = tr.pose([math.cos(0.8 * t), math.sin(0.8 * t), 0.0],
                         [0, 0, math.sin((0.8 * t + 1.57) / 2), math.cos((0.8 * t + 1.57) / 2)])
        gt_mocap = np.linalg.inv(map_to_mocap) @ gt_map
        est = gt_map.copy()
        est[0, 3] += error + rng.normal(scale=noise) if noise else error
        broadcaster.sendTransform([
            to_transform(est, 'map', 'base_link', stamp),
            to_transform(gt_mocap, 'mocap', 'base_link_gt', stamp)])
        executor.spin_once(timeout_sec=0.02)


def test_aligner_recovers_map_to_mocap(ros, tmp_path):
    map_to_mocap = tr.pose([2.0, -1.0, 0.0], [0, 0, math.sin(0.35), math.cos(0.35)])
    aligner = FrameAligner()
    aligner.output = str(tmp_path / 'a.yaml')
    aligner.min_samples = 20
    pub = rclpy.create_node('tf_pub')
    br = TransformBroadcaster(pub)
    ex = SingleThreadedExecutor()
    ex.add_node(aligner)
    ex.add_node(pub)
    drive(ex, pub, br, 6.0, map_to_mocap)
    t, rmse = aligner.compute()
    x, y, z, _, _, yaw = xyzrpy(t)
    assert rmse < 0.01
    assert abs(x - 2.0) < 0.01 and abs(y + 1.0) < 0.01 and abs(yaw - 0.7) < 0.01
    # The service writes the YAML and reports.
    from std_srvs.srv import Trigger
    resp = aligner.on_compute(Trigger.Request(), Trigger.Response())
    assert resp.success and os.path.exists(tmp_path / 'a.yaml')
    aligner.src = aligner.src[:3]
    assert not aligner.on_compute(Trigger.Request(), Trigger.Response()).success


def test_evaluator_measures_the_error(ros, tmp_path):
    map_to_mocap = tr.pose([0.5, 0.5, 0.0], [0, 0, 0, 1])
    ev = GtEvaluator()
    ev.output_dir = str(tmp_path / 'eval')
    ev.align = False
    ev.gt_parent = 'map'  # compare in one tree: publish the alignment as TF
    pub = rclpy.create_node('tf_pub2')
    br = TransformBroadcaster(pub)
    ex = SingleThreadedExecutor()
    ex.add_node(ev)
    ex.add_node(pub)
    from tf2_ros import StaticTransformBroadcaster
    st = StaticTransformBroadcaster(pub)
    st.sendTransform(to_transform(map_to_mocap, 'map', 'mocap', pub.get_clock().now().to_msg()))
    drive(ex, pub, br, 5.0, map_to_mocap, error=0.05)
    m = ev.compute()
    assert m.samples > 10
    assert abs(m.ate_rmse - 0.05) < 0.005  # constant 5 cm error
    assert m.rpe_trans_rmse < 0.005
    ev.save()
    s, p = tr.read_tum(str(tmp_path / 'eval' / 'groundtruth.tum'))
    assert len(s) == m.samples
