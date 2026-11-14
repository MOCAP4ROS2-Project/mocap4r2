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
Calibration of the robot map frame against the mocap frame.

Drive the robot around while its localization runs: pairs of poses (map ->
base_link, mocap -> base_link_gt) give the rigid transform map -> mocap, which is
published as a static transform and saved to YAML. Then the ground truth and the
robot estimate live in one TF tree.
"""

import math

from mocap4r2_gt_tools import trajectory as tr
from mocap4r2_gt_tools.ros_utils import from_transform, to_transform, xyzrpy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
import yaml


class FrameAligner(Node):

    def __init__(self):
        super().__init__('frame_aligner')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.mocap_frame = self.declare_parameter('mocap_frame', 'mocap').value
        self.gt_frame = self.declare_parameter('gt_frame', 'base_link_gt').value
        self.planar = self.declare_parameter('planar', True).value
        self.min_step = float(self.declare_parameter('min_step', 0.05).value)
        self.min_samples = int(self.declare_parameter('min_samples', 30).value)
        self.output = self.declare_parameter('output', 'map_to_mocap.yaml').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.static = StaticTransformBroadcaster(self)
        self.src, self.dst = [], []
        self.create_timer(0.1, self.sample)
        self.create_service(Trigger, '~/compute', self.on_compute)
        self.get_logger().info(
            f'Drive the robot around ({self.min_samples} samples {self.min_step} m apart), '
            f'then call ~/compute')

    def sample(self):
        try:
            m = self.buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            g = self.buffer.lookup_transform(
                self.mocap_frame, self.gt_frame, Time.from_msg(m.header.stamp))
        except TransformException:
            return
        p_map, p_mocap = from_transform(m)[:3, 3], from_transform(g)[:3, 3]
        if self.dst and np.linalg.norm(p_map - self.dst[-1]) < self.min_step:
            return
        self.src.append(p_mocap)
        self.dst.append(p_map)

    def compute(self):
        """Transform map -> mocap (pose of the mocap frame in the map frame)."""
        if len(self.src) < self.min_samples:
            raise ValueError(f'{len(self.src)} samples, {self.min_samples} needed: keep moving')
        t = tr.umeyama(np.array(self.src), np.array(self.dst), planar=self.planar)
        residual = np.linalg.norm(
            (t[:3, :3] @ np.array(self.src).T).T + t[:3, 3] - np.array(self.dst), axis=1)
        return t, float(np.sqrt(np.mean(residual ** 2)))

    def on_compute(self, _, response):
        try:
            t, rmse = self.compute()
        except ValueError as e:
            response.success, response.message = False, str(e)
            return response
        self.static.sendTransform(to_transform(
            t, self.map_frame, self.mocap_frame, self.get_clock().now().to_msg()))
        x, y, z, roll, pitch, yaw = xyzrpy(t)
        with open(self.output, 'w') as f:
            yaml.safe_dump({'parent': self.map_frame, 'child': self.mocap_frame,
                            'xyz': [x, y, z], 'rpy': [roll, pitch, yaw],
                            'samples': len(self.src), 'rmse': rmse}, f)
        response.success = True
        response.message = (
            f'{self.map_frame} -> {self.mocap_frame}: xyz {x:.3f} {y:.3f} {z:.3f}, '
            f'yaw {math.degrees(yaw):.2f} deg, rmse {rmse:.3f} m over {len(self.src)} '
            f'samples (saved to {self.output})')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FrameAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
