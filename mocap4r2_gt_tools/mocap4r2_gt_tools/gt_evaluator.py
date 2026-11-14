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
Online error of a robot pose estimate against the mocap ground truth.

Samples TF (estimate: map -> base_link, ground truth: mocap -> base_link_gt),
publishes ATE/RPE on ~/error and writes both trajectories in TUM format (for
evo) when it ends.
"""

import os

from mocap4r2_gt_tools import trajectory as tr
from mocap4r2_gt_tools.ros_utils import from_transform, to_pose
from mocap4r2_interfaces.msg import TrajectoryError
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class GtEvaluator(Node):

    def __init__(self):
        super().__init__('gt_evaluator')
        self.est_parent = self.declare_parameter('estimate_parent', 'map').value
        self.est_child = self.declare_parameter('estimate_child', 'base_link').value
        self.gt_parent = self.declare_parameter('gt_parent', 'mocap').value
        self.gt_child = self.declare_parameter('gt_child', 'base_link_gt').value
        self.align = self.declare_parameter('align', True).value
        self.planar = self.declare_parameter('planar', True).value
        self.rpe_delta = int(self.declare_parameter('rpe_delta', 10).value)
        self.output_dir = self.declare_parameter('output_dir', '').value
        rate = float(self.declare_parameter('rate', 10.0).value)
        if rate <= 0 or self.rpe_delta < 1:
            raise ValueError('rate must be > 0 and rpe_delta >= 1')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.stamps, self.est, self.gt = [], [], []
        self.pub = self.create_publisher(TrajectoryError, '~/error', 10)
        self.create_timer(1.0 / rate, self.sample)
        self.create_timer(1.0, self.report)

    def sample(self):
        try:
            e = self.buffer.lookup_transform(self.est_parent, self.est_child, Time())
            stamp = Time.from_msg(e.header.stamp)
            # Ground truth at the same instant as the estimate (TF interpolates).
            g = self.buffer.lookup_transform(self.gt_parent, self.gt_child, stamp)
        except TransformException:
            return
        s = stamp.nanoseconds * 1e-9
        if self.stamps and s <= self.stamps[-1]:
            return
        self.stamps.append(s)
        self.est.append(from_transform(e))
        self.gt.append(from_transform(g))

    def compute(self) -> TrajectoryError:
        msg = TrajectoryError()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gt_parent
        msg.samples = len(self.est)
        if len(self.est) < 3:
            return msg
        a = tr.ate(self.est, self.gt, align=self.align, planar=self.planar)
        r = tr.rpe(self.est, self.gt, self.rpe_delta)
        msg.ate_rmse, msg.ate_mean, msg.ate_max = a['rmse'], a['mean'], a['max']
        msg.rpe_trans_rmse, msg.rpe_rot_rmse = r['trans_rmse'], r['rot_rmse']
        msg.aligned = bool(self.align)
        msg.alignment = to_pose(a['alignment'])
        return msg

    def report(self):
        self.pub.publish(self.compute())

    def save(self):
        if not self.output_dir or not self.est:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        tr.write_tum(os.path.join(self.output_dir, 'estimate.tum'), self.stamps, self.est)
        tr.write_tum(os.path.join(self.output_dir, 'groundtruth.tum'), self.stamps, self.gt)
        m = self.compute()
        self.get_logger().info(
            f'{m.samples} poses: ATE {m.ate_rmse:.4f} m, RPE {m.rpe_trans_rmse:.4f} m, '
            f'{m.rpe_rot_rmse:.4f} rad; TUM files in {self.output_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = GtEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()
