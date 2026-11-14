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
Calibration of the marker body on a robot (parameter body_to_robot of robot_gt).

1. ~/reset, then turn the robot in place at least 90 degrees (the base stays put).
2. ~/turn_done, then (optional) drive it straight forward at least 0.3 m.
3. ~/compute: prints and saves body_to_robot = [x, y, z, roll, pitch, yaw].
"""

import math

from mocap4r2_gt_tools import trajectory as tr
from mocap4r2_gt_tools.ros_utils import from_pose
from mocap4r2_interfaces.msg import RigidBodies
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import yaml


class BodyCalibrator(Node):

    def __init__(self):
        super().__init__('body_calibrator')
        self.body = self.declare_parameter('rigid_body', 'robot').value
        self.base_height = float(self.declare_parameter('base_height', 0.0).value)
        self.output = self.declare_parameter('output', 'body_to_robot.yaml').value
        self.turn, self.drive = [], []
        self.turning = True
        self.create_subscription(
            RigidBodies, 'rigid_bodies', self.on_bodies,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_service(Trigger, '~/compute', self.on_compute)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_service(Trigger, '~/turn_done', self.on_turn_done)
        self.get_logger().info(
            'Turn the robot in place, call ~/turn_done, drive straight, call ~/compute')

    def on_bodies(self, msg):
        for rb in msg.rigid_bodies:
            if rb.name == self.body and rb.tracking_valid:
                (self.turn if self.turning else self.drive).append(from_pose(rb.pose))

    def on_reset(self, _, response):
        self.turn, self.drive = [], []
        self.turning = True
        response.success, response.message = True, 'recording the turn in place'
        return response

    def on_turn_done(self, _, response):
        self.turning = False
        response.success = True
        response.message = f'{len(self.turn)} turn poses; now drive straight forward'
        return response

    def on_compute(self, _, response):
        try:
            offset, radius = tr.body_offset_from_rotation(self.turn, self.base_height)
            yaw, note = 0.0, 'no straight drive recorded: yaw 0 assumed'
            if self.drive:
                yaw = tr.yaw_from_straight(self.drive)
                note = f'yaw from a straight drive of {len(self.drive)} poses'
        except ValueError as e:
            response.success, response.message = False, str(e)
            return response
        # body_to_robot is the robot frame in the body frame: offset, then yaw.
        value = [float(offset[0]), float(offset[1]), float(offset[2]), 0.0, 0.0, float(yaw)]
        with open(self.output, 'w') as f:
            yaml.safe_dump({'rigid_body': self.body, 'body_to_robot': value,
                            'turn_radius': radius}, f)
        response.success = True
        response.message = (
            f'body_to_robot: [{value[0]:.4f}, {value[1]:.4f}, {value[2]:.4f}, 0, 0, '
            f'{value[5]:.4f}] ({math.degrees(yaw):.1f} deg; {note}; saved to {self.output})')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BodyCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
