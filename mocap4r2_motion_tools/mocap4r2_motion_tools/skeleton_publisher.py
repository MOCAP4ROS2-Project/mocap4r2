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
Skeletons as TF (segments) and JointState (Cardan XYZ joint angles).

Subscribes to 'skeletons' (mocap4r2_interfaces/Skeletons) of a mocap system.
"""

from geometry_msgs.msg import TransformStamped
from mocap4r2_interfaces.msg import Skeletons
from mocap4r2_motion_tools.joint_angles import joint_angles
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class SkeletonPublisher(Node):

    def __init__(self):
        super().__init__('skeleton_publisher')
        self.prefix = self.declare_parameter('tf_prefix', '').value
        self.tf = TransformBroadcaster(self)
        self.joints = self.create_publisher(JointState, '~/joint_states', 10)
        self.create_subscription(
            Skeletons, 'skeletons', self.on_skeletons,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_skeletons(self, msg: Skeletons):
        transforms = []
        js = JointState()
        js.header = msg.header
        for sk in msg.skeletons:
            for seg in sk.segments:
                if not seg.tracking_valid:
                    continue
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = f'{self.prefix}{sk.name}/{seg.name}'
                p = seg.pose.position
                t.transform.translation.x, t.transform.translation.y = p.x, p.y
                t.transform.translation.z = p.z
                t.transform.rotation = seg.pose.orientation
                transforms.append(t)
            for name, (a, b, c) in joint_angles(sk.segments).items():
                for axis, v in zip('xyz', (a, b, c)):
                    js.name.append(f'{sk.name}/{name}_{axis}')
                    js.position.append(v)
        if transforms:
            self.tf.sendTransform(transforms)
        if js.name:
            self.joints.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
