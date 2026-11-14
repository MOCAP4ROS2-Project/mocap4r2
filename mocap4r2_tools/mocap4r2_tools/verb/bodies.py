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

import time

from mocap4r2_interfaces.msg import RigidBodies
from mocap4r2_tools.client import mocap_node, MocapClient, MocapError
from mocap4r2_tools.monitor import DATA_QOS
from mocap4r2_tools.verb import add_timeout_argument, VerbExtension
import rclpy


def bodies_table(msg: RigidBodies) -> str:
    rows = [('BODY', 'TRACKED', 'X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW', 'ERR(mm)', 'MARKERS')]
    for rb in msg.rigid_bodies:
        p, q = rb.pose.position, rb.pose.orientation
        rows.append((rb.name, 'yes' if rb.tracking_valid else 'LOST', f'{p.x:+.4f}', f'{p.y:+.4f}',
                     f'{p.z:+.4f}', f'{q.x:+.3f}', f'{q.y:+.3f}', f'{q.z:+.3f}', f'{q.w:+.3f}',
                     f'{rb.mean_error * 1000:.2f}' if rb.mean_error >= 0 else '-',
                     str(len(rb.markers))))
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return '\n'.join('  '.join(c.ljust(w) for c, w in zip(r, widths)).rstrip() for r in rows)


class BodiesVerb(VerbExtension):
    """Show the rigid bodies of a mocap system."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('system', help='Mocap system')
        add_timeout_argument(parser, 3.0)

    def main(self, *, args):
        with mocap_node() as node:
            client = MocapClient(node)
            systems = client.systems(args.timeout, wait_for=[args.system])
            try:
                name = client.resolve([args.system], systems)[0]
            except MocapError as e:
                return f'Error: {e}'
            info = systems[name]
            topics = [t for t, k in zip(info.topics, info.topic_types)
                      if k.endswith('/RigidBodies')]
            if not topics:
                return f'Error: {name} publishes no rigid bodies'
            got = []
            node.create_subscription(RigidBodies, topics[0], got.append, DATA_QOS)
            end = time.monotonic() + args.timeout
            while not got and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not got:
                return f'Error: no data on {topics[0]} (is {name} started?)'
        msg = got[-1]
        print(f'{topics[0]}  frame {msg.frame_number}  [{msg.header.frame_id}]')
        print(bodies_table(msg))
        return 0
