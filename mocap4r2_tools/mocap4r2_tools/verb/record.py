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

import datetime
import os
import signal
import subprocess

from mocap4r2_interfaces.msg import Control
from mocap4r2_tools.client import mocap_node, MocapClient, MocapError
from mocap4r2_tools.verb import add_timeout_argument, VerbExtension
import yaml

BASE_TOPICS = ['/tf', '/tf_static', '/mocap4r2/systems', '/diagnostics']


def topic_args(topics):
    """Topics as rosbag2 wants them: --topics from Jazzy, positional in Humble."""
    try:
        help_text = subprocess.run(['ros2', 'bag', 'record', '--help'], capture_output=True,
                                   text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        help_text = ''
    return (['--topics'] + topics) if '--topics' in help_text else topics


def session_metadata(session, systems, start, end, bag_dir):
    return {
        'mocap4r2_session': {
            'session': session,
            'start': start.isoformat(),
            'end': end.isoformat(),
            'bag': os.path.basename(bag_dir),
            'systems': [{
                'system': s.system, 'type': s.type, 'frame_id': s.frame_id,
                'frame_rate': float(s.frame_rate),
                'topics': [{'name': t, 'type': k} for t, k in zip(s.topics, s.topic_types)],
            } for s in systems],
        }
    }


class RecordVerb(VerbExtension):
    """Start mocap systems in a session and record their data with rosbag2."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('systems', nargs='*', help='Systems to record (default: all)')
        parser.add_argument('--session', default='', help='Session id (default: date and time)')
        parser.add_argument('-o', '--output', default='', help='Bag directory (default: session)')
        parser.add_argument('--topics', nargs='*', default=[],
                            help='Extra topics to record (e.g. robot odometry)')
        parser.add_argument('--no-control', action='store_true',
                            help='Do not START/STOP the systems, only record')
        add_timeout_argument(parser, 5.0)

    def main(self, *, args):
        session = args.session or datetime.datetime.now().strftime('mocap_%Y%m%d_%H%M%S')
        out = args.output or session
        if os.path.exists(out):
            return f'Error: {out} already exists'
        with mocap_node('mocap4r2_record', handle_signals=False) as node:
            client = MocapClient(node)
            systems = client.systems(3.0, wait_for=args.systems)
            try:
                targets = client.resolve(args.systems, systems) if args.systems else \
                    sorted(systems)
            except MocapError as e:
                return f'Error: {e}'
            if not targets:
                return 'Error: no mocap systems found'
            if not args.no_control:
                result = client.control(Control.START, targets, session_id=session,
                                        timeout=args.timeout, systems=systems)
                if not result.ok:
                    client.control(Control.STOP, targets, timeout=args.timeout, systems=systems)
                    failed = [n for n, a in result.acks.items() if not a.success] + result.missing
                    return f'Error: cannot start {", ".join(failed)}'
            systems = client.systems(2.0)
            infos = [systems[t] for t in targets if t in systems]
            topics = sorted(set(BASE_TOPICS + args.topics + [t for i in infos for t in i.topics]))
            print(f'Recording session "{session}" into {out}:')
            for t in topics:
                print(f'  {t}')
            start = datetime.datetime.now()
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', out] + topic_args(topics))
            try:
                bag.wait()
            except KeyboardInterrupt:
                bag.send_signal(signal.SIGINT)
                try:
                    bag.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    bag.kill()
            end = datetime.datetime.now()
            if not args.no_control:
                client.control(Control.STOP, targets, session_id=session, timeout=args.timeout,
                               systems=systems)
        if os.path.isdir(out):
            with open(os.path.join(out, 'mocap4r2_session.yaml'), 'w') as f:
                yaml.safe_dump(session_metadata(session, infos, start, end, out), f,
                               sort_keys=False)
        print(f'Session "{session}" recorded ({(end - start).total_seconds():.1f} s)')
        return 0
