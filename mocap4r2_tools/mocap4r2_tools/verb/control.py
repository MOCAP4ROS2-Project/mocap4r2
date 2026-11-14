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

from mocap4r2_interfaces.msg import Control
from mocap4r2_tools.client import mocap_node, MocapClient, MocapError
from mocap4r2_tools.verb import add_timeout_argument, VerbExtension


def run_control(args, command):
    with mocap_node() as node:
        client = MocapClient(node)
        systems = client.systems(min(args.timeout, 3.0), wait_for=args.systems)
        try:
            targets = client.resolve(args.systems, systems) if args.systems else []
            result = client.control(
                command, targets, session_id=args.session, timeout=args.timeout,
                systems=systems)
        except MocapError as e:
            return f'Error: {e}'
    verb = 'started' if command == Control.START else 'stopped'
    for name in sorted(result.acks):
        ack = result.acks[name]
        if ack.success:
            print(f'{name}: {verb}')
        else:
            print(f'{name}: FAILED ({ack.message})')
    for name in result.missing:
        print(f'{name}: no answer')
    return 0 if result.ok else 1


class StartVerb(VerbExtension):
    """Start mocap systems (all of them if none is given)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('systems', nargs='*', help='Systems to start (default: all)')
        parser.add_argument('--session', default='', help='Capture session id (e.g. trial name)')
        add_timeout_argument(parser, 5.0)

    def main(self, *, args):
        return run_control(args, Control.START)


class StopVerb(VerbExtension):
    """Stop mocap systems (all of them if none is given)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('systems', nargs='*', help='Systems to stop (default: all)')
        parser.add_argument('--session', default='', help='Capture session id')
        add_timeout_argument(parser, 5.0)

    def main(self, *, args):
        return run_control(args, Control.STOP)
