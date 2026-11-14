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

from mocap4r2_tools.client import mocap_node, MocapClient, MocapError
from mocap4r2_tools.verb import add_timeout_argument, VerbExtension


class InfoVerb(VerbExtension):
    """Show the details of a mocap system."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('system', help='Mocap system (fully qualified or plain node name)')
        add_timeout_argument(parser, 2.0)

    def main(self, *, args):
        with mocap_node() as node:
            client = MocapClient(node)
            systems = client.systems(args.timeout, wait_for=[args.system])
            try:
                name = client.resolve([args.system], systems)[0]
            except MocapError as e:
                return f'Error: {e}'
        s = systems[name]
        print(f'System:    {s.system}')
        print(f'Type:      {s.type}')
        print(f'State:     {s.state.label}')
        print(f'Frame:     {s.frame_id}')
        print(f'Rate:      {s.frame_rate:g} Hz' if s.frame_rate > 0 else 'Rate:      unknown')
        print(f'Session:   {s.session_id or "-"}')
        print('Topics:' if s.topics else 'Topics:    none')
        for topic, kind in zip(s.topics, s.topic_types):
            print(f'  {topic} [{kind or "?"}]')
        return 0
