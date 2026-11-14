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

from mocap4r2_tools.client import mocap_node, MocapClient
from mocap4r2_tools.verb import add_timeout_argument, VerbExtension


class ListVerb(VerbExtension):
    """List the mocap systems and their state."""

    def add_arguments(self, parser, cli_name):
        add_timeout_argument(parser, 2.0)

    def main(self, *, args):
        with mocap_node() as node:
            systems = MocapClient(node).systems(args.timeout)
        if not systems:
            print('No mocap systems found')
            return 0
        rows = [('SYSTEM', 'TYPE', 'STATE', 'FRAME', 'RATE', 'SESSION')]
        for name in sorted(systems):
            s = systems[name]
            rate = f'{s.frame_rate:g} Hz' if s.frame_rate > 0 else '-'
            rows.append((name, s.type, s.state.label, s.frame_id, rate, s.session_id or '-'))
        widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
        for r in rows:
            print('  '.join(c.ljust(w) for c, w in zip(r, widths)).rstrip())
        return 0
