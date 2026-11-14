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

from mocap4r2_tools.verb import VerbExtension


class ExportVerb(VerbExtension):
    """Export a recorded session (bag) to CSV, TUM (rigid bodies) or C3D (markers)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('bag', help='Bag directory of the session')
        parser.add_argument('-f', '--format', choices=['csv', 'tum', 'c3d'], default='csv')
        parser.add_argument('-o', '--output', default='', help='Output directory (default: bag)')
        parser.add_argument('--lowpass', type=float, default=0.0,
                            help='Zero-phase Butterworth cutoff in Hz (0: none)')
        parser.add_argument('--max-gap', type=int, default=0,
                            help='Fill gaps of up to this many frames (0: none)')

    def main(self, *, args):
        from mocap4r2_motion_tools.export import export_session
        try:
            files = export_session(args.bag, args.output or args.bag, args.format,
                                   args.lowpass, args.max_gap)
        except (OSError, RuntimeError, ValueError) as e:
            return f'Error: {e}'
        if not files:
            return 'Error: no mocap data in the bag'
        for f in files:
            print(f)
        return 0
