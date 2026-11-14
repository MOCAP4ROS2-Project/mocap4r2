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

from mocap4r2_tools.monitor import MocapMonitor
from mocap4r2_tools.verb import VerbExtension


class HzVerb(VerbExtension):
    """Frame rate, latency and dropped frames of the mocap systems."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--once', action='store_true', help='Print once and exit')
        parser.add_argument('--window', type=float, default=3.0,
                            help='Seconds of data before the first report (default 3)')

    def main(self, *, args):
        monitor = MocapMonitor('mocap4r2_hz')
        try:
            time.sleep(args.window)
            while True:
                systems, stats, _ = monitor.snapshot()
                if not systems:
                    print('No mocap systems found')
                for name in sorted(systems):
                    info, st = systems[name], stats.get(name)
                    if info.state.label != 'active' or not st:
                        print(f'{name}: {info.state.label}')
                        continue
                    nominal = f' (nominal {info.frame_rate:g})' if info.frame_rate > 0 else ''
                    print(f'{name}: {st.rate:.1f} Hz{nominal}  latency {st.latency_ms:.1f} ms  '
                          f'dropped {st.dropped} (total {st.total_dropped} of '
                          f'{st.total_frames})  {st.message}')
                    for body, ratio in sorted(st.tracking.items()):
                        print(f'    {body}: tracked {ratio * 100:.0f}%')
                if args.once:
                    return 0
                print()
                time.sleep(1.0)
        except KeyboardInterrupt:
            return 0
        finally:
            monitor.close()
