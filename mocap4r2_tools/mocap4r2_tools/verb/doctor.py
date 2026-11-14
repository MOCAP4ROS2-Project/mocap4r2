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

from mocap4r2_tools import doctor
from mocap4r2_tools.client import mocap_node
from mocap4r2_tools.monitor import MocapMonitor
from mocap4r2_tools.verb import VerbExtension
from ros2param.api import call_get_parameters

MARK = {doctor.OK: '✔', doctor.WARN: '!', doctor.FAIL: '✘'}


class DoctorVerb(VerbExtension):
    """Check the mocap setup: ROS, systems, data and network."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--window', type=float, default=3.0,
                            help='Seconds observing the systems (default 3)')

    def main(self, *, args):
        results = doctor.check_ros_env()
        monitor = MocapMonitor('mocap4r2_doctor')
        try:
            time.sleep(args.window)
            systems, stats, _ = monitor.snapshot()
            if not systems:
                results.append((doctor.FAIL, 'no mocap systems announced on /mocap4r2/systems '
                                             '(driver running? same ROS_DOMAIN_ID?)'))
            for name in sorted(systems):
                results += doctor.check_stats(name, systems[name], stats.get(name))
            # Parameters are read with another node: the monitor one is being spun.
            with mocap_node('mocap4r2_doctor_params') as pnode:
                for name in sorted(systems):
                    try:
                        resp = call_get_parameters(
                            node=pnode, node_name=name,
                            parameter_names=list(doctor.ADDRESS_PARAMS))
                    except Exception:  # noqa: B902  node without a parameter service
                        continue
                    for value in resp.values:
                        if value.string_value:
                            results += doctor.check_server(name, value.string_value)
        finally:
            monitor.close()
        for level, text in results:
            print(f' {MARK[level]} {text}')
        return 1 if any(level == doctor.FAIL for level, _ in results) else 0
