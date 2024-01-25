# Copyright 2024 Intelligent Robotics Lab
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

from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from mocap4r2cli.api import get_mocap4r2_status
from mocap4r2cli.verb import VerbExtension


class StatusVerb(VerbExtension):
    """Prints the status of the MOCAP4ROS2 systems."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        
        pass
        # parser.add_argument(
        #           '--simple', '-s', action='store_true',
        #           help="Display the message in a simple form.")

    def main(self, *, args):
        with NodeStrategy(args) as node:
            status = get_mocap4r2_status(node=node)
        
        print(status)
