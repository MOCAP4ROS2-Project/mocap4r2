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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        'estimate_parent': 'map', 'estimate_child': 'base_link',
        'gt_parent': 'mocap', 'gt_child': 'base_link_gt', 'output_dir': '',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [
            Node(
                package='mocap4r2_gt_tools',
                executable='gt_evaluator',
                parameters=[{k: LaunchConfiguration(k) for k in args}],
                output='screen'),
        ])
