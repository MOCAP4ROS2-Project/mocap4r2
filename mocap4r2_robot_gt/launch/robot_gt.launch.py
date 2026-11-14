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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mocap4r2_robot_gt')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_dir, 'config', 'robot_gt.yaml')),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace of the mocap system topics'),
        Node(
            package='mocap4r2_robot_gt',
            executable='robot_gt',
            name='robot_gt',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen'),
    ])
