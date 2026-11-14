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

from setuptools import find_packages
from setuptools import setup

package_name = 'mocap4r2_tools'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(include=[package_name, package_name + '.*'], exclude=['test']),
    include_package_data=True,
    package_data={package_name: ['vendor/**']},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    url='https://github.com/MOCAP4ROS2-Project/mocap4r2',
    description='Command line and terminal tools to monitor and control MOCAP4ROS2 systems',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'tui = mocap4r2_tools.tui.app:run_app',
        ],
        'ros2cli.command': [
            'mocap4r2 = mocap4r2_tools.command.mocap4r2:Mocap4r2Command',
        ],
        'ros2cli.extension_point': [
            'mocap4r2_tools.verb = mocap4r2_tools.verb:VerbExtension',
        ],
        'mocap4r2_tools.verb': [
            'list = mocap4r2_tools.verb.list:ListVerb',
            'info = mocap4r2_tools.verb.info:InfoVerb',
            'start = mocap4r2_tools.verb.control:StartVerb',
            'stop = mocap4r2_tools.verb.control:StopVerb',
            'bodies = mocap4r2_tools.verb.bodies:BodiesVerb',
            'hz = mocap4r2_tools.verb.hz:HzVerb',
            'record = mocap4r2_tools.verb.record:RecordVerb',
            'doctor = mocap4r2_tools.verb.doctor:DoctorVerb',
            'tui = mocap4r2_tools.verb.tui:TuiVerb',
            'export = mocap4r2_tools.verb.export:ExportVerb',
        ],
    },
)
