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

package_name = 'mocap4r2_gt_tools'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/gt_evaluation.launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    url='https://github.com/MOCAP4ROS2-Project/mocap4r2',
    description='Ground-truth tools: error of robot localization (ATE/RPE), map-mocap '
                'alignment and calibration of the marker body on a robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gt_evaluator = mocap4r2_gt_tools.gt_evaluator:main',
            'frame_aligner = mocap4r2_gt_tools.frame_aligner:main',
            'body_calibrator = mocap4r2_gt_tools.body_calibrator:main',
        ],
    },
)
