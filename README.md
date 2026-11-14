# MOCAP4ROS2 core and tools

[![rolling](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/rolling.yaml) [![lyrical](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/lyrical.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/lyrical.yaml) [![kilted](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/kilted.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/kilted.yaml) [![jazzy](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/jazzy.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/jazzy.yaml) [![humble](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/humble.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4r2/actions/workflows/humble.yaml)

The system-independent part of [MOCAP4ROS2](https://mocap4ros2-project.github.io):

| Package | Content |
|---|---|
| `mocap4r2_control` | base node of every mocap system: lifecycle, start/stop, announcement, diagnostics, frame publishing |
| `mocap4r2_sim_driver` | simulated mocap system (noise, occlusions, drops, latency) |
| `mocap4r2_marker_viz` | RViz visualization of markers, rigid bodies and skeletons |
| `mocap4r2_robot_gt` | robot ground truth as a TF tree parallel to the robot's, with odometry |
| `mocap4r2_gt_tools` | localization error (ATE/RPE, TUM export), map-mocap alignment, body calibration |
| `mocap4r2_motion_tools` | skeleton TF and joint angles, filtering, export to CSV/TUM/C3D |
| `mocap4r2_tools` | `ros2 mocap4r2` (list, start, stop, record, export, doctor...) and the terminal UI |

Humble, Jazzy, Kilted, Lyrical and Rolling from the `rolling` branch.
Documentation: https://mocap4ros2-project.github.io

## Acknowledgements

MOCAP4ROS2 started as a Focused Technical Project of [ROSIN](https://www.rosin-project.eu/),
which received funding from the European Union’s Horizon 2020 research and innovation
programme under grant agreement no. 732287. Developed at the
[Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/), Rey Juan Carlos University.
