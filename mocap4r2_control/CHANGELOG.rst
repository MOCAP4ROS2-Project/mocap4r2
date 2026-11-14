^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* MocapSystemNode replaces ControlledLifecycleNode: it owns the lifecycle callbacks (drivers implement hooks), answers start/stop with the real transition result, announces the system on /mocap4r2/systems and publishes frame rate, latency, drops, tracking ratio, server and connection on /diagnostics.
* FramePublisher: one stamp and frame number per frame for markers, rigid bodies and skeletons, optional TF; thread safe for SDK threads.
* AuxiliarSystem: other devices (EMG, cameras...) join capture sessions.
* system_main: common main with autostart (none, configure, activate).
* Removed mocap4r2_control_msgs (now in mocap4r2_interfaces) and rqt_mocap4r2_control (replaced by the terminal UI).
* Modern CMake, C++17, YAML launch files; builds on Humble to Rolling.
* Contributors: Francisco Martín Rico, José Miguel Guerrero, Juan Carlos Manzanares Serrano, Manuel Fernández Carmona

0.0.7 (2024-01-25)
------------------

0.0.6 (2024-01-23)
------------------


0.0.5 (2024-01-23)
------------------


0.0.4 (2024-01-23)
------------------

0.0.3 (2024-01-21)
------------------
* Rename mocap to mocap4r2 to meet with REP 144
* Contributors: Francisco Martín Rico

0.0.2 (2024-01-09)
------------------
* Added node_options parameter to constructor with default value, to be used with mocap4ros2 vicon driver
* Initial commit
* Contributors: Francisco Martín Rico, MFernandezCarmona
