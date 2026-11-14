^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_robot_gt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* Rewritten: ground truth as a TF tree parallel to the robot (mocap -> gt_origin -> <robot>_gt), rigid body selected by name, untracked or missing bodies skipped.
* Fixed RPY order (setRPY), origin offset composition and use of uninitialized transforms.
* Body to robot transform from a parameter or from TF; ~/set_origin service; ~/pose and ~/odom with filtered velocities.
* Removed mocap4r2_robot_gt_msgs (SetOrigin is in mocap4r2_interfaces).
* Contributors: Francisco Martín Rico, José Miguel Guerrero, Juan Carlos Manzanares Serrano, Manuel Fernández Carmona
