Changelog
=========

All notable changes to this project will be documented in this file.

[1.0.0] - 2024-12-15
--------------------

Initial release with full feature set.

Added
^^^^^

* **IK Controller** - Inverse kinematics control via topics
* **FK Controller** - Forward kinematics control via topics
* **Gripper Controller** - Open/close and position control
* **MoveIt2 Integration** - Motion planning with collision avoidance
* **Gazebo Simulation** - Full physics simulation with Gazebo Harmonic
* **Camera Integration** - Logitech C920 webcam with TF

Launch Files
^^^^^^^^^^^^

* ``bringup_all.launch.py`` - Complete hardware launch
* ``bringup_with_camera.launch.py`` - Hardware with camera
* ``simulation.launch.py`` - Gazebo simulation
* ``simulation_with_camera.launch.py`` - Simulation with camera
* ``full_control.launch.py`` - Controllers only
* ``camera.launch.py`` - Camera only

Services
^^^^^^^^

* ``MoveToPosition.srv`` - IK control service
* ``MoveJoints.srv`` - FK control service
* ``GripperControl.srv`` - Gripper control service

Configuration
^^^^^^^^^^^^^

* ``control_params.yaml`` - Controller parameters
* ``c920_camera.yaml`` - Camera settings

Hardware Support
^^^^^^^^^^^^^^^^

* U2D2 USB-to-Dynamixel adapter
* Logitech C920 webcam
* udev rules for both devices

Dependencies
^^^^^^^^^^^^

Based on official ROBOTIS packages (jazzy branch):

* DynamixelSDK
* dynamixel_interfaces
* dynamixel_hardware_interface
* open_manipulator

Roadmap
-------

Future planned features:

* Python API wrapper
* ROS2 action interfaces
* Additional camera support
* Vision-based control examples
