Open Manipulator X - ROS2 Jazzy Documentation
=============================================

A complete ROS2 Jazzy workspace for controlling the ROBOTIS Open Manipulator X robotic arm with MoveIt2 motion planning.

Features
--------

* **IK Control** - Move end-effector to Cartesian poses
* **FK Control** - Direct joint angle control
* **Gripper Control** - Open/close and position control
* **MoveIt2 Integration** - Motion planning with collision avoidance
* **Gazebo Simulation** - Full physics simulation with Gazebo Harmonic
* **Camera Integration** - Logitech C920 webcam support with TF

Quick Start
-----------

.. code-block:: bash

   # Build the workspace
   cd ~/Programming/OMX_jazzy
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --allow-overriding dynamixel_sdk
   source install/setup.bash

   # Launch with hardware
   ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0

   # Or simulation only
   ros2 launch omx_control simulation.launch.py

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   installation
   quickstart

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   usage
   control_examples
   camera
   simulation
   adding_a_joint

.. toctree::
   :maxdepth: 2
   :caption: Reference

   api
   configuration
   troubleshooting

.. toctree::
   :maxdepth: 1
   :caption: About

   changelog
   license


Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
