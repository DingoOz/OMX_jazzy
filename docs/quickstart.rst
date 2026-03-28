Quick Start Guide
=================

This guide helps you get the Open Manipulator X running quickly.

Launch Options Summary
----------------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Command
     - Description
   * - ``ros2 launch omx_control bringup_all.launch.py``
     - Hardware + MoveIt2 + Controllers
   * - ``ros2 launch omx_control simulation.launch.py``
     - Gazebo + MoveIt2 + Controllers
   * - ``ros2 launch omx_control bringup_with_camera.launch.py``
     - Hardware + Camera
   * - ``ros2 launch omx_control simulation_with_camera.launch.py``
     - Simulation + Camera
   * - ``ros2 launch omx_control full_control.launch.py``
     - Controllers only
   * - ``ros2 launch omx_control camera.launch.py``
     - Camera only

Option 1: Simulation (No Hardware)
----------------------------------

The easiest way to get started without any hardware:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source ~/Programming/OMX_jazzy/install/setup.bash
   ros2 launch omx_control simulation.launch.py

This launches:

* Gazebo Harmonic physics simulation
* MoveIt2 motion planning
* IK/FK/Gripper controllers
* RViz visualization

Option 2: Hardware
------------------

Connect your U2D2 and power on the robot:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source ~/Programming/OMX_jazzy/install/setup.bash
   ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0

.. warning::
   Ensure the robot arm is in a safe position before launching.
   The arm will move to its initial position on startup.

Option 3: Fake Hardware
-----------------------

Test control interfaces without a real robot:

.. code-block:: bash

   ros2 launch omx_control bringup_all.launch.py use_fake_hardware:=true

First Commands
--------------

Once launched, try these commands in a new terminal:

**Move to home position (FK):**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -1.0, 0.7, 0.3]
   }"

**Open gripper:**

.. code-block:: bash

   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

**Close gripper:**

.. code-block:: bash

   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"

**Move to position (IK):**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.2, y: 0.0, z: 0.2},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

What's Next?
------------

* :doc:`usage` - Detailed launch options
* :doc:`control_examples` - More control examples
* :doc:`camera` - Camera integration
* :doc:`api` - Full API reference
