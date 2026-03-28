Simulation
==========

The workspace includes full Gazebo Harmonic simulation for testing without hardware.

Prerequisites
-------------

Install Gazebo packages:

.. code-block:: bash

   sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control

Launching Simulation
--------------------

Basic Launch
^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control simulation.launch.py

This starts:

* Gazebo Harmonic physics simulation
* Robot model with ros2_control
* MoveIt2 motion planning
* IK/FK/Gripper controllers
* RViz2 visualization

With Camera
^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control simulation_with_camera.launch.py

This adds a real camera feed for hybrid testing.

Parameters
^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 20 55

   * - Parameter
     - Default
     - Description
   * - ``world``
     - empty_world
     - Gazebo world file
   * - ``start_rviz``
     - true
     - Launch RViz
   * - ``velocity_scaling``
     - 0.5
     - Motion velocity scale
   * - ``acceleration_scaling``
     - 0.5
     - Motion acceleration scale

Using the Simulation
--------------------

Once launched, control the robot exactly as you would with real hardware:

**FK Control:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -1.0, 0.7, 0.3]
   }"

**IK Control:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.2, y: 0.0, z: 0.2},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

**Gripper:**

.. code-block:: bash

   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"
   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"

Gazebo Interface
----------------

The Gazebo GUI provides additional controls:

* **Play/Pause** - Control simulation time
* **Reset** - Reset robot to initial position
* **Entity Inspector** - View model properties
* **Component Inspector** - View joint states

Gazebo Topics
^^^^^^^^^^^^^

Gazebo publishes additional topics:

.. code-block:: bash

   # List Gazebo topics
   ros2 topic list | grep gz

   # Clock (for simulation time)
   ros2 topic echo /clock

Custom Worlds
-------------

Place custom world files in the workspace and launch:

.. code-block:: bash

   ros2 launch omx_control simulation.launch.py world:=my_custom_world

Performance Tips
----------------

Headless Mode
^^^^^^^^^^^^^

For faster simulation without visualization:

.. code-block:: bash

   ros2 launch omx_control simulation.launch.py start_rviz:=false

Then run Gazebo in server mode (add to launch file or run separately).

Reduce Physics Rate
^^^^^^^^^^^^^^^^^^^

For slower computers, reduce the physics update rate in the world file.

Troubleshooting
---------------

Gazebo not starting
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Check Gazebo version
   gz sim --version

   # Run Gazebo standalone
   gz sim

   # Check for errors
   gz sim -v 4

Robot not spawning
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Check URDF is valid
   check_urdf /path/to/robot.urdf

   # Check ROS-Gazebo bridge
   ros2 topic list | grep gz

Controllers not loading
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # List controllers
   ros2 control list_controllers

   # Check controller manager
   ros2 service list | grep controller_manager

Slow simulation
^^^^^^^^^^^^^^^

* Reduce visualization (disable shadows, reduce quality)
* Use simpler collision meshes
* Reduce physics update rate
