Usage Guide
===========

This guide covers all launch options and their parameters.

Hardware Launch
---------------

Basic Launch
^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0

Launch Parameters
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``port_name``
     - /dev/ttyUSB0
     - Serial port for U2D2 adapter
   * - ``start_rviz``
     - true
     - Launch RViz visualization
   * - ``use_fake_hardware``
     - false
     - Use simulated hardware interface
   * - ``velocity_scaling``
     - 0.5
     - Motion velocity scale (0.0-1.0)
   * - ``acceleration_scaling``
     - 0.5
     - Motion acceleration scale (0.0-1.0)

Example with Custom Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control bringup_all.launch.py \
     port_name:=/dev/ttyUSB0 \
     velocity_scaling:=0.3 \
     acceleration_scaling:=0.3 \
     start_rviz:=true

Simulation Launch
-----------------

Basic Simulation
^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control simulation.launch.py

Simulation Parameters
^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``world``
     - empty_world
     - Gazebo world file name
   * - ``start_rviz``
     - true
     - Launch RViz visualization
   * - ``velocity_scaling``
     - 0.5
     - Motion velocity scale (0.0-1.0)
   * - ``acceleration_scaling``
     - 0.5
     - Motion acceleration scale (0.0-1.0)

Camera Launch
-------------

Camera Only
^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control camera.launch.py

Hardware with Camera
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control bringup_with_camera.launch.py port_name:=/dev/ttyUSB0

Camera Parameters
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 20 55

   * - Parameter
     - Default
     - Description
   * - ``camera_name``
     - camera
     - Namespace for camera topics
   * - ``video_device``
     - /dev/video0
     - Camera device path
   * - ``image_width``
     - 1280
     - Image width in pixels
   * - ``image_height``
     - 720
     - Image height in pixels
   * - ``framerate``
     - 30.0
     - Camera framerate
   * - ``camera_x``
     - 0.0
     - Camera X position (m)
   * - ``camera_y``
     - -0.3
     - Camera Y position (m)
   * - ``camera_z``
     - 0.4
     - Camera Z position (m)
   * - ``camera_pitch``
     - 0.5
     - Camera pitch angle (rad)
   * - ``camera_yaw``
     - 1.57
     - Camera yaw angle (rad)

Step-by-Step Launch
-------------------

For more control, launch components separately:

**Terminal 1 - Hardware:**

.. code-block:: bash

   ros2 launch open_manipulator_bringup open_manipulator_x.launch.py port_name:=/dev/ttyUSB0

**Terminal 2 - MoveIt2:**

.. code-block:: bash

   ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py

**Terminal 3 - Controllers:**

.. code-block:: bash

   ros2 launch omx_control full_control.launch.py

**Terminal 4 - Camera (optional):**

.. code-block:: bash

   ros2 launch omx_control camera.launch.py

Useful Commands
---------------

Check Controller Status
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 control list_controllers

List Active Topics
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 topic list | grep omx

Check Node Status
^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 node list
