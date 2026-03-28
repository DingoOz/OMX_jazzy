Troubleshooting
===============

This page covers common issues and solutions.

Hardware Issues
---------------

Permission denied on /dev/ttyUSB0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Symptom:** Cannot open serial port

**Quick fix:**

.. code-block:: bash

   sudo chmod 666 /dev/ttyUSB0

**Permanent fix:**

.. code-block:: bash

   sudo usermod -aG dialout $USER
   # Log out and back in

**With udev rules:**

.. code-block:: bash

   sudo cp 99-u2d2.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger

Robot not responding
^^^^^^^^^^^^^^^^^^^^

**Check connection:**

.. code-block:: bash

   ls /dev/ttyUSB*

**Check power:** Ensure the robot arm is powered on.

**Check communication:**

.. code-block:: bash

   # Try a simple serial test
   python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 1000000); print('OK')"

Robot moves erratically
^^^^^^^^^^^^^^^^^^^^^^^

1. Reduce velocity scaling:

   .. code-block:: bash

      ros2 launch omx_control bringup_all.launch.py velocity_scaling:=0.2

2. Check joint limits are not exceeded
3. Verify power supply is adequate

ROS2 Issues
-----------

MoveIt services not available
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Check if MoveIt is running:**

.. code-block:: bash

   ros2 service list | grep compute_ik

**Start MoveIt separately:**

.. code-block:: bash

   ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py

Controllers not loading
^^^^^^^^^^^^^^^^^^^^^^^

**List controllers:**

.. code-block:: bash

   ros2 control list_controllers

**Check controller manager:**

.. code-block:: bash

   ros2 service list | grep controller_manager

**Restart controllers:**

.. code-block:: bash

   ros2 control switch_controllers --stop arm_controller gripper_controller
   ros2 control switch_controllers --start arm_controller gripper_controller

Topics not publishing
^^^^^^^^^^^^^^^^^^^^^

**List topics:**

.. code-block:: bash

   ros2 topic list

**Check topic frequency:**

.. code-block:: bash

   ros2 topic hz /joint_states

**Verify node is running:**

.. code-block:: bash

   ros2 node list

TF not available
^^^^^^^^^^^^^^^^

**View TF tree:**

.. code-block:: bash

   ros2 run tf2_tools view_frames

**Check specific transform:**

.. code-block:: bash

   ros2 run tf2_ros tf2_echo link1 end_effector_link

Camera Issues
-------------

Camera not detected
^^^^^^^^^^^^^^^^^^^

**Check USB:**

.. code-block:: bash

   lsusb | grep Logitech

**List video devices:**

.. code-block:: bash

   v4l2-ctl --list-devices

**Check kernel messages:**

.. code-block:: bash

   dmesg | tail -20

Camera image is black
^^^^^^^^^^^^^^^^^^^^^

**Test with ffplay:**

.. code-block:: bash

   ffplay /dev/video0

**Check permissions:**

.. code-block:: bash

   ls -la /dev/video0
   sudo chmod 666 /dev/video0

Image too dark/bright
^^^^^^^^^^^^^^^^^^^^^

**Adjust exposure:**

.. code-block:: bash

   v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
   v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=300

**List available controls:**

.. code-block:: bash

   v4l2-ctl -d /dev/video0 --list-ctrls

Low framerate
^^^^^^^^^^^^^

**Reduce resolution:**

.. code-block:: bash

   ros2 launch omx_control camera.launch.py image_width:=640 image_height:=480

**Use MJPEG format:** Edit ``c920_camera.yaml``:

.. code-block:: yaml

   pixel_format: "mjpeg"

Simulation Issues
-----------------

Gazebo not starting
^^^^^^^^^^^^^^^^^^^

**Check installation:**

.. code-block:: bash

   gz sim --version

**Install if missing:**

.. code-block:: bash

   sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control

**Run with verbose output:**

.. code-block:: bash

   gz sim -v 4

Robot not spawning in Gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Check URDF validity:**

.. code-block:: bash

   check_urdf /path/to/robot.urdf

**Check Gazebo resource path:**

.. code-block:: bash

   echo $GZ_SIM_RESOURCE_PATH

Simulation is slow
^^^^^^^^^^^^^^^^^^

1. Disable shadows in Gazebo GUI
2. Reduce physics update rate
3. Use headless mode:

   .. code-block:: bash

      ros2 launch omx_control simulation.launch.py start_rviz:=false

Build Issues
------------

Package not found
^^^^^^^^^^^^^^^^^

**Source workspace:**

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source ~/Programming/OMX_jazzy/install/setup.bash

**Rebuild:**

.. code-block:: bash

   colcon build --symlink-install

Missing dependencies
^^^^^^^^^^^^^^^^^^^^

**Install rosdep dependencies:**

.. code-block:: bash

   rosdep install --from-paths src --ignore-src -r -y

Build fails
^^^^^^^^^^^

**Clean and rebuild:**

.. code-block:: bash

   rm -rf build/ install/ log/
   colcon build --symlink-install

Getting Help
------------

**Check ROS logs:**

.. code-block:: bash

   ros2 run rqt_console rqt_console

**Enable debug output:**

.. code-block:: bash

   export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

**View node info:**

.. code-block:: bash

   ros2 node info /ik_controller
