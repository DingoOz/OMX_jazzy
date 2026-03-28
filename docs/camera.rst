Camera Integration
==================

The workspace includes support for the Logitech C920 webcam with TF integration.

Setup
-----

Install udev Rules
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo cp 99-c920.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger

This creates a symlink ``/dev/c920`` pointing to your camera.

Find Your Camera
^^^^^^^^^^^^^^^^

.. code-block:: bash

   # List video devices
   v4l2-ctl --list-devices

   # Check available devices
   ls /dev/video*

Launching
---------

Camera Only
^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control camera.launch.py

   # With specific device
   ros2 launch omx_control camera.launch.py video_device:=/dev/video0

Hardware + Camera
^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch omx_control bringup_with_camera.launch.py port_name:=/dev/ttyUSB0

Simulation + Camera
^^^^^^^^^^^^^^^^^^^

For hybrid testing (simulated arm, real camera):

.. code-block:: bash

   ros2 launch omx_control simulation_with_camera.launch.py

Camera Parameters
-----------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

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
     - Image width (1920, 1280, 640)
   * - ``image_height``
     - 720
     - Image height (1080, 720, 480)
   * - ``framerate``
     - 30.0
     - Frames per second
   * - ``camera_frame_id``
     - camera_link
     - TF frame name

Camera Position Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^

The camera position is published as a static TF relative to ``link1`` (robot base):

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Parameter
     - Default
     - Description
   * - ``camera_x``
     - 0.0
     - X position in meters
   * - ``camera_y``
     - -0.3
     - Y position in meters
   * - ``camera_z``
     - 0.4
     - Z position in meters
   * - ``camera_roll``
     - 0.0
     - Roll angle in radians
   * - ``camera_pitch``
     - 0.5
     - Pitch angle in radians
   * - ``camera_yaw``
     - 1.57
     - Yaw angle in radians

Example camera position adjustment:

.. code-block:: bash

   ros2 launch omx_control bringup_with_camera.launch.py \
     port_name:=/dev/ttyUSB0 \
     camera_x:=0.0 \
     camera_y:=-0.25 \
     camera_z:=0.35 \
     camera_pitch:=0.6 \
     camera_yaw:=1.57

Topics
------

.. list-table::
   :header-rows: 1
   :widths: 35 35 30

   * - Topic
     - Type
     - Description
   * - ``/camera/image_raw``
     - sensor_msgs/Image
     - Raw image
   * - ``/camera/image_raw/compressed``
     - sensor_msgs/CompressedImage
     - JPEG compressed
   * - ``/camera/camera_info``
     - sensor_msgs/CameraInfo
     - Calibration data

Viewing Images
--------------

Using rqt_image_view
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 run rqt_image_view rqt_image_view

Then select ``/camera/image_raw`` from the dropdown.

Using RViz
^^^^^^^^^^

1. Launch with RViz enabled
2. Add an "Image" display
3. Set topic to ``/camera/image_raw``

Command Line
^^^^^^^^^^^^

.. code-block:: bash

   # Check topic is publishing
   ros2 topic hz /camera/image_raw

   # View camera info
   ros2 topic echo /camera/camera_info --once

Configuration
-------------

Camera settings can be adjusted in ``src/omx_control/config/c920_camera.yaml``:

.. code-block:: yaml

   camera_node:
     ros__parameters:
       video_device: "/dev/video0"
       framerate: 30.0
       image_width: 1280
       image_height: 720
       pixel_format: "yuyv"

       # Auto settings
       autofocus: true
       autoexposure: true
       auto_white_balance: true

       # Manual settings (when auto is off)
       brightness: 128
       contrast: 128
       saturation: 128
       focus: 0
       exposure: 166

Runtime Adjustments
^^^^^^^^^^^^^^^^^^^

Adjust settings at runtime using v4l2:

.. code-block:: bash

   # List available controls
   v4l2-ctl -d /dev/video0 --list-ctrls

   # Disable autofocus and set manual
   v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0,focus_absolute=30

   # Adjust exposure
   v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1,exposure_time_absolute=200

Camera Calibration
------------------

For accurate 3D perception, calibrate your camera:

.. code-block:: bash

   # Install calibration package
   sudo apt install ros-jazzy-camera-calibration

   # Run calibration (use a checkerboard)
   ros2 run camera_calibration cameracalibrator \
     --size 8x6 \
     --square 0.025 \
     --ros-args --remap image:=/camera/image_raw --remap camera:=/camera

Save the calibration file and update the camera config:

.. code-block:: yaml

   camera_node:
     ros__parameters:
       camera_info_url: "file:///home/user/.ros/camera_info/c920.yaml"

Troubleshooting
---------------

Camera not detected
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Check USB connection
   lsusb | grep Logitech

   # Check kernel messages
   dmesg | tail -20

   # Verify device exists
   ls -la /dev/video*

Image is dark/bright
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Disable auto-exposure
   v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1

   # Set manual exposure
   v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=300

Low framerate
^^^^^^^^^^^^^

Try a lower resolution:

.. code-block:: bash

   ros2 launch omx_control camera.launch.py image_width:=640 image_height:=480

Or use MJPEG format (edit c920_camera.yaml):

.. code-block:: yaml

   pixel_format: "mjpeg"
