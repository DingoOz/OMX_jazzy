Configuration
=============

This page documents all configuration options.

Control Parameters
------------------

File: ``src/omx_control/config/control_params.yaml``

IK Controller
^^^^^^^^^^^^^

.. code-block:: yaml

   ik_controller:
     ros__parameters:
       # Motion scaling (0.0-1.0)
       velocity_scaling: 0.5
       acceleration_scaling: 0.5

       # Planning parameters
       planning_time: 5.0           # Max planning time (seconds)
       num_planning_attempts: 10    # Number of planning attempts

       # Robot configuration
       planning_group: "arm"
       end_effector_link: "end_effector_link"
       base_link: "link1"

FK Controller
^^^^^^^^^^^^^

.. code-block:: yaml

   fk_controller:
     ros__parameters:
       # Motion scaling
       velocity_scaling: 0.5
       acceleration_scaling: 0.5

       # Trajectory interpolation
       interpolation_steps: 10      # Points in trajectory

       # Joint limits (radians)
       joint_limits:
         joint1:
           lower: -3.14159
           upper: 3.14159
         joint2:
           lower: -1.5
           upper: 1.5
         joint3:
           lower: -1.5
           upper: 1.4
         joint4:
           lower: -1.7
           upper: 1.97

Gripper Controller
^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   gripper_controller:
     ros__parameters:
       # Gripper limits (meters)
       gripper_closed: -0.010
       gripper_open: 0.019

       # Default effort (0.0-1.0)
       default_effort: 0.5

       # Gripper speed (m/s)
       gripper_speed: 0.02

Named Positions
^^^^^^^^^^^^^^^

.. code-block:: yaml

   named_positions:
     init:
       joint1: 0.0
       joint2: 0.0
       joint3: 0.0
       joint4: 0.0
     home:
       joint1: 0.0
       joint2: -1.0
       joint3: 0.7
       joint4: 0.3

Camera Configuration
--------------------

File: ``src/omx_control/config/c920_camera.yaml``

.. code-block:: yaml

   camera_node:
     ros__parameters:
       # Device settings
       video_device: "/dev/video0"
       framerate: 30.0
       io_method: "mmap"

       # Resolution
       image_width: 1280
       image_height: 720

       # Pixel format: "yuyv" or "mjpeg"
       pixel_format: "yuyv"

       # Color settings (0-255)
       brightness: 128
       contrast: 128
       saturation: 128
       sharpness: 128

       # Auto settings
       autofocus: true
       focus: 0                    # Manual focus (0-255)
       autoexposure: true
       exposure: 166               # Manual exposure
       auto_white_balance: true
       white_balance: 4000

       # Frame settings
       camera_frame_id: "camera_link"
       camera_name: "c920"

       # Calibration file (optional)
       # camera_info_url: "file:///path/to/calibration.yaml"

Resolution Options
^^^^^^^^^^^^^^^^^^

C920 supported resolutions:

.. list-table::
   :header-rows: 1
   :widths: 20 20 30 30

   * - Width
     - Height
     - Max FPS
     - Notes
   * - 1920
     - 1080
     - 30
     - Full HD
   * - 1280
     - 720
     - 30
     - HD (default)
   * - 640
     - 480
     - 30
     - SD

Hardware Controller Configuration
---------------------------------

File: ``open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml``

Controller Manager
^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       update_rate: 100  # Hz

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

       arm_controller:
         type: joint_trajectory_controller/JointTrajectoryController

       gripper_controller:
         type: joint_trajectory_controller/JointTrajectoryController

MoveIt Configuration
--------------------

Directory: ``open_manipulator_moveit_config/config/open_manipulator_x/``

Kinematics
^^^^^^^^^^

File: ``kinematics.yaml``

.. code-block:: yaml

   arm:
     kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
     kinematics_solver_search_resolution: 0.005
     kinematics_solver_timeout: 0.05

Joint Limits
^^^^^^^^^^^^

File: ``joint_limits.yaml``

.. code-block:: yaml

   joint_limits:
     joint1:
       has_velocity_limits: true
       max_velocity: 4.8
       has_acceleration_limits: true
       max_acceleration: 3.0
     # ... similar for other joints

Runtime Configuration
---------------------

Parameters can also be set at runtime:

.. code-block:: bash

   # Set parameter
   ros2 param set /ik_controller velocity_scaling 0.3

   # Get parameter
   ros2 param get /ik_controller velocity_scaling

   # List parameters
   ros2 param list /ik_controller

Environment Variables
---------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Variable
     - Description
   * - ``ROS_DOMAIN_ID``
     - ROS2 domain ID (default: 0)
   * - ``RMW_IMPLEMENTATION``
     - DDS implementation
   * - ``GZ_SIM_RESOURCE_PATH``
     - Gazebo model paths
