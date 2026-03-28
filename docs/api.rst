API Reference
=============

This page documents all topics, services, and parameters.

Topics
------

Arm Control Topics
^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 25 10 35

   * - Topic
     - Type
     - Dir
     - Description
   * - ``/omx/target_pose``
     - geometry_msgs/PoseStamped
     - Sub
     - Target end-effector pose for IK control
   * - ``/omx/target_joints``
     - sensor_msgs/JointState
     - Sub
     - Target joint positions for FK control
   * - ``/omx/current_pose``
     - geometry_msgs/PoseStamped
     - Pub
     - Current end-effector pose
   * - ``/omx/arm_joint_states``
     - sensor_msgs/JointState
     - Pub
     - Filtered arm joint states (4 joints)
   * - ``/joint_states``
     - sensor_msgs/JointState
     - Pub
     - All joint states (arm + gripper)

Gripper Topics
^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 25 10 35

   * - Topic
     - Type
     - Dir
     - Description
   * - ``/omx/gripper_command``
     - std_msgs/String
     - Sub
     - Command: "open" or "close"
   * - ``/omx/gripper_position``
     - std_msgs/Float64
     - Sub
     - Direct position in meters
   * - ``/omx/gripper_state``
     - std_msgs/Float64
     - Pub
     - Current gripper position

Camera Topics
^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 35 30 10 25

   * - Topic
     - Type
     - Dir
     - Description
   * - ``/camera/image_raw``
     - sensor_msgs/Image
     - Pub
     - Raw camera image
   * - ``/camera/image_raw/compressed``
     - sensor_msgs/CompressedImage
     - Pub
     - JPEG compressed image
   * - ``/camera/camera_info``
     - sensor_msgs/CameraInfo
     - Pub
     - Camera calibration

Services
--------

MoveToPosition
^^^^^^^^^^^^^^

Move end-effector to a target pose using IK.

**Service:** ``omx_control/srv/MoveToPosition``

**Request:**

.. code-block:: text

   geometry_msgs/Pose target_pose     # Target pose
   float64 velocity_scaling 0.5       # Velocity scale (0.0-1.0)
   float64 acceleration_scaling 0.5   # Acceleration scale (0.0-1.0)
   bool wait_for_completion true      # Wait for motion

**Response:**

.. code-block:: text

   bool success                       # True if successful
   string message                     # Status message
   float64[] final_joint_positions    # Final joint positions

MoveJoints
^^^^^^^^^^

Move joints to target positions using FK.

**Service:** ``omx_control/srv/MoveJoints``

**Request:**

.. code-block:: text

   float64[] joint_positions          # Target positions (radians)
   float64 velocity_scaling 0.5       # Velocity scale
   float64 acceleration_scaling 0.5   # Acceleration scale
   bool wait_for_completion true      # Wait for motion

**Response:**

.. code-block:: text

   bool success                       # True if successful
   string message                     # Status message
   geometry_msgs/Pose final_pose      # Final end-effector pose

GripperControl
^^^^^^^^^^^^^^

Control the gripper.

**Service:** ``omx_control/srv/GripperControl``

**Request:**

.. code-block:: text

   string command                     # "open", "close", or "position"
   float64 position 0.0               # Position for "position" command
   float64 effort 0.5                 # Gripper effort (0.0-1.0)
   bool wait_for_completion true      # Wait for action

**Response:**

.. code-block:: text

   bool success                       # True if successful
   string message                     # Status message
   float64 current_position           # Current gripper position

TF Frames
---------

Frame Hierarchy
^^^^^^^^^^^^^^^

.. code-block:: text

   world
   └── link1 (base)
       ├── link2
       │   └── link3
       │       └── link4
       │           └── link5
       │               ├── end_effector_link
       │               ├── gripper_left_link
       │               └── gripper_right_link
       └── camera_link (static transform)

Key Frames
^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Frame
     - Description
   * - ``link1``
     - Robot base frame
   * - ``end_effector_link``
     - End-effector (tool) frame
   * - ``gripper_left_link``
     - Left gripper finger
   * - ``camera_link``
     - Camera optical frame

Joint Information
-----------------

Joint Limits
^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 15 15 15 15 40

   * - Joint
     - Lower
     - Upper
     - Type
     - Description
   * - joint1
     - -3.14159
     - 3.14159
     - revolute
     - Base rotation
   * - joint2
     - -1.5
     - 1.5
     - revolute
     - Shoulder
   * - joint3
     - -1.5
     - 1.4
     - revolute
     - Elbow
   * - joint4
     - -1.7
     - 1.97
     - revolute
     - Wrist
   * - gripper_left
     - -0.01
     - 0.019
     - prismatic
     - Gripper (m)

Joint Order
^^^^^^^^^^^

When sending joint commands, use this order:

.. code-block:: python

   joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

MoveIt2 Groups
--------------

Planning Groups
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Group
     - Joints
     - Description
   * - ``arm``
     - joint1-4
     - Main arm planning group
   * - ``gripper``
     - gripper_left, gripper_right
     - Gripper group

Named States
^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 15 35 50

   * - Name
     - Joint Values
     - Description
   * - ``init``
     - [0, 0, 0, 0]
     - All zeros
   * - ``home``
     - [0, -1.0, 0.7, 0.3]
     - Ready position

Controller Information
----------------------

Available Controllers
^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Controller
     - Description
   * - ``arm_controller``
     - Joint trajectory controller for arm
   * - ``gripper_controller``
     - Joint trajectory controller for gripper
   * - ``joint_state_broadcaster``
     - Publishes joint states

Check controller status:

.. code-block:: bash

   ros2 control list_controllers
