Control Examples
================

This page provides detailed examples for controlling the Open Manipulator X.

IK Control (Inverse Kinematics)
-------------------------------

IK control moves the end-effector to a specified Cartesian pose. The system computes
the required joint angles automatically.

Basic IK Command
^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.2, y: 0.0, z: 0.2},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

Workspace Positions
^^^^^^^^^^^^^^^^^^^

**Center position:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.286, y: 0.0, z: 0.204},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

**Extended forward:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.35, y: 0.0, z: 0.15},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

**Left side:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.2, y: 0.15, z: 0.2},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

**Right side:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.2, y: -0.15, z: 0.2},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

**Low position:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.25, y: 0.0, z: 0.05},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

FK Control (Forward Kinematics)
-------------------------------

FK control moves joints directly to specified angles (in radians).

Basic FK Command
^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -0.5, 0.3, 0.2]
   }"

Named Positions
^^^^^^^^^^^^^^^

**Init position (all zeros):**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, 0.0, 0.0, 0.0]
   }"

**Home position:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -1.0, 0.7, 0.3]
   }"

**Folded position:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -1.4, 1.3, 0.1]
   }"

**Looking left:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [1.57, -1.0, 0.7, 0.3]
   }"

**Looking right:**

.. code-block:: bash

   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [-1.57, -1.0, 0.7, 0.3]
   }"

Gripper Control
---------------

String Commands
^^^^^^^^^^^^^^^

.. code-block:: bash

   # Open gripper
   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

   # Close gripper
   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"

Position Control
^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Fully open (0.019m)
   ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.019"

   # Three quarters open
   ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.012"

   # Half open
   ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.005"

   # Quarter open
   ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.0"

   # Fully closed (-0.01m)
   ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: -0.01"

Monitoring
----------

Joint States
^^^^^^^^^^^^

.. code-block:: bash

   # All joint states
   ros2 topic echo /joint_states

   # Arm joints only
   ros2 topic echo /omx/arm_joint_states

   # Single reading
   ros2 topic echo /joint_states --once

Gripper State
^^^^^^^^^^^^^

.. code-block:: bash

   ros2 topic echo /omx/gripper_state

End-Effector Pose
^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 topic echo /omx/current_pose

Topic Frequency
^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 topic hz /joint_states

Pick and Place Example
----------------------

A simple pick and place sequence:

.. code-block:: bash

   # Move to home
   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -1.0, 0.7, 0.3]
   }"

   # Open gripper
   sleep 2
   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

   # Move to pick position
   sleep 1
   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.25, y: 0.0, z: 0.05},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

   # Close gripper (pick object)
   sleep 2
   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"

   # Lift
   sleep 1
   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.25, y: 0.0, z: 0.2},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

   # Move to place position
   sleep 2
   ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'link1'},
     pose: {
       position: {x: 0.2, y: 0.15, z: 0.05},
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
     }
   }"

   # Open gripper (release object)
   sleep 2
   ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

   # Return home
   sleep 1
   ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
     name: ['joint1', 'joint2', 'joint3', 'joint4'],
     position: [0.0, -1.0, 0.7, 0.3]
   }"
