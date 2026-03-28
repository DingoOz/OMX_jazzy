# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Jazzy workspace for controlling ROBOTIS Open Manipulator X robotic arm with MoveIt2 motion planning. The system provides IK/FK control, gripper management, Gazebo simulation, and camera integration.

## Build Commands

```bash
# Build workspace (always use these flags for this repo)
colcon build --symlink-install --allow-overriding dynamixel_sdk

# Source workspace
source install/setup.bash

# Build single package
colcon build --packages-select <package_name> --symlink-install

# Run tests
colcon test
colcon test-result --verbose
```

## Launch Commands

```bash
# Hardware with MoveIt2 + controllers
ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0

# Gazebo simulation
ros2 launch omx_control simulation.launch.py

# Fake hardware mode (no physical robot)
ros2 launch omx_control bringup_all.launch.py use_fake_hardware:=true

# Hardware with camera
ros2 launch omx_control bringup_with_camera.launch.py port_name:=/dev/ttyUSB0
```

## Architecture

```
User Commands (Topics/Services)
        ↓
omx_control (IK/FK/Gripper Controllers) - Python nodes
        ↓
MoveIt2 Planning (/compute_ik, /compute_fk)
        ↓
ros2_controllers (arm_controller, gripper_controller)
        ↓
ros2_control Framework
        ↓
dynamixel_hardware_interface (C++ plugin)
        ↓
DynamixelSDK → Hardware
```

### Key Packages

- **omx_control**: High-level Python controllers (`src/omx_control/omx_control/`)
  - `ik_controller.py`: Cartesian pose control via `/omx/target_pose`
  - `fk_controller.py`: Joint angle control via `/omx/target_joints`
  - `gripper_controller.py`: Gripper via `/omx/gripper_command`

- **dynamixel_hardware_interface**: C++ ros2_control hardware plugin (`src/dynamixel_hardware_interface/`)

- **open_manipulator_moveit_config**: MoveIt2 configuration (`src/open_manipulator/open_manipulator_moveit_config/`)

- **open_manipulator_description**: URDF/Xacro models (`src/open_manipulator/open_manipulator_description/`)

## Control Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/omx/target_pose` | geometry_msgs/PoseStamped | IK target (Cartesian) |
| `/omx/target_joints` | sensor_msgs/JointState | FK target (joint angles) |
| `/omx/gripper_command` | std_msgs/String | "open" or "close" |
| `/omx/gripper_position` | std_msgs/Float64 | Gripper position (meters) |
| `/omx/current_pose` | geometry_msgs/PoseStamped | Current end-effector pose |

## Joint Limits (radians)

- joint1: [-π, π] (base rotation)
- joint2: [-1.5, 1.5] (shoulder)
- joint3: [-1.5, 1.4] (elbow)
- joint4: [-1.7, 1.97] (wrist)
- gripper: [-0.01, 0.019] meters

## Configuration Files

- `src/omx_control/config/control_params.yaml`: Controller parameters (velocity/acceleration scaling)
- `src/omx_control/config/c920_camera.yaml`: Camera settings
- `src/open_manipulator/open_manipulator_moveit_config/config/`: MoveIt2 configs (kinematics, planners, controllers)

## Testing Control

```bash
# IK: Move to Cartesian pose
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'link1'}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# FK: Move to joint angles
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{name: ['joint1', 'joint2', 'joint3', 'joint4'], position: [0.0, -1.0, 0.7, 0.3]}"

# Gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"
```

## Debugging

```bash
# Check controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check MoveIt services
ros2 service list | grep compute_ik

# Check hardware connection
ls /dev/ttyUSB*
```
