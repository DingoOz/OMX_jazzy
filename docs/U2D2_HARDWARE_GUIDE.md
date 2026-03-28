# Open Manipulator X Hardware Guide (U2D2)

This guide covers setting up and controlling the Open Manipulator X robotic arm using the U2D2 USB-to-Dynamixel adapter.

## Hardware Overview

| Component | Description |
|-----------|-------------|
| Open Manipulator X | ROBOTIS 4-DOF robotic arm with gripper |
| U2D2 | USB-to-Dynamixel communication adapter |
| 12V Power Supply | Powers the Dynamixel motors (not USB-powered) |

**Connection diagram:**
```
PC (USB) → U2D2 → Dynamixel Chain → Open Manipulator X Motors
                                  ↑
                            12V Power Supply
```

## 1. Hardware Setup

### 1.1 Connect the U2D2

1. Connect the U2D2 to your PC via USB
2. Connect the Dynamixel cable from the U2D2 to the first motor on the arm
3. Connect the 12V power supply to the arm (motors require external power)
4. Power on the arm

### 1.2 Verify USB Connection

```bash
# Check if U2D2 is detected
ls /dev/ttyUSB*
```

You should see `/dev/ttyUSB0` (or similar). If not, check USB connection.

### 1.3 Set Up Permissions

**Option A: Quick fix (temporary)**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Option B: Permanent fix (recommended)**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Install udev rules
sudo cp 99-u2d2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes to take effect
```

### 1.4 Verify Permissions

```bash
# Should show your user in the dialout group
groups $USER

# Should show rw permissions
ls -la /dev/ttyUSB0
```

## 2. Building the Workspace

```bash
# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Navigate to workspace
cd ~/Programming/OMX_jazzy

# Build (use these exact flags)
colcon build --symlink-install --allow-overriding dynamixel_sdk

# Source the workspace
source install/setup.bash
```

## 3. Launching the System

### 3.1 Basic Launch (Hardware + MoveIt2 + Controllers)

```bash
ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0
```

This launches:
- Dynamixel hardware interface
- MoveIt2 motion planning
- IK, FK, and Gripper controllers
- RViz visualization

### 3.2 Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port_name` | `/dev/ttyUSB0` | Serial port for U2D2 |
| `velocity_scaling` | `0.5` | Motion speed (0.0-1.0) |
| `acceleration_scaling` | `0.5` | Acceleration (0.0-1.0) |
| `start_rviz` | `true` | Launch RViz |
| `use_fake_hardware` | `false` | Simulate without hardware |

**Example with slower motion:**
```bash
ros2 launch omx_control bringup_all.launch.py \
  port_name:=/dev/ttyUSB0 \
  velocity_scaling:=0.3 \
  acceleration_scaling:=0.3
```

### 3.3 Verify System is Running

In a new terminal (with workspace sourced):

```bash
# Check controllers are active
ros2 control list_controllers
```

Expected output:
```
arm_controller[joint_trajectory_controller/JointTrajectoryController] active
gripper_controller[position_controllers/GripperActionController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

```bash
# Check joint states are publishing
ros2 topic hz /joint_states
```

Should show ~50 Hz update rate.

## 4. Controlling the Arm

### 4.1 IK Control (Cartesian Space)

Move the end-effector to a target position in 3D space. The system computes joint angles automatically.

```bash
# Move to position (x=0.2m, y=0, z=0.2m)
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {
    position: {x: 0.2, y: 0.0, z: 0.2},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Common positions:**

| Position | x | y | z | Description |
|----------|---|---|---|-------------|
| Home | 0.286 | 0.0 | 0.204 | Ready position |
| Forward | 0.35 | 0.0 | 0.15 | Extended forward |
| Left | 0.2 | 0.15 | 0.2 | Reaching left |
| Right | 0.2 | -0.15 | 0.2 | Reaching right |
| High | 0.2 | 0.0 | 0.35 | Elevated |

**Workspace limits (approximate):**
- X: 0.1 to 0.38 meters (forward reach)
- Y: -0.3 to 0.3 meters (side reach)
- Z: 0.0 to 0.4 meters (height)

### 4.2 FK Control (Joint Space)

Move individual joints to specific angles (radians).

```bash
# Move to joint angles
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -1.0, 0.7, 0.3]
}"
```

**Predefined positions:**

| Name | joint1 | joint2 | joint3 | joint4 | Description |
|------|--------|--------|--------|--------|-------------|
| Init | 0.0 | 0.0 | 0.0 | 0.0 | All zeros (straight up) |
| Home | 0.0 | -1.0 | 0.7 | 0.3 | Ready position |
| Folded | 0.0 | -1.4 | 1.3 | 0.1 | Compact position |

**Joint limits:**

| Joint | Min (rad) | Max (rad) | Description |
|-------|-----------|-----------|-------------|
| joint1 | -3.14 | 3.14 | Base rotation |
| joint2 | -1.5 | 1.5 | Shoulder |
| joint3 | -1.5 | 1.4 | Elbow |
| joint4 | -1.7 | 1.97 | Wrist |

### 4.3 Gripper Control

**String commands (simple):**
```bash
# Open gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

# Close gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"
```

**Position control (precise):**
```bash
# Fully open (0.019m)
ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.019"

# Half open
ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.005"

# Fully closed (-0.01m)
ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: -0.01"
```

## 5. Monitoring

### 5.1 Joint States

```bash
# All joints (arm + gripper)
ros2 topic echo /joint_states

# Arm joints only
ros2 topic echo /omx/arm_joint_states
```

### 5.2 End-Effector Pose

```bash
ros2 topic echo /omx/current_pose
```

### 5.3 Gripper State

```bash
ros2 topic echo /omx/gripper_state
```

## 6. Example Workflow

Here's a complete pick-and-place example:

```bash
# 1. Move to home position
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -1.0, 0.7, 0.3]
}"

# 2. Open gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

# 3. Move above pick location
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {position: {x: 0.25, y: 0.1, z: 0.15}, orientation: {w: 1.0}}
}"

# 4. Move down to pick
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {position: {x: 0.25, y: 0.1, z: 0.05}, orientation: {w: 1.0}}
}"

# 5. Close gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"

# 6. Lift object
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {position: {x: 0.25, y: 0.1, z: 0.2}, orientation: {w: 1.0}}
}"

# 7. Move to place location
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {position: {x: 0.25, y: -0.1, z: 0.2}, orientation: {w: 1.0}}
}"

# 8. Lower and release
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {position: {x: 0.25, y: -0.1, z: 0.05}, orientation: {w: 1.0}}
}"
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

# 9. Return home
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -1.0, 0.7, 0.3]
}"
```

## 7. Troubleshooting

### U2D2 Not Detected

```bash
# Check USB devices
lsusb | grep -i ftdi

# Check dmesg for connection
dmesg | tail -20
```

If not detected:
- Try a different USB port
- Try a different USB cable
- Check if U2D2 LED is lit when connected

### Permission Denied

```bash
# Check current permissions
ls -la /dev/ttyUSB0

# Quick fix
sudo chmod 666 /dev/ttyUSB0

# Verify group membership
groups $USER | grep dialout
```

### Motors Not Responding

1. **Check power**: Motors need 12V external power, not USB
2. **Check baud rate**: Default is 1000000 bps
3. **Check motor IDs**: Should be 11, 12, 13, 14 for arm; 15 for gripper

```bash
# Scan for motors (requires dynamixel_sdk tools)
ros2 run dynamixel_sdk_examples read_write_node
```

### IK Solution Not Found

The target pose may be outside the arm's workspace. Try:
- Moving the target closer to the base
- Checking joint limits aren't exceeded
- Using FK to move to a known-good position first

```bash
# Check if MoveIt IK service is available
ros2 service list | grep compute_ik
```

### Arm Moves Erratically

Reduce velocity and acceleration scaling:
```bash
ros2 launch omx_control bringup_all.launch.py \
  port_name:=/dev/ttyUSB0 \
  velocity_scaling:=0.2 \
  acceleration_scaling:=0.2
```

### Controllers Not Starting

```bash
# Check controller manager
ros2 control list_controllers

# Check hardware interface
ros2 control list_hardware_interfaces
```

If hardware interfaces show as "unavailable", the U2D2 connection has failed.

## 8. Safety Notes

1. **Start slow**: Use `velocity_scaling:=0.2` when first testing
2. **Clear workspace**: Ensure no obstacles in arm's reach
3. **Emergency stop**: Kill the launch process (Ctrl+C) to stop motion
4. **Power off**: Disconnect 12V power to fully disable motors
5. **Joint limits**: The system enforces limits, but avoid commanding extreme positions

## 9. Additional Resources

- [ROBOTIS Open Manipulator X e-Manual](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/)
- [ROBOTIS U2D2 e-Manual](https://emanual.robotis.com/docs/en/parts/interface/u2d2/)
- [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) - For motor configuration and testing
