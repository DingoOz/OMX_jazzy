# Open Manipulator X - ROS2 Jazzy with MoveIt2

A complete ROS2 Jazzy workspace for controlling the ROBOTIS Open Manipulator X robotic arm with MoveIt2 motion planning, supporting Inverse Kinematics (IK), Forward Kinematics (FK), gripper control, and camera integration.

## Features

- **IK Control**: Move end-effector to Cartesian poses
- **FK Control**: Direct joint angle control
- **Gripper Control**: Open/close and position control
- **MoveIt2 Integration**: Motion planning with collision avoidance
- **Gazebo Simulation**: Full physics simulation with Gazebo Harmonic
- **Camera Integration**: Logitech C920 webcam support with TF

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| Open Manipulator X | ROBOTIS 4-DOF robotic arm |
| U2D2 | USB-to-Dynamixel adapter |
| Logitech C920 | HD webcam (optional) |
| Ubuntu 24.04 | Operating system |

## Quick Start

```bash
# Clone and build
cd ~/Programming/OMX_jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --allow-overriding dynamixel_sdk
source install/setup.bash

# Launch with hardware
ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0

# Or simulation only
ros2 launch omx_control simulation.launch.py
```

---

## Installation

### Step 1: Install ROS2 Jazzy

Follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html) for Ubuntu 24.04.

### Step 2: Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-warehouse-ros-sqlite \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz \
  ros-jazzy-usb-cam \
  ros-jazzy-image-transport \
  ros-jazzy-compressed-image-transport \
  v4l-utils
```

### Step 3: Build the Workspace

```bash
cd ~/Programming/OMX_jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --allow-overriding dynamixel_sdk
source install/setup.bash
```

### Step 4: Set Up Hardware Permissions

**U2D2 (Robot arm):**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Install udev rules
sudo cp 99-u2d2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes
```

**C920 Camera (optional):**
```bash
sudo cp 99-c920.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 5: Add to Shell Configuration (Optional)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/Programming/OMX_jazzy/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### Launch Options

| Command | Description |
|---------|-------------|
| `ros2 launch omx_control bringup_all.launch.py` | Hardware + MoveIt2 + Controllers |
| `ros2 launch omx_control simulation.launch.py` | Gazebo + MoveIt2 + Controllers |
| `ros2 launch omx_control bringup_with_camera.launch.py` | Hardware + Camera |
| `ros2 launch omx_control simulation_with_camera.launch.py` | Simulation + Camera |
| `ros2 launch omx_control full_control.launch.py` | Controllers only |
| `ros2 launch omx_control camera.launch.py` | Camera only |

### Hardware Launch

**Basic launch:**
```bash
ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0
```

**With custom parameters:**
```bash
ros2 launch omx_control bringup_all.launch.py \
  port_name:=/dev/ttyUSB0 \
  velocity_scaling:=0.3 \
  acceleration_scaling:=0.3 \
  start_rviz:=true
```

**Fake hardware mode (no robot):**
```bash
ros2 launch omx_control bringup_all.launch.py use_fake_hardware:=true
```

### Simulation Launch

**Basic Gazebo simulation:**
```bash
ros2 launch omx_control simulation.launch.py
```

**With custom world:**
```bash
ros2 launch omx_control simulation.launch.py world:=empty_world
```

### With Camera

**Hardware + Camera:**
```bash
ros2 launch omx_control bringup_with_camera.launch.py \
  port_name:=/dev/ttyUSB0 \
  video_device:=/dev/video0
```

**Custom camera position:**
```bash
ros2 launch omx_control bringup_with_camera.launch.py \
  port_name:=/dev/ttyUSB0 \
  camera_x:=0.0 \
  camera_y:=-0.3 \
  camera_z:=0.4 \
  camera_pitch:=0.5 \
  camera_yaw:=1.57
```

---

## Control Examples

### IK Control (Inverse Kinematics)

Move end-effector to a target pose in Cartesian space.

**Via topic:**
```bash
# Move to position (x=0.2, y=0.0, z=0.2)
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {
    position: {x: 0.2, y: 0.0, z: 0.2},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Example positions:**
```bash
# Home position (ready for manipulation)
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {
    position: {x: 0.286, y: 0.0, z: 0.204},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

# Extended forward
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {
    position: {x: 0.35, y: 0.0, z: 0.15},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

# Left side
ros2 topic pub --once /omx/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {
    position: {x: 0.2, y: 0.15, z: 0.2},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### FK Control (Forward Kinematics)

Move joints to specific angles (radians).

**Via topic:**
```bash
# Move to specific joint angles
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -0.5, 0.3, 0.2]
}"
```

**Named positions:**
```bash
# Init position (all zeros)
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, 0.0, 0.0, 0.0]
}"

# Home position
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -1.0, 0.7, 0.3]
}"

# Folded position
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState "{
  name: ['joint1', 'joint2', 'joint3', 'joint4'],
  position: [0.0, -1.4, 1.3, 0.1]
}"
```

### Gripper Control

**String commands:**
```bash
# Open gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'open'"

# Close gripper
ros2 topic pub --once /omx/gripper_command std_msgs/msg/String "data: 'close'"
```

**Position control (meters):**
```bash
# Fully open (0.019m)
ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.019"

# Half open
ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: 0.005"

# Fully closed (-0.01m)
ros2 topic pub --once /omx/gripper_position std_msgs/msg/Float64 "data: -0.01"
```

### Monitoring State

```bash
# Watch joint states
ros2 topic echo /joint_states

# Watch arm-only joints
ros2 topic echo /omx/arm_joint_states

# Watch gripper state
ros2 topic echo /omx/gripper_state

# Watch current end-effector pose
ros2 topic echo /omx/current_pose
```

### Camera

```bash
# View camera image
ros2 run rqt_image_view rqt_image_view

# Echo camera info
ros2 topic echo /camera/camera_info

# List camera topics
ros2 topic list | grep camera
```

---

## API Reference

### Topics

#### Arm Control
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/omx/target_pose` | geometry_msgs/PoseStamped | Subscribe | Target end-effector pose (IK) |
| `/omx/target_joints` | sensor_msgs/JointState | Subscribe | Target joint positions (FK) |
| `/omx/current_pose` | geometry_msgs/PoseStamped | Publish | Current end-effector pose |
| `/omx/arm_joint_states` | sensor_msgs/JointState | Publish | Filtered arm joint states |

#### Gripper
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/omx/gripper_command` | std_msgs/String | Subscribe | "open" or "close" |
| `/omx/gripper_position` | std_msgs/Float64 | Subscribe | Position in meters |
| `/omx/gripper_state` | std_msgs/Float64 | Publish | Current position |

#### Camera
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Publish | Raw camera image |
| `/camera/image_raw/compressed` | sensor_msgs/CompressedImage | Publish | JPEG compressed |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Publish | Calibration data |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `omx_control/srv/MoveToPosition` | Custom | IK move with scaling |
| `omx_control/srv/MoveJoints` | Custom | FK move with scaling |
| `omx_control/srv/GripperControl` | Custom | Gripper with effort |

### TF Frames

```
world
└── link1 (base)
    ├── link2
    │   └── link3
    │       └── link4
    │           └── link5
    │               ├── end_effector_link
    │               ├── gripper_left_link
    │               └── gripper_right_link
    └── camera_link (static)
```

### Joint Limits

| Joint | Lower (rad) | Upper (rad) | Description |
|-------|-------------|-------------|-------------|
| joint1 | -3.14159 | 3.14159 | Base rotation |
| joint2 | -1.5 | 1.5 | Shoulder |
| joint3 | -1.5 | 1.4 | Elbow |
| joint4 | -1.7 | 1.97 | Wrist |
| gripper | -0.01 | 0.019 | Gripper (meters) |

---

## Configuration

### Control Parameters

Edit `src/omx_control/config/control_params.yaml`:

```yaml
ik_controller:
  ros__parameters:
    velocity_scaling: 0.5      # 0.0-1.0
    acceleration_scaling: 0.5  # 0.0-1.0
    planning_time: 5.0         # seconds
    num_planning_attempts: 10
```

### Camera Settings

Edit `src/omx_control/config/c920_camera.yaml`:

```yaml
camera_node:
  ros__parameters:
    video_device: "/dev/video0"
    image_width: 1280
    image_height: 720
    framerate: 30.0
    autofocus: true
    autoexposure: true
```

---

## Troubleshooting

### Permission denied on /dev/ttyUSB0

```bash
# Quick fix
sudo chmod 666 /dev/ttyUSB0

# Permanent fix
sudo usermod -aG dialout $USER
# Log out and back in
```

### MoveIt services not available

```bash
# Check if MoveIt is running
ros2 service list | grep compute_ik

# Restart MoveIt
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
```

### Robot not moving

1. Check joint positions are within limits
2. Verify arm is powered on
3. Check U2D2 connection: `ls /dev/ttyUSB*`
4. Check controller status: `ros2 control list_controllers`

### Camera not working

```bash
# List cameras
v4l2-ctl --list-devices

# Test camera
ffplay /dev/video0

# Check ROS topics
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

### Gazebo not starting

```bash
# Check Gazebo installation
gz sim --version

# Install if missing
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
```

---

## Package Structure

```
OMX_jazzy/
├── src/
│   ├── DynamixelSDK/                    # Motor communication SDK
│   ├── dynamixel_interfaces/            # Custom ROS2 messages
│   ├── dynamixel_hardware_interface/    # ros2_control hardware interface
│   ├── open_manipulator/                # ROBOTIS packages
│   │   ├── open_manipulator_bringup/    # Hardware launch files
│   │   ├── open_manipulator_description/# URDF and meshes
│   │   ├── open_manipulator_moveit_config/ # MoveIt2 configuration
│   │   ├── open_manipulator_teleop/     # Keyboard teleoperation
│   │   └── ...
│   └── omx_control/                     # Custom control package
│       ├── omx_control/
│       │   ├── ik_controller.py         # Inverse kinematics node
│       │   ├── fk_controller.py         # Forward kinematics node
│       │   └── gripper_controller.py    # Gripper control node
│       ├── srv/
│       │   ├── MoveToPosition.srv       # IK service
│       │   ├── MoveJoints.srv           # FK service
│       │   └── GripperControl.srv       # Gripper service
│       ├── launch/
│       │   ├── bringup_all.launch.py    # Complete hardware launch
│       │   ├── bringup_with_camera.launch.py
│       │   ├── simulation.launch.py     # Gazebo launch
│       │   ├── simulation_with_camera.launch.py
│       │   ├── full_control.launch.py   # Controllers only
│       │   └── camera.launch.py         # Camera only
│       └── config/
│           ├── control_params.yaml      # Controller settings
│           └── c920_camera.yaml         # Camera settings
├── docs/                                # Sphinx documentation
├── 99-u2d2.rules                        # U2D2 udev rules
├── 99-c920.rules                        # C920 udev rules
└── README.md
```

---

## MoveIt2 and RViz2

### Launch MoveIt2 with RViz2

This starts the `move_group` node and RViz2 with the MoveIt motion planning plugin pre-configured:

```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
```

**With simulation time (when using Gazebo):**
```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true
```

**Without RViz2 (headless move_group only):**
```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py start_rviz:=false
```

### Launch RViz2 Standalone (connect to existing move_group)

If `move_group` is already running (e.g., via `bringup_all.launch.py`), launch just the RViz2 MoveIt interface:

```bash
ros2 launch omx_control rviz_moveit.launch.py
```

### Launch RViz2 Manually

```bash
# Basic RViz2 (no config)
rviz2

# With the MoveIt config
rviz2 -d $(ros2 pkg prefix open_manipulator_moveit_config)/share/open_manipulator_moveit_config/config/moveit.rviz
```

---

## License

Apache-2.0

## References

- [ROBOTIS Open Manipulator X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/)
- [ROBOTIS GitHub](https://github.com/ROBOTIS-GIT/open_manipulator)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

## Launch Commands

```bash
# Terminal 1: Bringup
source install/setup.bash
ros2 launch omx_control bringup_all.launch.py port_name:=/dev/ttyUSB0 start_rviz:=false

# Terminal 2: RViz with MoveIt
source install/setup.bash
ros2 launch omx_control rviz_moveit.launch.py
```
