# Adding a Joint to the Open Manipulator X

This guide walks through every file that must be edited to insert a new
revolute or prismatic joint into the arm. The OMX is described as a chain
of links/joints in xacro, exposed through `ros2_control`, planned by
MoveIt2, and addressed by name from the Python controllers in
`omx_control`. All of those layers must agree on the joint list or the
arm will not start up.

Two helper scripts live in `scripts/`:

- `scripts/add_joint.py` — scaffolds the edits below for a new joint.
- `scripts/verify_joints.sh` — checks that every layer references the
  same joint set.

Run `verify_joints.sh` before and after editing — it is the fastest way
to catch a missed file.

---

## 1. Layers that reference joints

Each new joint must be added to **all** of the following. A mismatch
between any pair will surface as a controller-spawn or MoveIt-planning
failure at launch time.

| Layer                | File                                                                                                       | What to add                                                                                                       |
| -------------------- | ---------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| URDF (kinematics)    | `src/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro` | new `<link>` block, new `<joint>` block with `<origin>`, `<axis>`, `<limit>` and `<dynamics>`                     |
| Mesh                 | `src/open_manipulator/open_manipulator_description/meshes/open_manipulator_x/`                             | a new `.stl` if the link has visual/collision geometry (optional — a primitive `<box>` works for prototyping)     |
| Gazebo               | `src/open_manipulator/open_manipulator_description/gazebo/open_manipulator_x.gazebo.xacro`                 | `<xacro:Link reference="…"/>` for the new link, `<xacro:SimpleTransmission …/>` for the new joint                 |
| ros2_control xacro   | `src/open_manipulator/open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro` | `<joint>` entry with command/state interfaces, a matching `<gpio name="dxlN">` block, and bump `number_of_joints` / `number_of_transmissions` plus extend the two `*_matrix` blocks |
| Bringup controllers  | `src/open_manipulator/open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml` | joint name under `arm_controller.joints` (or a new controller group)                                              |
| Initial position     | `src/open_manipulator/open_manipulator_bringup/config/open_manipulator_x/initial_positions.yaml`           | joint name and home value                                                                                         |
| MoveIt SRDF          | `src/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x/open_manipulator_x.srdf`    | joint inside `<group name="arm">`, plus values inside `init`/`home` group states, plus `<disable_collisions>` rows for adjacent links |
| MoveIt joint limits  | `src/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x/joint_limits.yaml`          | velocity/acceleration limits                                                                                      |
| MoveIt controllers   | `src/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x/moveit_controllers.yaml`    | joint name under `arm_controller.joints`                                                                          |
| omx_control config   | `src/omx_control/config/control_params.yaml`                                                               | `joint_limits.<name>` block and entries in `named_positions.init` / `named_positions.home`                        |
| omx_control Python   | `src/omx_control/omx_control/fk_controller.py`, `ik_controller.py`, `teach_playback.py`                    | extend the `ARM_JOINTS` list and the `JOINT_LIMITS` dict                                                          |
| omx_control srv      | `src/omx_control/srv/MoveJoints.srv`                                                                       | update the comment listing joint order                                                                            |

If you only need a new fixed frame (sensor mount, tool plate, marker),
you can skip everything below the URDF/Gazebo rows — a fixed joint is
not actuated.

---

## 2. Picking parameters

Before touching files, decide:

1. **Name** — e.g. `joint5`, `wrist_roll`. The new joint will be appended
   after the existing four arm joints.
2. **Type** — `revolute` or `prismatic`. Continuous joints are not used
   by MoveIt here.
3. **Parent link** — the link the new joint is attached to. For an
   extra wrist joint that is `link5`.
4. **Child link** — the new link the joint drives. You will create it
   in the URDF.
5. **Origin** — `xyz` and `rpy` of the joint relative to the parent
   link's frame, in metres / radians.
6. **Axis** — unit vector along which the joint rotates or translates,
   in the parent frame.
7. **Limits** — `lower`, `upper` (rad or m), `velocity` (rad/s or m/s),
   `effort` (N·m or N).
8. **Dynamixel ID** — the bus ID the new motor has been flashed with
   (use Dynamixel Wizard). The bringup config uses IDs `11–15` today,
   so a sixth servo would typically be `16`.
9. **Adjacent links** — which existing links the new link may touch in
   its rest pose; collision checks between adjacent pairs are disabled
   in the SRDF.

Write these down — `scripts/add_joint.py` will ask for them and will
patch most of the files for you.

---

## 3. Worked example: adding `joint5`

The example below adds a revolute `joint5` between the current
`link5` and the gripper. It is illustrative; pick values to match your
hardware.

### 3.1 URDF — `open_manipulator_x_arm.urdf.xacro`

Insert a new `<joint>` and `<link>` *before* the gripper joints:

```xml
<joint name="${prefix}joint5" type="revolute">
  <parent link="${prefix}link5"/>
  <child  link="${prefix}link6"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit velocity="4.8" effort="1000" lower="${-pi/2}" upper="${pi/2}"/>
  <dynamics damping="0.1"/>
</joint>

<link name="${prefix}link6">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><box size="0.04 0.04 0.04"/></geometry>
    <material name="grey"><color rgba="0.2 0.2 0.2 1"/></material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><box size="0.04 0.04 0.04"/></geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

Then re-parent the gripper joints from `link5` to `link6`.

### 3.2 Gazebo — `open_manipulator_x.gazebo.xacro`

```xml
<xacro:Link reference="${prefix}link6"/>
<xacro:SimpleTransmission trans="${prefix}trans5"
                          joint="${prefix}joint5"
                          actuator="${prefix}actuator5"/>
```

Renumber the existing gripper transmissions (`trans5/6` → `trans6/7`)
so they stay unique.

### 3.3 ros2_control xacro — `open_manipulator_x_position.ros2_control.xacro`

1. Bump `number_of_joints` and `number_of_transmissions` from `5` to
   `6`.
2. Extend the two `transmission_to_joint_matrix` / `joint_to_transmission_matrix`
   blocks to 6×6 identity.
3. Add the joint block:

   ```xml
   <joint name="${prefix}joint5">
     <command_interface name="position"/>
     <state_interface name="position"/>
     <state_interface name="velocity"/>
     <state_interface name="effort"/>
   </joint>
   ```

4. Add a `<gpio name="dxl6">` block mirroring `dxl4` (same gains,
   Operating Mode 3) with the new servo's bus `ID`.

> The gripper `dxl5` block stays last because it remaps to a prismatic
> joint via `use_revolute_to_prismatic_gripper`.

### 3.4 Bringup controllers — `hardware_controller_manager.yaml`

```yaml
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
```

### 3.5 Initial positions — `initial_positions.yaml`

Add a `0.0` entry for the new joint in the `home` list and add
`joint5` to `joint_names`.

### 3.6 SRDF — `open_manipulator_x.srdf`

```xml
<group name="arm">
  …existing joints…
  <joint name="joint5"/>
</group>

<group_state name="init" group="arm">
  …
  <joint name="joint5" value="0"/>
</group_state>
<group_state name="home" group="arm">
  …
  <joint name="joint5" value="0"/>
</group_state>

<disable_collisions link1="link5" link2="link6" reason="Adjacent"/>
<disable_collisions link1="link6" link2="gripper_left_link"  reason="Adjacent"/>
<disable_collisions link1="link6" link2="gripper_right_link" reason="Adjacent"/>
```

Re-run the MoveIt Setup Assistant later for a complete collision
matrix, or extend the rules by hand for adjacent pairs only.

### 3.7 MoveIt joint limits — `joint_limits.yaml`

```yaml
joint5:
  has_velocity_limits: true
  max_velocity: 5.0
  has_acceleration_limits: true
  max_acceleration: 5.0
```

### 3.8 MoveIt controllers — `moveit_controllers.yaml`

```yaml
arm_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
```

### 3.9 omx_control — `control_params.yaml`

```yaml
fk_controller:
  ros__parameters:
    joint_limits:
      …
      joint5:
        lower: -1.5708
        upper: 1.5708

/**:
  ros__parameters:
    named_positions:
      init:
        joint5: 0.0
      home:
        joint5: 0.0
```

### 3.10 omx_control Python

In `fk_controller.py`, `ik_controller.py`, and `teach_playback.py`:

```python
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

JOINT_LIMITS = {
    …,
    'joint5': (-1.5708, 1.5708),
}
```

Update the named-position table in `fk_controller.move_to_named_position`
similarly.

### 3.11 srv comment — `MoveJoints.srv`

Update the comment so callers know the new joint order, e.g.
`# [joint1, joint2, joint3, joint4, joint5]`.

---

## 4. Verification

```bash
# 1. Confirm joint sets agree across layers
./scripts/verify_joints.sh

# 2. Rebuild
colcon build --symlink-install --allow-overriding dynamixel_sdk
source install/setup.bash

# 3. Visual sanity check on the URDF in RViz
ros2 launch omx_control bringup_all.launch.py use_fake_hardware:=true

# 4. Plan with MoveIt to a named state
ros2 topic pub --once /omx/target_joints sensor_msgs/msg/JointState \
  "{name: ['joint1','joint2','joint3','joint4','joint5'],
    position: [0.0, -1.0, 0.7, 0.3, 0.0]}"
```

If MoveIt complains that a joint is not in the planning group, the SRDF
is the layer to recheck. If `ros2 control list_controllers` shows
`arm_controller` as `inactive`, the controllers YAML or ros2_control
xacro joint list is the layer to recheck.

---

## 5. Removing a joint

Reverse the same edits: drop the joint from every list above, decrement
`number_of_joints` and shrink the transmission matrices, and re-parent
whatever the deleted link's child was. `scripts/verify_joints.sh`
catches a half-finished removal the same way it catches a
half-finished addition.
