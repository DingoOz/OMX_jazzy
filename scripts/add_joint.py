#!/usr/bin/env python3
"""
Scaffolder for inserting a new joint into the Open Manipulator X stack.

This patches every layer that references the joint list:

  - URDF arm xacro (new <link> and <joint> appended off `--parent`)
  - Gazebo xacro (<xacro:Link> and <xacro:SimpleTransmission>)
  - ros2_control xacro (<joint> + <gpio>, plus number_of_joints,
    number_of_transmissions, and the two identity matrices)
  - bringup hardware_controller_manager.yaml (arm_controller.joints)
  - bringup initial_positions.yaml (joint_names + home list)
  - MoveIt SRDF (<group name="arm">, init/home group_states)
  - MoveIt joint_limits.yaml
  - MoveIt moveit_controllers.yaml (arm_controller.joints)
  - omx_control control_params.yaml (joint_limits and named_positions)
  - omx_control python (ARM_JOINTS, JOINT_LIMITS in fk/ik/teach_playback)
  - omx_control srv MoveJoints.srv (comment update)

Dry-run by default; pass --apply to actually write the files.

This script always appends the new joint as a leaf off `--parent`. If you
want to splice it in between existing links (e.g. between link5 and the
gripper) you must additionally re-parent the gripper joints in the URDF
by hand — see docs/adding_a_joint.md.

Run scripts/verify_joints.sh after applying to confirm every layer
agrees.
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


REPO = Path(__file__).resolve().parent.parent
DESC = REPO / 'src/open_manipulator/open_manipulator_description'
BRINGUP = REPO / 'src/open_manipulator/open_manipulator_bringup'
MOVEIT = REPO / 'src/open_manipulator/open_manipulator_moveit_config'
OMX = REPO / 'src/omx_control'


@dataclass
class JointSpec:
    name: str
    parent: str
    child: str
    jtype: str            # "revolute" or "prismatic"
    axis: tuple[float, float, float]
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    lower: float
    upper: float
    velocity: float
    effort: float
    dxl_id: int
    home: float

    def axis_str(self) -> str:
        return ' '.join(str(v) for v in self.axis)

    def xyz_str(self) -> str:
        return ' '.join(str(v) for v in self.origin_xyz)

    def rpy_str(self) -> str:
        return ' '.join(str(v) for v in self.origin_rpy)


# ----------------------------------------------------------------------------
# Patch primitives
# ----------------------------------------------------------------------------


class FilePatch:
    """Tracks original vs. patched content of one file."""

    def __init__(self, path: Path):
        self.path = path
        self.original = path.read_text() if path.exists() else ''
        self.content = self.original

    def replace(self, old: str, new: str, count: int = 1) -> None:
        if old not in self.content:
            raise ValueError(f'{self.path}: anchor not found:\n{old}')
        self.content = self.content.replace(old, new, count)

    def sub(self, pattern: str, repl: str, flags: int = 0, count: int = 1) -> None:
        new, n = re.subn(pattern, repl, self.content, count=count, flags=flags)
        if n == 0:
            raise ValueError(f'{self.path}: regex {pattern!r} matched nothing')
        self.content = new

    @property
    def changed(self) -> bool:
        return self.content != self.original

    def write(self) -> None:
        backup = self.path.with_suffix(self.path.suffix + '.bak')
        shutil.copy2(self.path, backup)
        self.path.write_text(self.content)


def fail_if_joint_present(p: FilePatch, joint: str, anchor: str) -> bool:
    """Return True if joint is already in file; caller should skip."""
    if re.search(rf'\b{re.escape(joint)}\b', p.content):
        print(f'  [skip] {p.path.relative_to(REPO)}: {joint!r} already present')
        return True
    return False


# ----------------------------------------------------------------------------
# Per-file patches
# ----------------------------------------------------------------------------


def patch_urdf_arm(spec: JointSpec) -> FilePatch:
    p = FilePatch(DESC / 'urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    block = f'''
    <joint name="${{prefix}}{spec.name}" type="{spec.jtype}">
      <parent link="${{prefix}}{spec.parent}"/>
      <child link="${{prefix}}{spec.child}"/>
      <origin xyz="{spec.xyz_str()}" rpy="{spec.rpy_str()}"/>
      <axis xyz="{spec.axis_str()}"/>
      <limit velocity="{spec.velocity}" effort="{spec.effort}" lower="{spec.lower}" upper="{spec.upper}"/>
      <dynamics damping="0.1"/>
    </joint>

    <link name="${{prefix}}{spec.child}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.04 0.04 0.04"/>
        </geometry>
        <material name="grey">
          <color rgba="0.2 0.2 0.2 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.04 0.04 0.04"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0"/>
        <mass value="0.05"/>
        <inertia ixx="1.0e-5" ixy="0.0" ixz="0.0"
                 iyy="1.0e-5" iyz="0.0"
                 izz="1.0e-5"/>
      </inertial>
    </link>

'''
    p.replace('  </xacro:macro>', block + '  </xacro:macro>')
    return p


def patch_gazebo(spec: JointSpec) -> FilePatch:
    p = FilePatch(DESC / 'gazebo/open_manipulator_x.gazebo.xacro')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # Find highest existing trans number.
    nums = [int(m.group(1)) for m in re.finditer(r'trans="\$\{prefix\}trans(\d+)"', p.content)]
    next_n = max(nums) + 1 if nums else 1

    link_line = f'  <xacro:Link reference="${{prefix}}{spec.child}"/>\n'
    trans_line = (
        f'  <xacro:SimpleTransmission trans="${{prefix}}trans{next_n}" '
        f'joint="${{prefix}}{spec.name}" actuator="${{prefix}}actuator{next_n}"/>\n'
    )

    # Anchor on the final <xacro:SimpleTransmission> line so we append inside
    # the outer macro (the file has three </xacro:macro> tags — only the last
    # is the outer one).
    last_trans = list(re.finditer(
        r'  <xacro:SimpleTransmission trans="\$\{prefix\}trans\d+"[^>]*?/>\n',
        p.content,
    ))
    if not last_trans:
        raise ValueError(f'{p.path}: could not find a SimpleTransmission anchor')
    end = last_trans[-1].end()
    p.content = p.content[:end] + link_line + trans_line + p.content[end:]
    return p


def patch_ros2_control(spec: JointSpec) -> FilePatch:
    p = FilePatch(DESC / 'ros2_control/open_manipulator_x_position.ros2_control.xacro')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # 1. Bump number_of_joints and number_of_transmissions.
    def bump(match: re.Match) -> str:
        name, val = match.group(1), int(match.group(2))
        return f'<param name="{name}">{val + 1}</param>'

    p.content = re.sub(
        r'<param name="(number_of_joints|number_of_transmissions)">(\d+)</param>',
        bump,
        p.content,
    )

    # 2. Expand the two identity matrices.
    matrix_re = re.compile(
        r'(<param name="(?:transmission_to_joint_matrix|joint_to_transmission_matrix)">)\s*'
        r'([0-9,\s]+?)\s*(</param>)',
        re.MULTILINE,
    )

    def expand_matrix(match: re.Match) -> str:
        head, body, tail = match.group(1), match.group(2), match.group(3)
        rows = [r.strip() for r in body.strip().split('\n') if r.strip()]
        n = len(rows)
        # New (n+1)x(n+1) identity.
        new_rows = []
        for i in range(n + 1):
            row = ['1' if i == j else '0' for j in range(n + 1)]
            new_rows.append('              ' + ', '.join(row))
        return head + '\n' + ',\n'.join(new_rows) + '\n            ' + tail

    p.content = matrix_re.sub(expand_matrix, p.content)

    # 3. Insert new <joint> block before the gripper_left_joint so arm joints
    #    stay grouped together. (gripper_left_joint is the first non-arm joint.)
    joint_block = (
        f'      <joint name="${{prefix}}{spec.name}">\n'
        f'        <command_interface name="position"/>\n'
        f'        <state_interface name="position"/>\n'
        f'        <state_interface name="velocity"/>\n'
        f'        <state_interface name="effort"/>\n'
        f'      </joint>\n'
    )
    anchor = '      <joint name="${prefix}gripper_left_joint">'
    p.replace(anchor, joint_block + anchor)

    # 4. Append new <gpio name="dxlN"> block after the last existing one.
    gpio_nums = [int(m.group(1)) for m in re.finditer(r'<gpio name="dxl(\d+)">', p.content)]
    next_dxl = max(gpio_nums) + 1 if gpio_nums else 1
    gpio_block = f'''      <gpio name="dxl{next_dxl}">
        <param name="type">dxl</param>
        <param name="ID">{spec.dxl_id}</param>
        <command_interface name="Goal Position"/>
        <state_interface name="Present Position"/>
        <state_interface name="Present Velocity"/>
        <state_interface name="Present Current"/>
        <param name="Operating Mode">3</param>
        <param name="Position P Gain">800</param>
        <param name="Position I Gain">100</param>
        <param name="Position D Gain">100</param>
        <param name="Profile Velocity">20</param>
        <param name="Profile Acceleration">10</param>
        <param name="Drive Mode">4</param>
        <param name="Return Delay Time">0</param>
      </gpio>

'''
    p.replace('    </ros2_control>', gpio_block + '    </ros2_control>')
    return p


def patch_hardware_controller_manager(spec: JointSpec) -> FilePatch:
    p = FilePatch(BRINGUP / 'config/open_manipulator_x/hardware_controller_manager.yaml')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # arm_controller joints list.
    p.sub(
        r'(arm_controller:\s*\n\s*ros__parameters:\s*\n\s*joints:\s*\n(?:\s*-\s*\S+\n)+)',
        lambda m: m.group(1) + f'        - {spec.name}\n',
        flags=re.MULTILINE,
    )
    return p


def patch_initial_positions(spec: JointSpec) -> FilePatch:
    p = FilePatch(BRINGUP / 'config/open_manipulator_x/initial_positions.yaml')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # joint_names list.
    p.sub(
        r'(joint_names:\s*\n(?:\s*-\s*\S+\n)+)',
        lambda m: m.group(1) + f'      - {spec.name}\n',
        flags=re.MULTILINE,
    )
    # home values: append `, <home>` before the closing ].
    p.sub(
        r'(home:\s*\[[^\]]+)\]',
        lambda m: f'{m.group(1)}, {spec.home}]',
    )
    return p


def patch_srdf(spec: JointSpec) -> FilePatch:
    p = FilePatch(MOVEIT / 'config/open_manipulator_x/open_manipulator_x.srdf')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # Insert joint into <group name="arm"> before end_effector_joint if present, else before </group>.
    arm_pattern = re.compile(r'(<group name="arm">.*?)(\s*<joint name="end_effector_joint"/>)?\s*(</group>)',
                              re.DOTALL)

    def add_to_arm_group(m: re.Match) -> str:
        body, ee, end = m.group(1), m.group(2) or '', m.group(3)
        insertion = f'\n        <joint name="{spec.name}"/>'
        return body + insertion + ee + '\n    ' + end

    new, n = arm_pattern.subn(add_to_arm_group, p.content, count=1)
    if n == 0:
        raise ValueError(f'{p.path}: could not locate <group name="arm">')
    p.content = new

    # Add to init and home group states.
    for state in ('init', 'home'):
        state_re = re.compile(
            rf'(<group_state name="{state}" group="arm">.*?)(\s*</group_state>)',
            re.DOTALL,
        )
        p.content = state_re.sub(
            lambda m: m.group(1) + f'\n        <joint name="{spec.name}" value="{spec.home}"/>' + m.group(2),
            p.content,
            count=1,
        )

    # Add adjacent collision disable.
    p.replace(
        '</robot>',
        f'    <disable_collisions link1="{spec.parent}" link2="{spec.child}" reason="Adjacent"/>\n</robot>',
    )
    return p


def patch_moveit_joint_limits(spec: JointSpec) -> FilePatch:
    p = FilePatch(MOVEIT / 'config/open_manipulator_x/joint_limits.yaml')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    block = (
        f'  {spec.name}:\n'
        f'    has_velocity_limits: true\n'
        f'    max_velocity: 5.0\n'
        f'    has_acceleration_limits: true\n'
        f'    max_acceleration: 5.0\n'
    )
    p.content = p.content.rstrip() + '\n' + block
    return p


def patch_moveit_controllers(spec: JointSpec) -> FilePatch:
    p = FilePatch(MOVEIT / 'config/open_manipulator_x/moveit_controllers.yaml')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    p.sub(
        r'(arm_controller:\s*\n\s*type:.*?\n\s*action_ns:.*?\n\s*joints:\s*\n(?:\s*-\s*\S+\n)+)',
        lambda m: m.group(1) + f'      - {spec.name}\n',
        flags=re.MULTILINE,
    )
    return p


def patch_control_params(spec: JointSpec) -> FilePatch:
    p = FilePatch(OMX / 'config/control_params.yaml')
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # Append joint_limits entry under fk_controller -> joint_limits.
    limits_block = (
        f'      {spec.name}:\n'
        f'        lower: {spec.lower}\n'
        f'        upper: {spec.upper}\n'
    )
    p.sub(
        r'(joint_limits:\s*\n(?:\s{6}\S+:\s*\n(?:\s{8}\S+:.*\n)+)+)',
        lambda m: m.group(1) + limits_block,
        flags=re.MULTILINE,
    )

    # Append to named_positions.init / .home.
    for state in ('init', 'home'):
        state_re = re.compile(
            rf'({state}:\s*\n(?:\s{{8}}\S+:.*\n)+)',
            re.MULTILINE,
        )
        p.content = state_re.sub(
            lambda m: m.group(1) + f'        {spec.name}: {spec.home}\n',
            p.content,
            count=1,
        )

    return p


def _try_sub(p: FilePatch, pattern: str, repl, flags: int = 0, count: int = 1) -> bool:
    """Substitute if the pattern matches; return whether anything changed."""
    new, n = re.subn(pattern, repl, p.content, count=count, flags=flags)
    if n == 0:
        return False
    p.content = new
    return True


def _patch_python_module(path: Path, spec: JointSpec) -> FilePatch:
    p = FilePatch(path)
    if fail_if_joint_present(p, spec.name, p.path.name):
        return p

    # ARM_JOINTS list (handles both class attr and module-level).
    _try_sub(
        p,
        r"ARM_JOINTS\s*=\s*\[([^\]]+)\]",
        lambda m: f"ARM_JOINTS = [{m.group(1).rstrip()}, '{spec.name}']",
    )

    # JOINT_LIMITS dict (may not exist in every module — best-effort).
    # Preserve indentation by reusing the last entry's leading whitespace.
    def _insert_limit(m: re.Match) -> str:
        body, closing = m.group(1), m.group(2)
        indent_match = re.search(r'\n([ \t]+)\'[^\']+\':\s*\(', body)
        indent = indent_match.group(1) if indent_match else '    '
        return f"{body.rstrip()}\n{indent}'{spec.name}': ({spec.lower}, {spec.upper}),{closing}"

    _try_sub(
        p,
        r"(JOINT_LIMITS\s*=\s*\{[^}]*?)(\n\s*\})",
        _insert_limit,
        flags=re.DOTALL,
    )
    return p


def patch_python_modules(spec: JointSpec) -> list[FilePatch]:
    return [
        _patch_python_module(OMX / 'omx_control/fk_controller.py', spec),
        _patch_python_module(OMX / 'omx_control/ik_controller.py', spec),
        _patch_python_module(OMX / 'omx_control/teach_playback.py', spec),
    ]


def patch_movejoints_srv(spec: JointSpec) -> FilePatch:
    p = FilePatch(OMX / 'srv/MoveJoints.srv')
    if spec.name in p.content:
        print(f'  [skip] {p.path.relative_to(REPO)}: {spec.name!r} already present')
        return p

    # Append joint to the comment listing joint order.
    p.sub(
        r'(\[joint1,\s*joint2,\s*joint3,\s*joint4)([^\]]*\])',
        lambda m: f'{m.group(1)}, {spec.name}{m.group(2)}',
    )
    return p


# ----------------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------------


def parse_triple(s: str, name: str) -> tuple[float, float, float]:
    parts = s.split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f'--{name} expects three space-separated numbers, got: {s!r}'
        )
    return tuple(float(x) for x in parts)  # type: ignore[return-value]


def make_spec(args: argparse.Namespace) -> JointSpec:
    # Default child link name: bump number suffix on parent if possible.
    if args.child:
        child = args.child
    else:
        m = re.match(r'^(.*?)(\d+)$', args.parent)
        if m:
            child = f'{m.group(1)}{int(m.group(2)) + 1}'
        else:
            child = f'{args.parent}_child'

    return JointSpec(
        name=args.name,
        parent=args.parent,
        child=child,
        jtype=args.type,
        axis=parse_triple(args.axis, 'axis'),
        origin_xyz=parse_triple(args.origin_xyz, 'origin-xyz'),
        origin_rpy=parse_triple(args.origin_rpy, 'origin-rpy'),
        lower=args.lower,
        upper=args.upper,
        velocity=args.velocity,
        effort=args.effort,
        dxl_id=args.dxl_id,
        home=args.home,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Scaffold a new joint across the OMX_jazzy stack.',
    )
    parser.add_argument('--name', required=True, help='Joint name, e.g. joint5')
    parser.add_argument('--parent', default='link5', help='Existing parent link (default: link5)')
    parser.add_argument('--child', default=None, help='New child link name (default: increment parent suffix)')
    parser.add_argument('--type', choices=('revolute', 'prismatic'), default='revolute')
    parser.add_argument('--axis', default='0 0 1', help='Joint axis xyz (default: "0 0 1")')
    parser.add_argument('--origin-xyz', default='0 0 0.05', help='Joint origin xyz')
    parser.add_argument('--origin-rpy', default='0 0 0', help='Joint origin rpy')
    parser.add_argument('--lower', type=float, default=-math.pi / 2)
    parser.add_argument('--upper', type=float, default=math.pi / 2)
    parser.add_argument('--velocity', type=float, default=4.8)
    parser.add_argument('--effort', type=float, default=1000.0)
    parser.add_argument('--dxl-id', type=int, default=16, help='Dynamixel bus ID (default: 16)')
    parser.add_argument('--home', type=float, default=0.0, help='Home/init position value')
    parser.add_argument('--apply', action='store_true', help='Write changes (default: dry-run)')

    args = parser.parse_args()
    spec = make_spec(args)

    patchers: list[Callable[[JointSpec], FilePatch | list[FilePatch]]] = [
        patch_urdf_arm,
        patch_gazebo,
        patch_ros2_control,
        patch_hardware_controller_manager,
        patch_initial_positions,
        patch_srdf,
        patch_moveit_joint_limits,
        patch_moveit_controllers,
        patch_control_params,
        patch_python_modules,
        patch_movejoints_srv,
    ]

    print(f'Adding joint: {spec.name} ({spec.jtype})')
    print(f'  parent={spec.parent} child={spec.child}')
    print(f'  axis=[{spec.axis_str()}] origin xyz=[{spec.xyz_str()}] rpy=[{spec.rpy_str()}]')
    print(f'  limits=[{spec.lower}, {spec.upper}] dxl ID={spec.dxl_id} home={spec.home}')
    print()

    all_patches: list[FilePatch] = []
    for fn in patchers:
        try:
            result = fn(spec)
        except Exception as exc:  # noqa: BLE001
            print(f'  [error] {fn.__name__}: {exc}', file=sys.stderr)
            return 1
        if isinstance(result, list):
            all_patches.extend(result)
        else:
            all_patches.append(result)

    changed = [p for p in all_patches if p.changed]
    unchanged = [p for p in all_patches if not p.changed]

    print(f'Files to update ({len(changed)}):')
    for p in changed:
        print(f'  - {p.path.relative_to(REPO)}')
    if unchanged:
        print(f'\nFiles unchanged ({len(unchanged)}):')
        for p in unchanged:
            print(f'  - {p.path.relative_to(REPO)}')

    if not args.apply:
        print('\nDry run — re-run with --apply to write changes (a .bak is saved alongside each modified file).')
        return 0

    for p in changed:
        p.write()
    print(f'\nWrote {len(changed)} files. Backups saved as <file>.bak.')
    print('Next steps:')
    print('  1. Review the diffs (git diff).')
    print('  2. Run scripts/verify_joints.sh.')
    print('  3. colcon build --symlink-install --allow-overriding dynamixel_sdk')
    print('  4. ros2 launch omx_control bringup_all.launch.py use_fake_hardware:=true')
    return 0


if __name__ == '__main__':
    sys.exit(main())
