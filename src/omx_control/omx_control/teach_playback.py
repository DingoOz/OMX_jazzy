#!/usr/bin/env python3
"""
Teach and Playback TUI for Open Manipulator X.

Record arm poses by physically positioning the arm (torque off),
then play them back through MoveIt (OMPL planner) with visualization in RViz.
"""

import curses
import json
import logging
import os
import signal
import subprocess
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header
from std_srvs.srv import SetBool
from dynamixel_interfaces.srv import RebootDxl
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
)
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Set up file logger
LOG_FILE = Path.home() / 'teach_playback.log'
logger = logging.getLogger('teach_playback')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(LOG_FILE, mode='w')
fh.setFormatter(logging.Formatter(
    '%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S'))
logger.addHandler(fh)


ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_JOINT = 'gripper_left_joint'
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
PLANNING_GROUP = 'arm'
DEFAULT_SAVE_FILE = str(Path.home() / 'waypoints.json')

# Joint limits from URDF (arm only - gripper uses its own controller limits)
JOINT_LIMITS = {
    'joint1': (-3.14159, 3.14159),
    'joint2': (-1.5, 1.5),
    'joint3': (-1.5, 1.4),
    'joint4': (-1.7, 1.97),
}


def clamp_joints(positions: dict) -> dict:
    """Clamp joint positions to within URDF limits."""
    clamped = {}
    for j, val in positions.items():
        if j in JOINT_LIMITS:
            lo, hi = JOINT_LIMITS[j]
            clamped_val = max(lo, min(hi, val))
            if clamped_val != val:
                logger.warning(
                    f'Clamped {j}: {val:.4f} -> {clamped_val:.4f} '
                    f'(limits: [{lo}, {hi}])')
            clamped[j] = clamped_val
        else:
            clamped[j] = val
    return clamped


class TeachPlaybackNode(Node):
    def __init__(self):
        super().__init__('teach_playback')

        self.current_positions = {}
        self.torque_on = True
        self.lock = threading.Lock()

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)

        self.torque_client = self.create_client(
            SetBool, '/dynamixel_hardware_interface/set_dxl_torque')

        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action')

        self.gripper_pub = self.create_publisher(
            Float64, '/omx/gripper_position', 10)

        # Gripper action client (GripperCommand)
        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Trajectory publisher to sync arm_controller before torque on
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Reboot service client
        self.reboot_client = self.create_client(
            RebootDxl, '/dynamixel_hardware_interface/reboot_dxl')

    def joint_state_cb(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.current_positions[name] = pos
            logger.debug(
                'Joint states: ' +
                ', '.join(f'{n}={p:.4f}' for n, p in
                          zip(msg.name, msg.position)))

    def get_positions(self):
        with self.lock:
            return dict(self.current_positions)

    def move_direct(self, target: dict, duration: float = 5.0):
        """Move arm directly via arm_controller trajectory, bypassing MoveIt.
        Used to get the arm to a valid position before MoveIt planning."""
        clamped = clamp_joints(target)

        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [clamped.get(j, 0.0) for j in ARM_JOINTS]
        point.velocities = [0.0] * len(ARM_JOINTS)
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        msg.points.append(point)

        logger.info('Direct move to: ' +
                     ', '.join(f'{j}={clamped.get(j, 0.0):.4f}' for j in ARM_JOINTS))
        self.traj_pub.publish(msg)

        # Wait for motion and verify arrival
        time.sleep(duration + 1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        final = self.get_positions()
        errors = [abs(clamped.get(j, 0.0) - final.get(j, 0.0)) for j in ARM_JOINTS]
        max_err = max(errors)
        logger.info('Direct move arrived at: ' +
                     ', '.join(f'{j}={final.get(j, 0.0):.4f}' for j in ARM_JOINTS))
        logger.info(f'Direct move max error: {max_err:.4f} rad')
        if max_err > 0.1:
            logger.warning(f'Arm may not have reached target, waiting longer...')
            time.sleep(3.0)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)

    def sync_controller_to_current(self):
        """Publish current joint positions to arm_controller so its
        internal goal matches the physical arm position. This prevents
        the arm from snapping back to a stale goal when torque is enabled."""
        rclpy.spin_once(self, timeout_sec=0.1)
        pos = self.get_positions()
        clamped = clamp_joints(pos)

        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [clamped.get(j, 0.0) for j in ARM_JOINTS]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100ms
        msg.points.append(point)

        logger.info('Syncing arm_controller to current position: ' +
                     ', '.join(f'{j}={clamped.get(j, 0.0):.4f}' for j in ARM_JOINTS))
        self.traj_pub.publish(msg)
        time.sleep(0.2)

    def set_torque(self, enable: bool) -> str:
        logger.info(f'Setting torque {"ON" if enable else "OFF"}')

        # Before enabling torque, sync the controller to current position
        # so the arm doesn't snap back to a stale goal
        if enable:
            self.sync_controller_to_current()

        if not self.torque_client.wait_for_service(timeout_sec=3.0):
            logger.error('Torque service not available')
            return 'Torque service not available'
        req = SetBool.Request()
        req.data = enable
        future = self.torque_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.torque_on = enable
            logger.info(f'Torque {"ON" if enable else "OFF"}: {future.result().message}')
            return f'Torque {"ON" if enable else "OFF"}'
        logger.error('Torque service call failed')
        return 'Torque service call failed'

    def move_to_joint_target(self, joint_positions: dict,
                              velocity_scaling: float = 0.5) -> tuple:
        logger.info('=== MoveGroup Goal ===')
        logger.info(f'  Target joints: ' +
                     ', '.join(f'{j}={joint_positions[j]:.4f}' for j in ARM_JOINTS))
        logger.info(f'  Velocity scaling: {velocity_scaling}')
        logger.info(f'  Pipeline: ompl')

        # Log current positions for comparison
        current = self.get_positions()
        logger.info(f'  Current joints: ' +
                     ', '.join(f'{j}={current.get(j, 0.0):.4f}' for j in ARM_JOINTS))
        logger.info(f'  Deltas: ' +
                     ', '.join(f'{j}={joint_positions[j] - current.get(j, 0.0):+.4f}'
                               for j in ARM_JOINTS))

        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            logger.error('MoveGroup action server not available')
            return False, 'MoveGroup not available'

        # Clamp target positions to joint limits
        clamped_target = clamp_joints(joint_positions)

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = PLANNING_GROUP
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = velocity_scaling
        request.pipeline_id = 'ompl'

        # Use current monitored state as start (no explicit override).
        # The arm should already be at a valid position from direct move
        # or a previous MoveIt execution.

        constraints = Constraints()
        for joint_name in ARM_JOINTS:
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = clamped_target[joint_name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            logger.debug(f'  Constraint: {joint_name} = {jc.position:.4f} '
                          f'(tol: ±{jc.tolerance_above})')

        request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        logger.info('Sending goal to MoveGroup...')
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.error('Goal REJECTED by MoveGroup')
            return False, 'Goal rejected'

        logger.info('Goal ACCEPTED, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            logger.error('No result received (timeout?)')
            return False, 'No result'

        error_code = result.result.error_code.val
        logger.info(f'MoveGroup result error_code: {error_code}')

        # Log the planned trajectory details
        traj = result.result.planned_trajectory
        if traj and traj.joint_trajectory.points:
            pts = traj.joint_trajectory.points
            logger.info(f'  Planned trajectory: {len(pts)} points')
            logger.info(f'  Joint names: {traj.joint_trajectory.joint_names}')
            logger.info(f'  Start point: {[f"{p:.4f}" for p in pts[0].positions]}')
            logger.info(f'  End point:   {[f"{p:.4f}" for p in pts[-1].positions]}')
            duration = pts[-1].time_from_start
            logger.info(f'  Duration: {duration.sec}.{duration.nanosec // 1000000:03d}s')
        else:
            logger.warning('  No trajectory in result')

        # Log actual position after execution
        time.sleep(0.2)
        rclpy.spin_once(self, timeout_sec=0.1)
        final = self.get_positions()
        logger.info(f'  Final joints: ' +
                     ', '.join(f'{j}={final.get(j, 0.0):.4f}' for j in ARM_JOINTS))
        logger.info(f'  Final errors: ' +
                     ', '.join(f'{j}={joint_positions[j] - final.get(j, 0.0):+.4f}'
                               for j in ARM_JOINTS))

        if error_code == 1:  # SUCCESS
            logger.info('=== Motion SUCCESS ===')
            return True, 'OK'
        else:
            logger.error(f'=== Motion FAILED (error_code={error_code}) ===')
            return False, f'Error {error_code}'

    def reboot_servos(self) -> str:
        """Reboot all Dynamixel servos to clear error states."""
        logger.info('Rebooting all servos')
        if not self.reboot_client.wait_for_service(timeout_sec=3.0):
            logger.error('Reboot service not available')
            return 'Reboot service not available'
        req = RebootDxl.Request()
        req.header = Header()
        req.header.stamp = self.get_clock().now().to_msg()
        future = self.reboot_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            if future.result().result:
                logger.info('Reboot successful')
                return 'All servos rebooted'
            else:
                logger.error('Reboot returned false')
                return 'Reboot failed'
        logger.error('Reboot service call failed')
        return 'Reboot service call failed'

    def move_gripper(self, position: float):
        """Move gripper via GripperCommand action."""
        logger.info(f'Moving gripper to {position:.4f}m')
        if not self.gripper_action_client.wait_for_server(timeout_sec=3.0):
            logger.error('Gripper action server not available')
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.5
        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            result = result_future.result()
            if result:
                logger.info(f'Gripper result: pos={result.result.position:.4f}, '
                            f'stalled={result.result.stalled}, '
                            f'reached={result.result.reached_goal}')
            else:
                logger.warning('Gripper: no result')
        else:
            logger.error('Gripper goal rejected')


class TeachPlaybackTUI:
    def __init__(self, node: TeachPlaybackNode):
        self.node = node
        self.waypoints = []
        self.status = 'Ready'
        self.status_ok = True
        self.scroll_offset = 0
        self.playing = False
        self.velocity = 0.5

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.timeout(100)  # 100ms refresh

        # Colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)    # title
        curses.init_pair(2, curses.COLOR_CYAN, -1)     # labels
        curses.init_pair(3, curses.COLOR_YELLOW, -1)   # values
        curses.init_pair(4, curses.COLOR_RED, -1)      # torque off / errors
        curses.init_pair(5, curses.COLOR_GREEN, -1)    # torque on / success
        curses.init_pair(6, curses.COLOR_WHITE, -1)    # normal
        curses.init_pair(7, curses.COLOR_MAGENTA, -1)  # waypoint highlight

        while True:
            # Spin ROS
            rclpy.spin_once(self.node, timeout_sec=0)

            stdscr.erase()
            h, w = stdscr.getmaxyx()
            w = min(w, 80)

            row = 0

            # Title
            title = ' Teach & Playback  (MoveIt OMPL) '
            stdscr.addstr(row, max(0, (w - len(title)) // 2), title,
                          curses.A_BOLD | curses.color_pair(1))
            row += 1
            stdscr.addstr(row, 0, '─' * w, curses.color_pair(2))
            row += 1

            # Current positions
            stdscr.addstr(row, 1, 'CURRENT JOINTS', curses.A_BOLD | curses.color_pair(2))
            row += 1

            positions = self.node.get_positions()
            col1_x, col2_x = 3, 28
            for i, jname in enumerate(ARM_JOINTS):
                val = positions.get(jname, 0.0)
                x = col1_x if i % 2 == 0 else col2_x
                if i % 2 == 0 and i > 0:
                    row += 1
                label = f'{jname}:'
                stdscr.addstr(row, x, label, curses.color_pair(6))
                stdscr.addstr(row, x + len(label) + 1,
                              f'{val:+.4f}', curses.A_BOLD | curses.color_pair(3))
            row += 1

            grip_val = positions.get(GRIPPER_JOINT, 0.0)
            stdscr.addstr(row, col1_x, 'gripper:', curses.color_pair(6))
            stdscr.addstr(row, col1_x + 9,
                          f'{grip_val:+.4f} m', curses.A_BOLD | curses.color_pair(3))

            # Torque status
            torque_str = ' TORQUE ON ' if self.node.torque_on else ' TORQUE OFF '
            torque_color = curses.color_pair(5) if self.node.torque_on else curses.color_pair(4)
            stdscr.addstr(row, col2_x, torque_str,
                          curses.A_BOLD | curses.A_REVERSE | torque_color)
            row += 1

            stdscr.addstr(row, 0, '─' * w, curses.color_pair(2))
            row += 1

            # Waypoints
            wp_header = f'WAYPOINTS ({len(self.waypoints)})'
            stdscr.addstr(row, 1, wp_header, curses.A_BOLD | curses.color_pair(2))

            vel_str = f'vel: {self.velocity:.1f}'
            stdscr.addstr(row, w - len(vel_str) - 2, vel_str, curses.color_pair(6))
            row += 1

            # Waypoint list area
            wp_area_height = max(3, h - row - 8)
            if not self.waypoints:
                stdscr.addstr(row, 3, '(none - press [r] to record)',
                              curses.color_pair(6) | curses.A_DIM)
                row += 1
            else:
                visible = self.waypoints[self.scroll_offset:
                                         self.scroll_offset + wp_area_height]
                for i, wp in enumerate(visible):
                    idx = self.scroll_offset + i
                    parts = []
                    for j in ARM_JOINTS:
                        parts.append(f'{j[0]}{j[-1]}={wp.get(j, 0.0):+.3f}')
                    parts.append(f'g={wp.get(GRIPPER_JOINT, 0.0):+.4f}')
                    line = f' {idx + 1:>3}: {" ".join(parts)}'
                    stdscr.addstr(row, 1, line[:w - 2],
                                  curses.color_pair(7))
                    row += 1

                if len(self.waypoints) > wp_area_height:
                    more = len(self.waypoints) - self.scroll_offset - wp_area_height
                    if more > 0:
                        stdscr.addstr(row, 3, f'... {more} more (scroll: ↑/↓)',
                                      curses.color_pair(6) | curses.A_DIM)
                    row += 1

            # Pad to controls area
            controls_row = h - 7
            if row < controls_row:
                row = controls_row

            stdscr.addstr(row, 0, '─' * w, curses.color_pair(2))
            row += 1

            # Controls
            stdscr.addstr(row, 1, 'CONTROLS', curses.A_BOLD | curses.color_pair(2))
            row += 1

            controls = [
                ('[t] Torque OFF', '[o] Torque ON', '[r] Record pose'),
                ('[p] Playback', '[s] Save', '[l] Load'),
                ('[d] Delete last', '[c] Clear all', '[+/-] Velocity'),
                ('[x] Reboot servos', '[↑/↓] Scroll', '[q] Quit'),
            ]
            for line_items in controls:
                col = 3
                for item in line_items:
                    if not item:
                        continue
                    if row < h - 2:
                        # Highlight the key
                        bracket = item.find(']')
                        stdscr.addstr(row, col, item[:bracket + 1],
                                      curses.A_BOLD | curses.color_pair(3))
                        stdscr.addstr(row, col + bracket + 1, item[bracket + 1:],
                                      curses.color_pair(6))
                    col += 20
                row += 1

            # Status bar
            status_row = h - 1
            status_color = curses.color_pair(5) if self.status_ok else curses.color_pair(4)
            stdscr.addstr(status_row, 0, ' ' * (w - 1),
                          curses.A_REVERSE | status_color)
            stdscr.addstr(status_row, 1, self.status[:w - 2],
                          curses.A_REVERSE | curses.A_BOLD | status_color)

            stdscr.refresh()

            # Handle input
            key = stdscr.getch()
            if key == -1:
                continue

            if key == ord('q'):
                self.set_status('Enabling torque and exiting...')
                stdscr.refresh()
                self.node.set_torque(True)
                break

            elif key == ord('t'):
                result = self.node.set_torque(False)
                self.set_status(f'{result} - move arm by hand, [r] to record')

            elif key == ord('o'):
                result = self.node.set_torque(True)
                self.set_status(result)

            elif key == ord('r'):
                rclpy.spin_once(self.node, timeout_sec=0.1)
                pos = self.node.get_positions()
                raw_wp = {j: pos.get(j, 0.0) for j in ALL_JOINTS}
                wp = clamp_joints(raw_wp)  # Only clamps arm joints
                self.waypoints.append(wp)
                logger.info(f'RECORDED waypoint {len(self.waypoints)}: ' +
                            ', '.join(f'{j}={wp[j]:.4f}' for j in ALL_JOINTS))
                self.set_status(f'Recorded waypoint {len(self.waypoints)}')
                # Auto-scroll to bottom
                max_scroll = max(0, len(self.waypoints) - wp_area_height)
                self.scroll_offset = max_scroll

            elif key == ord('p'):
                if not self.waypoints:
                    self.set_status('No waypoints to play back', ok=False)
                else:
                    self._playback(stdscr, h, w)

            elif key == ord('s'):
                self._save_dialog(stdscr, h, w)

            elif key == ord('l'):
                self._load_dialog(stdscr, h, w)

            elif key == ord('d'):
                if self.waypoints:
                    self.waypoints.pop()
                    self.set_status(f'Deleted last waypoint ({len(self.waypoints)} remaining)')
                    max_scroll = max(0, len(self.waypoints) - wp_area_height)
                    self.scroll_offset = min(self.scroll_offset, max_scroll)
                else:
                    self.set_status('No waypoints to delete', ok=False)

            elif key == ord('c'):
                count = len(self.waypoints)
                self.waypoints.clear()
                self.scroll_offset = 0
                self.set_status(f'Cleared {count} waypoints')

            elif key == ord('+') or key == ord('='):
                self.velocity = min(1.0, self.velocity + 0.1)
                self.set_status(f'Velocity scaling: {self.velocity:.1f}')

            elif key == ord('-') or key == ord('_'):
                self.velocity = max(0.1, self.velocity - 0.1)
                self.set_status(f'Velocity scaling: {self.velocity:.1f}')

            elif key == ord('x'):
                self.set_status('Rebooting all servos...')
                self._refresh_status(stdscr, h, w)
                result = self.node.reboot_servos()
                self.set_status(result)

            elif key == curses.KEY_UP:
                self.scroll_offset = max(0, self.scroll_offset - 1)

            elif key == curses.KEY_DOWN:
                max_scroll = max(0, len(self.waypoints) - wp_area_height)
                self.scroll_offset = min(max_scroll, self.scroll_offset + 1)

    def set_status(self, msg: str, ok: bool = True):
        self.status = msg
        self.status_ok = ok

    def _playback(self, stdscr, h, w):
        logger.info(f'=== PLAYBACK START ({len(self.waypoints)} waypoints, '
                     f'vel={self.velocity}) ===')
        self.node.set_torque(True)
        time.sleep(0.5)

        # Move directly to first waypoint to ensure valid start state
        # (arm may be out of joint limits after teach mode)
        self.set_status('Moving to start position (direct)...')
        self._refresh_status(stdscr, h, w)
        self.node.move_direct(self.waypoints[0], duration=5.0)

        # Move gripper for first waypoint
        if GRIPPER_JOINT in self.waypoints[0]:
            self.node.move_gripper(self.waypoints[0][GRIPPER_JOINT])
            time.sleep(1.0)

        self.set_status('Waypoint 1 reached, starting MoveIt playback...')
        self._refresh_status(stdscr, h, w)

        # Use MoveIt OMPL for remaining waypoints
        for i, wp in enumerate(self.waypoints[1:], start=2):
            logger.info(f'--- Waypoint {i}/{len(self.waypoints)} ---')
            self.set_status(
                f'Playing {i}/{len(self.waypoints)} - planning via OMPL...')
            self._refresh_status(stdscr, h, w)

            success, msg = self.node.move_to_joint_target(
                wp, velocity_scaling=self.velocity)

            if not success:
                self.set_status(
                    f'Failed at waypoint {i}: {msg}', ok=False)
                return

            # Move gripper
            if GRIPPER_JOINT in wp:
                self.set_status(
                    f'Waypoint {i}/{len(self.waypoints)} - moving gripper...')
                self._refresh_status(stdscr, h, w)
                self.node.move_gripper(wp[GRIPPER_JOINT])
                time.sleep(1.0)

            self.set_status(
                f'Waypoint {i}/{len(self.waypoints)} complete')
            self._refresh_status(stdscr, h, w)
            time.sleep(0.3)

        self.set_status(f'Playback complete ({len(self.waypoints)} waypoints)')

    def _refresh_status(self, stdscr, h, w):
        """Quick refresh of just the status bar."""
        rclpy.spin_once(self.node, timeout_sec=0)
        status_row = h - 1
        status_color = (curses.color_pair(5) if self.status_ok
                        else curses.color_pair(4))
        stdscr.addstr(status_row, 0, ' ' * (w - 1),
                      curses.A_REVERSE | status_color)
        stdscr.addstr(status_row, 1, self.status[:w - 2],
                      curses.A_REVERSE | curses.A_BOLD | status_color)
        stdscr.refresh()

    def _save_dialog(self, stdscr, h, w):
        curses.echo()
        curses.curs_set(1)
        stdscr.addstr(h - 1, 0, ' ' * (w - 1))
        stdscr.addstr(h - 1, 1, f'Save to [{DEFAULT_SAVE_FILE}]: ')
        stdscr.refresh()
        try:
            path = stdscr.getstr(h - 1, 32, 200).decode().strip()
        except Exception:
            path = ''
        curses.noecho()
        curses.curs_set(0)

        if not path:
            path = DEFAULT_SAVE_FILE

        try:
            with open(path, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            self.set_status(f'Saved {len(self.waypoints)} waypoints to {path}')
        except Exception as e:
            self.set_status(f'Save failed: {e}', ok=False)

    def _load_dialog(self, stdscr, h, w):
        curses.echo()
        curses.curs_set(1)
        stdscr.addstr(h - 1, 0, ' ' * (w - 1))
        stdscr.addstr(h - 1, 1, f'Load from [{DEFAULT_SAVE_FILE}]: ')
        stdscr.refresh()
        try:
            path = stdscr.getstr(h - 1, 34, 200).decode().strip()
        except Exception:
            path = ''
        curses.noecho()
        curses.curs_set(0)

        if not path:
            path = DEFAULT_SAVE_FILE

        try:
            with open(path, 'r') as f:
                self.waypoints = json.load(f)
            self.scroll_offset = 0
            self.set_status(f'Loaded {len(self.waypoints)} waypoints from {path}')
        except FileNotFoundError:
            self.set_status(f'File not found: {path}', ok=False)
        except Exception as e:
            self.set_status(f'Load failed: {e}', ok=False)


def kill_conflicting_nodes():
    """Kill omx_control nodes that conflict with teach_playback action clients."""
    nodes_to_kill = ['ik_controller', 'fk_controller', 'gripper_controller']
    for name in nodes_to_kill:
        subprocess.run(
            ['pkill', '-f', f'__node:={name}'],
            capture_output=True)
    time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    # Kill conflicting omx_control nodes that share action clients
    print('Stopping conflicting controller nodes...')
    kill_conflicting_nodes()

    node = TeachPlaybackNode()

    # Wait for joint states before starting TUI
    print('Waiting for joint states...')
    while not node.get_positions():
        rclpy.spin_once(node, timeout_sec=0.5)
    print('Connected. Launching TUI...')

    tui = TeachPlaybackTUI(node)
    try:
        curses.wrapper(tui.run)
    except KeyboardInterrupt:
        node.set_torque(True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
