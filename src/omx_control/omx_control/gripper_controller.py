#!/usr/bin/env python3
"""
Gripper Controller Node for Open Manipulator X
Provides gripper control via service and topic interfaces.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionClient
from rclpy.duration import Duration

from typing import Optional


class GripperController(Node):
    """Node providing gripper control for Open Manipulator X."""

    # Gripper joint names
    GRIPPER_JOINTS = ['gripper_left_joint']  # Right joint is mirrored

    # Gripper limits (meters) from URDF
    GRIPPER_CLOSED = -0.010  # Fully closed
    GRIPPER_OPEN = 0.019     # Fully open

    def __init__(self):
        super().__init__('gripper_controller')

        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('default_effort', 0.5)
        self.declare_parameter('gripper_speed', 0.02)  # m/s

        # Get parameters
        self.default_effort = self.get_parameter('default_effort').value
        self.gripper_speed = self.get_parameter('gripper_speed').value

        # Current joint state
        self.current_joint_state: Optional[JointState] = None

        # Action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        # Command topic subscriber (string commands: "open", "close")
        self.command_sub = self.create_subscription(
            String,
            '/omx/gripper_command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )

        # Position topic subscriber (for direct position control)
        self.position_sub = self.create_subscription(
            Float64,
            '/omx/gripper_position',
            self.position_callback,
            10,
            callback_group=self.callback_group
        )

        # Publisher for current gripper state
        self.state_pub = self.create_publisher(
            Float64,
            '/omx/gripper_state',
            10
        )

        # Timer for publishing state
        self.state_timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info('Gripper Controller initialized')
        self.get_logger().info(f'Gripper range: [{self.GRIPPER_CLOSED:.3f}, {self.GRIPPER_OPEN:.3f}] m')

    def joint_state_callback(self, msg: JointState):
        """Store current joint state."""
        self.current_joint_state = msg

    def publish_state(self):
        """Publish current gripper state."""
        position = self.get_current_gripper_position()
        if position is not None:
            msg = Float64()
            msg.data = position
            self.state_pub.publish(msg)

    def command_callback(self, msg: String):
        """Handle string commands (open/close)."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received gripper command: {command}')

        if command == 'open':
            success, message = self.open_gripper()
        elif command == 'close':
            success, message = self.close_gripper()
        else:
            self.get_logger().error(f'Unknown command: {command}. Use "open" or "close"')
            return

        if success:
            self.get_logger().info(f'Gripper command completed: {message}')
        else:
            self.get_logger().error(f'Gripper command failed: {message}')

    def position_callback(self, msg: Float64):
        """Handle direct position commands."""
        self.get_logger().info(f'Received gripper position: {msg.data:.4f}')

        success, message, _ = self.set_position(msg.data)

        if success:
            self.get_logger().info(f'Gripper position set: {message}')
        else:
            self.get_logger().error(f'Failed to set gripper position: {message}')

    def get_current_gripper_position(self) -> Optional[float]:
        """Get current gripper position."""
        if self.current_joint_state is None:
            return None

        joint_name = self.GRIPPER_JOINTS[0]
        if joint_name in self.current_joint_state.name:
            idx = self.current_joint_state.name.index(joint_name)
            return self.current_joint_state.position[idx]
        return None

    def validate_position(self, position: float) -> tuple[bool, str, float]:
        """Validate and clamp gripper position."""
        if position < self.GRIPPER_CLOSED:
            return True, f'Position clamped to closed ({self.GRIPPER_CLOSED})', self.GRIPPER_CLOSED
        elif position > self.GRIPPER_OPEN:
            return True, f'Position clamped to open ({self.GRIPPER_OPEN})', self.GRIPPER_OPEN
        return True, 'Valid', position

    def execute_gripper_trajectory(self, target_position: float,
                                   wait: bool = True) -> tuple[bool, str]:
        """Execute gripper trajectory to reach target position."""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            return False, 'Gripper trajectory action server not available'

        # Get current position
        current_position = self.get_current_gripper_position()
        if current_position is None:
            return False, 'Could not get current gripper position'

        # Calculate duration
        movement = abs(target_position - current_position)
        duration = movement / self.gripper_speed
        duration = max(duration, 0.3)  # Minimum duration

        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.GRIPPER_JOINTS

        # Start point
        start_point = JointTrajectoryPoint()
        start_point.positions = [current_position]
        start_point.velocities = [0.0]
        start_point.time_from_start = Duration(seconds=0.0).to_msg()
        trajectory.points.append(start_point)

        # End point
        end_point = JointTrajectoryPoint()
        end_point.positions = [target_position]
        end_point.velocities = [0.0]
        end_point.time_from_start = Duration(seconds=duration).to_msg()
        trajectory.points.append(end_point)

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, 'Gripper trajectory goal rejected'

        if wait:
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5.0)

            result = result_future.result()
            if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                return True, 'Gripper trajectory executed successfully'
            else:
                return False, f'Gripper trajectory failed: error code {result.result.error_code}'
        else:
            return True, 'Gripper trajectory sent (not waiting)'

    def open_gripper(self, wait: bool = True) -> tuple[bool, str]:
        """Open the gripper fully."""
        self.get_logger().info('Opening gripper...')
        return self.execute_gripper_trajectory(self.GRIPPER_OPEN, wait)

    def close_gripper(self, wait: bool = True) -> tuple[bool, str]:
        """Close the gripper fully."""
        self.get_logger().info('Closing gripper...')
        return self.execute_gripper_trajectory(self.GRIPPER_CLOSED, wait)

    def set_position(self, position: float,
                     wait: bool = True) -> tuple[bool, str, float]:
        """Set gripper to specific position."""
        # Validate and clamp position
        valid, message, clamped_position = self.validate_position(position)

        self.get_logger().info(f'Setting gripper to position: {clamped_position:.4f}')

        success, exec_message = self.execute_gripper_trajectory(clamped_position, wait)

        if not success:
            return False, exec_message, clamped_position

        return True, message, clamped_position

    def get_gripper_percentage(self) -> Optional[float]:
        """Get gripper opening as percentage (0=closed, 100=open)."""
        position = self.get_current_gripper_position()
        if position is None:
            return None

        range_size = self.GRIPPER_OPEN - self.GRIPPER_CLOSED
        percentage = (position - self.GRIPPER_CLOSED) / range_size * 100.0
        return max(0.0, min(100.0, percentage))


def main(args=None):
    rclpy.init(args=args)

    node = GripperController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
