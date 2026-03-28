#!/usr/bin/env python3
"""
IK Controller Node for Open Manipulator X
Provides inverse kinematics control via service and topic interfaces.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from moveit_msgs.msg import RobotState, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionClient
from rclpy.duration import Duration

import numpy as np
from typing import List, Optional


class IKController(Node):
    """Node providing IK control for Open Manipulator X."""

    # Joint names for the arm (excluding gripper)
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    # Planning group name from SRDF
    PLANNING_GROUP = 'arm'

    # End effector link
    END_EFFECTOR_LINK = 'end_effector_link'

    # Base link
    BASE_LINK = 'link1'

    def __init__(self):
        super().__init__('ik_controller')

        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('velocity_scaling', 0.5)
        self.declare_parameter('acceleration_scaling', 0.5)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)

        # Get parameters
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value

        # Current joint state
        self.current_joint_state: Optional[JointState] = None

        # Service clients
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )

        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self.callback_group
        )

        # Action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
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

        # Topic subscriber for target pose
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/omx/target_pose',
            self.target_pose_callback,
            10,
            callback_group=self.callback_group
        )

        # Publisher for current end-effector pose (for feedback)
        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            '/omx/current_pose',
            10
        )

        self.get_logger().info('IK Controller initialized')
        self.get_logger().info(f'Waiting for services: /compute_ik, /compute_cartesian_path')

    def joint_state_callback(self, msg: JointState):
        """Store current joint state."""
        self.current_joint_state = msg

    def target_pose_callback(self, msg: PoseStamped):
        """Handle target pose from topic."""
        self.get_logger().info(f'Received target pose: [{msg.pose.position.x:.3f}, '
                              f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]')

        success, message = self.move_to_pose(
            msg.pose,
            self.velocity_scaling,
            self.acceleration_scaling,
            wait=True
        )

        if success:
            self.get_logger().info(f'Motion completed: {message}')
        else:
            self.get_logger().error(f'Motion failed: {message}')

    def get_current_arm_positions(self) -> Optional[List[float]]:
        """Get current arm joint positions."""
        if self.current_joint_state is None:
            return None

        positions = []
        for joint_name in self.ARM_JOINTS:
            if joint_name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            else:
                return None
        return positions

    def compute_ik(self, target_pose: Pose) -> Optional[List[float]]:
        """Compute inverse kinematics for target pose."""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service not available')
            return None

        # Build IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.PLANNING_GROUP
        request.ik_request.robot_state.joint_state = self.current_joint_state or JointState()
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=5.0).to_msg()

        # Set target pose
        request.ik_request.pose_stamped.header.frame_id = self.BASE_LINK
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = target_pose

        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('IK service call failed')
            return None

        response = future.result()

        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(f'IK computation failed: error code {response.error_code.val}')
            return None

        # Extract arm joint positions from solution
        solution_positions = []
        for joint_name in self.ARM_JOINTS:
            if joint_name in response.solution.joint_state.name:
                idx = response.solution.joint_state.name.index(joint_name)
                solution_positions.append(response.solution.joint_state.position[idx])

        return solution_positions if len(solution_positions) == len(self.ARM_JOINTS) else None

    def execute_trajectory(self, joint_positions: List[float],
                          velocity_scaling: float,
                          acceleration_scaling: float,
                          wait: bool = True) -> tuple[bool, str]:
        """Execute joint trajectory to reach target positions."""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            return False, 'Trajectory action server not available'

        # Get current positions
        current_positions = self.get_current_arm_positions()
        if current_positions is None:
            return False, 'Could not get current joint positions'

        # Calculate trajectory duration based on max joint movement
        max_movement = max(abs(t - c) for t, c in zip(joint_positions, current_positions))
        base_velocity = 1.0  # rad/s at full speed
        duration = max_movement / (base_velocity * velocity_scaling)
        duration = max(duration, 0.5)  # Minimum duration

        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.ARM_JOINTS

        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.velocities = [0.0] * len(self.ARM_JOINTS)
        start_point.time_from_start = Duration(seconds=0.0).to_msg()
        trajectory.points.append(start_point)

        # End point (target position)
        end_point = JointTrajectoryPoint()
        end_point.positions = joint_positions
        end_point.velocities = [0.0] * len(self.ARM_JOINTS)
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
            return False, 'Trajectory goal rejected'

        if wait:
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10.0)

            result = result_future.result()
            if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                return True, 'Trajectory executed successfully'
            else:
                return False, f'Trajectory execution failed: error code {result.result.error_code}'
        else:
            return True, 'Trajectory sent (not waiting for completion)'

    def move_to_pose(self, target_pose: Pose,
                     velocity_scaling: float = 0.5,
                     acceleration_scaling: float = 0.5,
                     wait: bool = True) -> tuple[bool, str]:
        """Move end-effector to target pose using IK."""
        # Compute IK solution
        joint_positions = self.compute_ik(target_pose)
        if joint_positions is None:
            return False, 'Could not compute IK solution'

        self.get_logger().info(f'IK solution: {[f"{p:.3f}" for p in joint_positions]}')

        # Execute trajectory
        return self.execute_trajectory(
            joint_positions,
            velocity_scaling,
            acceleration_scaling,
            wait
        )

    def move_to_position(self, x: float, y: float, z: float,
                        qx: float = 0.0, qy: float = 0.0,
                        qz: float = 0.0, qw: float = 1.0,
                        velocity_scaling: float = 0.5,
                        acceleration_scaling: float = 0.5,
                        wait: bool = True) -> tuple[bool, str]:
        """Move end-effector to target position with optional orientation."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return self.move_to_pose(pose, velocity_scaling, acceleration_scaling, wait)


def main(args=None):
    rclpy.init(args=args)

    node = IKController()

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
