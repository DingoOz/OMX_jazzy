#!/usr/bin/env python3
"""
FK Controller Node for Open Manipulator X
Provides forward kinematics control via service and topic interfaces.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from omx_control.srv import MoveJoints

from moveit_msgs.srv import GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionClient
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException

import numpy as np
from typing import List, Optional


class FKController(Node):
    """Node providing FK control for Open Manipulator X."""

    # Joint names for the arm (excluding gripper)
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    # Planning group name from SRDF
    PLANNING_GROUP = 'arm'

    # End effector link
    END_EFFECTOR_LINK = 'end_effector_link'

    # Base link
    BASE_LINK = 'link1'

    def __init__(self):
        super().__init__('fk_controller')

        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters (values primarily come from control_params.yaml)
        self.declare_parameter('velocity_scaling', 0.5)
        self.declare_parameter('acceleration_scaling', 0.5)
        self.declare_parameter('interpolation_steps', 10)

        # Joint limits and named positions are provided by the yaml under fk_controller and /**
        # Declare with defaults matching the yaml for safety if no param file is used.
        default_limits = {
            'joint1': (-3.14159, 3.14159),
            'joint2': (-1.5, 1.5),
            'joint3': (-1.5, 1.4),
            'joint4': (-1.7, 1.97),
        }
        self.declare_parameter('joint_limits', default_limits)

        default_named = {
            'init': [0.0, 0.0, 0.0, 0.0],
            'home': [0.0, -1.0, 0.7, 0.3],
        }
        self.declare_parameter('named_positions', default_named)

        # Get parameters
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').value
        self.interpolation_steps = self.get_parameter('interpolation_steps').value

        # Load joint limits from parameter (yaml or override)
        self.joint_limits = default_limits
        try:
            jl = self.get_parameter('joint_limits').value
            if isinstance(jl, dict):
                loaded = {}
                for j, v in jl.items():
                    if isinstance(v, dict) and 'lower' in v and 'upper' in v:
                        loaded[j] = (float(v['lower']), float(v['upper']))
                    elif isinstance(v, (list, tuple)) and len(v) == 2:
                        loaded[j] = (float(v[0]), float(v[1]))
                if loaded:
                    self.joint_limits = loaded
        except Exception as e:
            self.get_logger().warn(f'Using default joint limits: {e}')

        # Current joint state
        self.current_joint_state: Optional[JointState] = None

        # Service client for FK computation
        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk',
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

        # Topic subscriber for target joints
        self.target_joints_sub = self.create_subscription(
            JointState,
            '/omx/target_joints',
            self.target_joints_callback,
            10,
            callback_group=self.callback_group
        )

        # Publisher for current end-effector pose
        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            '/omx/current_pose',
            10
        )

        # Publisher for current joint states (filtered for arm only)
        self.arm_joints_pub = self.create_publisher(
            JointState,
            '/omx/arm_joint_states',
            10
        )

        # Custom high-level service (documented API)
        self.move_joints_srv = self.create_service(
            MoveJoints,
            '/omx/move_joints',
            self._move_joints_callback,
            callback_group=self.callback_group
        )

        # TF for current end-effector pose (published from the node that already
        # owns joint filtering and FK). This makes /omx/current_pose actually work
        # (previously the publisher was created in both controllers but never used).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publish current pose at a sensible monitoring rate (TF lookup is cheap;
        # we avoid doing it at full joint_states rate).
        self.current_pose_timer = self.create_timer(
            0.1, self._publish_current_pose, callback_group=self.callback_group
        )

        self.get_logger().info('FK Controller initialized')
        self.get_logger().info(f'Arm joints: {self.ARM_JOINTS}')

    def joint_state_callback(self, msg: JointState):
        """Store current joint state and publish arm-only state."""
        self.current_joint_state = msg

        # Publish filtered arm joint state
        arm_state = JointState()
        arm_state.header = msg.header
        arm_state.name = []
        arm_state.position = []
        arm_state.velocity = []
        arm_state.effort = []

        for joint_name in self.ARM_JOINTS:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                arm_state.name.append(joint_name)
                arm_state.position.append(msg.position[idx])
                if len(msg.velocity) > idx:
                    arm_state.velocity.append(msg.velocity[idx])
                if len(msg.effort) > idx:
                    arm_state.effort.append(msg.effort[idx])

        self.arm_joints_pub.publish(arm_state)

    def _publish_current_pose(self):
        """Publish the current end-effector pose by looking up the TF tree.

        The tree is maintained by robot_state_publisher from the URDF + current
        /joint_states. This is the authoritative "where the arm actually is" and
        works in both real hardware and simulation without requiring the MoveIt
        FK service.
        """
        if self.current_joint_state is None:
            return

        # Quick check that we have a complete arm state
        if self.get_current_arm_positions() is None:
            return

        try:
            # Use zero time for latest available transform
            trans = self.tf_buffer.lookup_transform(
                self.BASE_LINK,
                self.END_EFFECTOR_LINK,
                rclpy.time.Time(),  # latest
                timeout=Duration(seconds=0.1).to_msg()
            )

            pose_msg = PoseStamped()
            pose_msg.header = trans.header
            # Ensure consistent base frame
            pose_msg.header.frame_id = self.BASE_LINK
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation

            self.current_pose_pub.publish(pose_msg)
        except TransformException as ex:
            # Transient during startup or if TF tree not fully populated yet.
            # Avoid log spam at 10 Hz.
            self.get_logger().debug(f'Could not lookup current EE pose TF: {ex}')

    def target_joints_callback(self, msg: JointState):
        """Handle target joints from topic."""
        # Extract positions for our joints
        positions = []
        for joint_name in self.ARM_JOINTS:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            else:
                self.get_logger().error(f'Joint {joint_name} not found in target message')
                return

        self.get_logger().info(f'Received target joints: {[f"{p:.3f}" for p in positions]}')

        success, message, _ = self.move_joints(
            positions,
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

    def validate_joint_positions(self, positions: List[float]) -> tuple[bool, str]:
        """Validate joint positions against limits (loaded from control_params.yaml)."""
        if len(positions) != len(self.ARM_JOINTS):
            return False, f'Expected {len(self.ARM_JOINTS)} positions, got {len(positions)}'

        for i, (joint_name, pos) in enumerate(zip(self.ARM_JOINTS, positions)):
            if joint_name not in self.joint_limits:
                continue
            lower, upper = self.joint_limits[joint_name]
            if pos < lower or pos > upper:
                return False, f'{joint_name} position {pos:.3f} outside limits [{lower:.3f}, {upper:.3f}]'

        return True, 'Valid'

    def compute_fk(self, joint_positions: List[float]) -> Optional[Pose]:
        """Compute forward kinematics to get end-effector pose."""
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('FK service not available')
            return None

        # Build FK request
        request = GetPositionFK.Request()
        request.header.frame_id = self.BASE_LINK
        request.header.stamp = self.get_clock().now().to_msg()
        request.fk_link_names = [self.END_EFFECTOR_LINK]

        # Set robot state with target positions
        request.robot_state.joint_state.name = self.ARM_JOINTS
        request.robot_state.joint_state.position = joint_positions

        # Call service
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('FK service call failed')
            return None

        response = future.result()

        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(f'FK computation failed: error code {response.error_code.val}')
            return None

        if len(response.pose_stamped) > 0:
            return response.pose_stamped[0].pose

        return None

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

        # Build trajectory with interpolated points
        trajectory = JointTrajectory()
        trajectory.joint_names = self.ARM_JOINTS

        # Create interpolated trajectory points
        num_points = max(2, self.interpolation_steps)
        for i in range(num_points):
            t = i / (num_points - 1)
            point = JointTrajectoryPoint()

            # Linear interpolation
            point.positions = [
                c + t * (target - c)
                for c, target in zip(current_positions, joint_positions)
            ]
            point.velocities = [0.0] * len(self.ARM_JOINTS)
            point.time_from_start = Duration(seconds=t * duration).to_msg()
            trajectory.points.append(point)

        # Ensure final point has exact target positions
        trajectory.points[-1].positions = list(joint_positions)

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

    def move_joints(self, joint_positions: List[float],
                    velocity_scaling: float = 0.5,
                    acceleration_scaling: float = 0.5,
                    wait: bool = True) -> tuple[bool, str, Optional[Pose]]:
        """Move joints to target positions."""
        # Validate positions
        valid, message = self.validate_joint_positions(joint_positions)
        if not valid:
            return False, message, None

        self.get_logger().info(f'Moving to joints: {[f"{p:.3f}" for p in joint_positions]}')

        # Execute trajectory
        success, message = self.execute_trajectory(
            joint_positions,
            velocity_scaling,
            acceleration_scaling,
            wait
        )

        # Compute FK for final pose
        final_pose = None
        if success:
            final_pose = self.compute_fk(joint_positions)

        return success, message, final_pose

    def move_to_named_position(self, name: str,
                               velocity_scaling: float = 0.5,
                               acceleration_scaling: float = 0.5,
                               wait: bool = True) -> tuple[bool, str, Optional[Pose]]:
        """Move to a named position (loaded from control_params.yaml under /**: named_positions)."""
        named = self.get_parameter('named_positions').value or {
            'init': [0.0, 0.0, 0.0, 0.0],
            'home': [0.0, -1.0, 0.7, 0.3],
        }

        if name not in named:
            available = list(named.keys()) if isinstance(named, dict) else []
            return False, f'Unknown position: {name}. Available: {available}', None

        # Convert dict form {joint1: val, ...} to ordered list for our joints
        if isinstance(named[name], dict):
            pos_list = [named[name].get(j, 0.0) for j in self.ARM_JOINTS]
        else:
            pos_list = list(named[name])

        return self.move_joints(
            pos_list,
            velocity_scaling,
            acceleration_scaling,
            wait
        )

    def _move_joints_callback(self, request: MoveJoints.Request,
                              response: MoveJoints.Response) -> MoveJoints.Response:
        """ROS 2 service handler for the documented MoveJoints API."""
        positions = list(request.joint_positions)
        self.get_logger().info(
            f'Service MoveJoints: {["{:.3f}".format(p) for p in positions]} '
            f'wait={request.wait_for_completion}'
        )

        vel = request.velocity_scaling if request.velocity_scaling > 0.0 else self.velocity_scaling
        acc = request.acceleration_scaling if request.acceleration_scaling > 0.0 else self.acceleration_scaling

        success, message, final_pose = self.move_joints(
            positions,
            vel,
            acc,
            wait=request.wait_for_completion
        )

        response.success = success
        response.message = message
        if final_pose is not None:
            response.final_pose = final_pose
        return response


def main(args=None):
    rclpy.init(args=args)

    node = FKController()

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
