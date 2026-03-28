#!/usr/bin/env python3
"""Launch RViz2 with MoveIt parameters for connecting to an already-running move_group."""

from pathlib import Path

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='open_manipulator_x', package_name='open_manipulator_moveit_config')
        .robot_description_semantic(
            str(Path('config') / 'open_manipulator_x' / 'open_manipulator_x.srdf'))
        .joint_limits(str(Path('config') / 'open_manipulator_x' / 'joint_limits.yaml'))
        .trajectory_execution(
            str(Path('config') / 'open_manipulator_x' / 'moveit_controllers.yaml'))
        .robot_description_kinematics(
            str(Path('config') / 'open_manipulator_x' / 'kinematics.yaml'))
        .to_moveit_configs()
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('open_manipulator_moveit_config'), 'config', 'moveit.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([rviz_node])
