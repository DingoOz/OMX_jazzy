#!/usr/bin/env python3
"""
Complete bringup launch file for Open Manipulator X.
Launches hardware, MoveIt2, and all custom controllers in the correct order.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for U2D2 (use /dev/u2d2 if udev rule is set up)'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware (no real robot)'
        ),
        DeclareLaunchArgument(
            'velocity_scaling',
            default_value='0.5',
            description='Default velocity scaling factor (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'acceleration_scaling',
            default_value='0.5',
            description='Default acceleration scaling factor (0.0-1.0)'
        ),
    ]

    # Launch configurations
    port_name = LaunchConfiguration('port_name')
    start_rviz = LaunchConfiguration('start_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    velocity_scaling = LaunchConfiguration('velocity_scaling')
    acceleration_scaling = LaunchConfiguration('acceleration_scaling')

    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('omx_control'),
        'config',
        'control_params.yaml'
    ])

    # Include hardware bringup launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'open_manipulator_x.launch.py'
            ])
        ]),
        launch_arguments={
            'port_name': port_name,
            'start_rviz': 'false',  # We'll use MoveIt's RViz
            'use_fake_hardware': use_fake_hardware,
        }.items()
    )

    # Include MoveIt launch (with delay to ensure hardware is up)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_moveit_config'),
                'launch',
                'open_manipulator_x_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'start_rviz': start_rviz,
        }.items()
    )

    # Delay MoveIt launch to ensure hardware is ready
    delayed_moveit_launch = TimerAction(
        period=3.0,
        actions=[moveit_launch]
    )

    # IK Controller node
    ik_controller_node = Node(
        package='omx_control',
        executable='ik_controller.py',
        name='ik_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'velocity_scaling': velocity_scaling,
                'acceleration_scaling': acceleration_scaling,
            }
        ],
    )

    # FK Controller node
    fk_controller_node = Node(
        package='omx_control',
        executable='fk_controller.py',
        name='fk_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'velocity_scaling': velocity_scaling,
                'acceleration_scaling': acceleration_scaling,
            }
        ],
    )

    # Gripper Controller node
    gripper_controller_node = Node(
        package='omx_control',
        executable='gripper_controller.py',
        name='gripper_controller',
        output='screen',
        parameters=[config_file],
    )

    # Delay control nodes to ensure MoveIt is ready
    delayed_controllers = TimerAction(
        period=8.0,
        actions=[
            ik_controller_node,
            fk_controller_node,
            gripper_controller_node,
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            hardware_launch,
            delayed_moveit_launch,
            delayed_controllers,
        ]
    )
