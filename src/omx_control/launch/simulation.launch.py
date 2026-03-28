#!/usr/bin/env python3
"""
Simulation launch file for Open Manipulator X.
Launches Gazebo Harmonic simulation with MoveIt2 and all custom controllers.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value='empty_world',
            description='Gazebo world file name (without .sdf extension)'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
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
    world = LaunchConfiguration('world')
    start_rviz = LaunchConfiguration('start_rviz')
    velocity_scaling = LaunchConfiguration('velocity_scaling')
    acceleration_scaling = LaunchConfiguration('acceleration_scaling')

    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('omx_control'),
        'config',
        'control_params.yaml'
    ])

    # Include Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'open_manipulator_x_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
        }.items()
    )

    # Include MoveIt launch (with delay to ensure Gazebo is up)
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
            'use_sim': 'true',
        }.items()
    )

    # Delay MoveIt launch to ensure Gazebo and controllers are ready
    delayed_moveit_launch = TimerAction(
        period=5.0,
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
                'use_sim_time': True,
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
                'use_sim_time': True,
            }
        ],
    )

    # Gripper Controller node
    gripper_controller_node = Node(
        package='omx_control',
        executable='gripper_controller.py',
        name='gripper_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
            }
        ],
    )

    # Delay control nodes to ensure MoveIt is ready
    delayed_controllers = TimerAction(
        period=10.0,
        actions=[
            ik_controller_node,
            fk_controller_node,
            gripper_controller_node,
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            delayed_moveit_launch,
            delayed_controllers,
        ]
    )
