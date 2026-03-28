#!/usr/bin/env python3
"""
Full control launch file for Open Manipulator X.
Launches IK, FK, and gripper controllers along with MoveIt2.

Prerequisites:
- Hardware must be started first:
    ros2 launch open_manipulator_bringup open_manipulator_x.launch.py
- Then MoveIt:
    ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
- Then this launch file
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
    ]

    # Launch configurations
    velocity_scaling = LaunchConfiguration('velocity_scaling')
    acceleration_scaling = LaunchConfiguration('acceleration_scaling')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('omx_control'),
        'config',
        'control_params.yaml'
    ])

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
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            ik_controller_node,
            fk_controller_node,
            gripper_controller_node,
        ]
    )
