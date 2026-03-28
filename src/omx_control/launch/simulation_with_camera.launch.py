#!/usr/bin/env python3
"""
Simulation with camera launch file for Open Manipulator X.
Launches Gazebo simulation with MoveIt2, controllers, and C920 camera.
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
            description='Gazebo world file name'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'velocity_scaling',
            default_value='0.5',
            description='Default velocity scaling factor'
        ),
        DeclareLaunchArgument(
            'acceleration_scaling',
            default_value='0.5',
            description='Default acceleration scaling factor'
        ),
        # Camera arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Camera namespace'
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='Camera image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='Camera image height'
        ),
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.0',
            description='Camera X position'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='-0.3',
            description='Camera Y position'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.4',
            description='Camera Z position'
        ),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.5',
            description='Camera pitch angle'
        ),
        DeclareLaunchArgument(
            'camera_yaw',
            default_value='1.57',
            description='Camera yaw angle'
        ),
    ]

    # Launch configurations
    world = LaunchConfiguration('world')
    start_rviz = LaunchConfiguration('start_rviz')
    velocity_scaling = LaunchConfiguration('velocity_scaling')
    acceleration_scaling = LaunchConfiguration('acceleration_scaling')
    camera_name = LaunchConfiguration('camera_name')
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

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

    # Include MoveIt launch
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

    delayed_moveit_launch = TimerAction(
        period=5.0,
        actions=[moveit_launch]
    )

    # Include camera launch (real camera even in simulation for hybrid testing)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('omx_control'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'camera_x': camera_x,
            'camera_y': camera_y,
            'camera_z': camera_z,
            'camera_pitch': camera_pitch,
            'camera_yaw': camera_yaw,
        }.items()
    )

    # Control nodes
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
            camera_launch,
            delayed_controllers,
        ]
    )
