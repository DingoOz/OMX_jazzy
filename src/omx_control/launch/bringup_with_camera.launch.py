#!/usr/bin/env python3
"""
Complete bringup with camera for Open Manipulator X.
Launches hardware, MoveIt2, controllers, and C920 camera.
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
        # Robot arguments
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for U2D2'
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
        # Camera mounting position (adjust for your setup)
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.0',
            description='Camera X position relative to robot base'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='-0.3',
            description='Camera Y position relative to robot base'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.4',
            description='Camera Z position relative to robot base'
        ),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.5',
            description='Camera pitch angle (radians)'
        ),
        DeclareLaunchArgument(
            'camera_yaw',
            default_value='1.57',
            description='Camera yaw angle (radians)'
        ),
    ]

    # Launch configurations
    port_name = LaunchConfiguration('port_name')
    start_rviz = LaunchConfiguration('start_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
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
            'start_rviz': 'false',
            'use_fake_hardware': use_fake_hardware,
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
        }.items()
    )

    # Delay MoveIt launch
    delayed_moveit_launch = TimerAction(
        period=3.0,
        actions=[moveit_launch]
    )

    # Include camera launch
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

    # Delay control nodes
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
            camera_launch,
            delayed_controllers,
        ]
    )
