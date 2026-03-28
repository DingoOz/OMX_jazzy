#!/usr/bin/env python3
"""
Camera launch file for Logitech C920 webcam.
Launches the camera driver with TF frame publishing.
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
            'camera_name',
            default_value='camera',
            description='Name/namespace for the camera'
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path (use /dev/c920 if udev rule is set)'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='Image width (1920, 1280, or 640)'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='Image height (1080, 720, or 480)'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Camera framerate'
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='TF frame ID for the camera'
        ),
        # Camera position relative to robot base (link1)
        # Adjust these based on your actual camera mounting
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.0',
            description='Camera X position relative to link1 (meters)'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='-0.3',
            description='Camera Y position relative to link1 (meters)'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.4',
            description='Camera Z position relative to link1 (meters)'
        ),
        DeclareLaunchArgument(
            'camera_roll',
            default_value='0.0',
            description='Camera roll angle (radians)'
        ),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.5',
            description='Camera pitch angle - tilted down (radians)'
        ),
        DeclareLaunchArgument(
            'camera_yaw',
            default_value='1.57',
            description='Camera yaw angle - facing the arm (radians)'
        ),
    ]

    # Launch configurations
    camera_name = LaunchConfiguration('camera_name')
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # Camera config file
    camera_config = PathJoinSubstitution([
        FindPackageShare('omx_control'),
        'config',
        'c920_camera.yaml'
    ])

    # USB Camera node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=camera_name,
        parameters=[
            camera_config,
            {
                'video_device': video_device,
                'image_width': image_width,
                'image_height': image_height,
                'framerate': framerate,
                'camera_frame_id': camera_frame_id,
            },
        ],
        output='screen',
        remappings=[
            ('image_raw', 'image_raw'),
            ('image_raw/compressed', 'image_raw/compressed'),
            ('camera_info', 'camera_info'),
        ]
    )

    # Static TF publisher for camera position
    # Publishes camera_link relative to link1 (robot base)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', 'link1',
            '--child-frame-id', camera_frame_id,
        ],
        output='screen',
    )

    # Optional: Image rectification node (if camera is calibrated)
    # Uncomment after running camera calibration
    # rectify_node = Node(
    #     package='image_proc',
    #     executable='rectify_node',
    #     namespace=camera_name,
    #     remappings=[
    #         ('image', 'image_raw'),
    #         ('camera_info', 'camera_info'),
    #         ('image_rect', 'image_rect'),
    #     ],
    # )

    return LaunchDescription(
        declared_arguments + [
            camera_node,
            camera_tf_node,
        ]
    )
