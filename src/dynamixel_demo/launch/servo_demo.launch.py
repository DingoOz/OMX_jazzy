#!/usr/bin/env python3
"""Launch the minimal single-servo ROS 2 demo node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    dxl_id = LaunchConfiguration('dxl_id')

    declared_arguments = [
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='Serial device for the U2D2.'),
        DeclareLaunchArgument(
            'baud', default_value='57600',
            description='Bus baud rate (OMX arm uses 1000000).'),
        DeclareLaunchArgument(
            'dxl_id', default_value='1',
            description='Dynamixel ID (factory default: 1).'),
    ]

    servo_node = Node(
        package='dynamixel_demo',
        executable='servo_ros_node',
        name='servo_ros_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': ParameterValue(baud, value_type=int),
            'dxl_id': ParameterValue(dxl_id, value_type=int),
        }],
    )

    return LaunchDescription(declared_arguments + [servo_node])
