#!/usr/bin/env python3
"""
Demo 2 -- Minimal ROS 2 control of a single Dynamixel servo.

A small, self-contained ``rclpy`` node that opens the serial port itself with the
DynamixelSDK (no MoveIt, no ros2_control) and exposes two simple topics:

    Subscribes:  /dxl/goal_position    std_msgs/Float64   target angle [rad]
    Publishes:   /dxl/present_position std_msgs/Float64   measured angle [rad]

Angles are referenced to the X-series encoder centre (raw value 2048), so 0.0 rad
is the mid-point and the usable range is roughly +/- pi.

Run::

    ros2 launch dynamixel_demo servo_demo.launch.py port:=/dev/ttyUSB0
    # then, in other terminals:
    ros2 topic pub --once /dxl/goal_position std_msgs/msg/Float64 "{data: 0.5}"
    ros2 topic echo /dxl/present_position
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# X-series Protocol 2.0 control table (see single_servo_demo.py).
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 12-bit absolute encoder: 4096 raw units per revolution, centred at 2048.
TICKS_PER_REV = 4096
CENTER_TICK = 2048


def rad_to_tick(rad):
    """Convert an angle in radians (0 = encoder centre) to a raw 0..4095 tick."""
    tick = int(round(rad / (2.0 * math.pi) * TICKS_PER_REV)) + CENTER_TICK
    return max(0, min(TICKS_PER_REV - 1, tick))


def tick_to_rad(tick):
    """Convert a raw tick to an angle in radians (0 = encoder centre)."""
    return (tick - CENTER_TICK) / TICKS_PER_REV * (2.0 * math.pi)


class ServoRosNode(Node):
    """Bridge a single Dynamixel servo to a pair of ROS 2 topics."""

    def __init__(self):
        super().__init__('servo_ros_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('dxl_id', 1)
        self.declare_parameter('publish_rate', 10.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.dxl_id = self.get_parameter('dxl_id').value
        publish_rate = self.get_parameter('publish_rate').value

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.torque_enabled = False

        if not self._connect():
            raise RuntimeError('Failed to connect to the Dynamixel servo.')

        self.present_pub = self.create_publisher(
            Float64, '/dxl/present_position', 10)
        self.goal_sub = self.create_subscription(
            Float64, '/dxl/goal_position', self._goal_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_present)

        self.get_logger().info(
            'Ready. Publish a target on /dxl/goal_position (rad), '
            'watch /dxl/present_position.')

    def _connect(self):
        if not self.port_handler.openPort():
            self.get_logger().error(
                'Could not open port %s (in the "dialout" group?).' % self.port)
            return False
        if not self.port_handler.setBaudRate(self.baud):
            self.get_logger().error('Could not set baud rate %d.' % self.baud)
            return False

        model, comm_result, dxl_error = self.packet_handler.ping(
            self.port_handler, self.dxl_id)
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error(
                'No response from ID %d at %d baud. Check power, ID and baud.'
                % (self.dxl_id, self.baud))
            return False
        self.get_logger().info(
            'Connected to Dynamixel ID %d (model %d) on %s @ %d baud.'
            % (self.dxl_id, model, self.port, self.baud))

        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error('Could not enable torque.')
            return False
        self.torque_enabled = True
        return True

    def _goal_callback(self, msg):
        tick = rad_to_tick(msg.data)
        comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, tick)
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().warning(
                'Failed to write goal %.3f rad (tick %d).' % (msg.data, tick))
        else:
            self.get_logger().info(
                'Goal set: %.3f rad (tick %d).' % (msg.data, tick))

    def _publish_present(self):
        tick, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            return
        self.present_pub.publish(Float64(data=tick_to_rad(tick)))

    def shutdown(self):
        """Release torque and close the port."""
        if self.torque_enabled:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if self.port_handler.is_open:
            self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoRosNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if isinstance(exc, RuntimeError):
            print('Startup error: %s' % exc)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
