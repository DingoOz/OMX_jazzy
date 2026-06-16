#!/usr/bin/env python3
"""
Demo 1 -- Direct single-servo control with the DynamixelSDK (no ROS).

A minimal, hardened distillation of the ROBOTIS ``read_write.py`` example. It
opens the serial port, pings one servo to confirm wiring, enables torque, sweeps
the horn between two goal positions while printing the present position, then
disables torque and closes the port cleanly.

Targets a Protocol 2.0 X-series servo (XL430 / XM430, as used on the Open
Manipulator X). Defaults assume a *factory-fresh* servo: ID 1 at 57600 baud.
Note that the assembled OMX arm runs its bus at 1 Mbps (``--baud 1000000``).

Run directly::

    python3 single_servo_demo.py --port /dev/ttyUSB0 --id 1 --baud 57600

or, after building the workspace::

    ros2 run dynamixel_demo single_servo_demo --port /dev/ttyUSB0
"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# ---------------------------------------------------------------------------
# X-series (XL430 / XM430) Protocol 2.0 control table.
# See src/DynamixelSDK/python/tests/protocol2_0/read_write.py
# ---------------------------------------------------------------------------
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# The X-series has a 12-bit absolute encoder: 0..4095 over a full revolution.
DXL_MIN_POSITION = 1024     # ~90 deg one way of centre
DXL_MAX_POSITION = 3072     # ~90 deg the other way
DXL_MOVING_THRESHOLD = 20   # raw units; "close enough" to the goal


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Sweep a single Dynamixel X-series servo using the DynamixelSDK.')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial device for the U2D2 (default: /dev/ttyUSB0).')
    parser.add_argument('--baud', type=int, default=57600,
                        help='Bus baud rate. Factory default is 57600; the OMX '
                             'arm uses 1000000 (default: 57600).')
    parser.add_argument('--id', type=int, default=1, dest='dxl_id',
                        help='Dynamixel ID (factory default: 1).')
    parser.add_argument('--cycles', type=int, default=4,
                        help='Number of goal-position changes to perform '
                             '(default: 4).')
    parser.add_argument('--min', type=int, default=DXL_MIN_POSITION, dest='pos_min',
                        help='Lower goal position, raw 0..4095 (default: %(default)s).')
    parser.add_argument('--max', type=int, default=DXL_MAX_POSITION, dest='pos_max',
                        help='Upper goal position, raw 0..4095 (default: %(default)s).')
    return parser.parse_args(argv)


def _check(packet_handler, comm_result, dxl_error, what):
    """Return True on success; print a decoded SDK error and return False."""
    if comm_result != COMM_SUCCESS:
        print('  [FAIL] %s: %s' % (what, packet_handler.getTxRxResult(comm_result)))
        return False
    if dxl_error != 0:
        print('  [WARN] %s: %s' % (what, packet_handler.getRxPacketError(dxl_error)))
        return False
    return True


def run_demo(args):
    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print('[FAIL] Could not open port %s. Is the U2D2 plugged in and are you '
              'in the "dialout" group?' % args.port)
        return 1
    print('[ OK ] Opened port %s' % args.port)

    if not port_handler.setBaudRate(args.baud):
        print('[FAIL] Could not set baud rate %d.' % args.baud)
        port_handler.closePort()
        return 1
    print('[ OK ] Baud rate set to %d' % args.baud)

    torque_enabled = False
    try:
        # 1) Ping -- confirms the servo is wired, powered and at the expected ID/baud.
        model_number, comm_result, dxl_error = packet_handler.ping(
            port_handler, args.dxl_id)
        if not _check(packet_handler, comm_result, dxl_error, 'ping'):
            print('[FAIL] No response from ID %d at %d baud. Check power, the ID, '
                  'and the baud rate.' % (args.dxl_id, args.baud))
            return 1
        print('[ OK ] Found Dynamixel ID %d (model number %d)'
              % (args.dxl_id, model_number))

        # 2) Enable torque so the servo will hold and move to goal positions.
        comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, args.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if not _check(packet_handler, comm_result, dxl_error, 'torque enable'):
            return 1
        torque_enabled = True
        print('[ OK ] Torque enabled -- the servo is now holding position\n')

        # 3) Sweep between the two goal positions.
        goals = [args.pos_min, args.pos_max]
        for cycle in range(args.cycles):
            goal = goals[cycle % 2]
            comm_result, dxl_error = packet_handler.write4ByteTxRx(
                port_handler, args.dxl_id, ADDR_GOAL_POSITION, goal)
            if not _check(packet_handler, comm_result, dxl_error, 'write goal'):
                return 1

            # Poll present position until the servo reaches the goal.
            while True:
                present, comm_result, dxl_error = packet_handler.read4ByteTxRx(
                    port_handler, args.dxl_id, ADDR_PRESENT_POSITION)
                if not _check(packet_handler, comm_result, dxl_error, 'read position'):
                    break
                print('  [ID:%03d] cycle %d/%d  GoalPos:%4d  PresPos:%4d'
                      % (args.dxl_id, cycle + 1, args.cycles, goal, present))
                if abs(goal - present) <= DXL_MOVING_THRESHOLD:
                    break
                time.sleep(0.05)

        print('\n[ OK ] Sweep complete.')
        return 0

    finally:
        # Always release torque and close the port, even on error / Ctrl-C.
        if torque_enabled:
            packet_handler.write1ByteTxRx(
                port_handler, args.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            print('[ OK ] Torque disabled -- the horn now turns freely.')
        port_handler.closePort()
        print('[ OK ] Port closed.')


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        return run_demo(args)
    except KeyboardInterrupt:
        print('\nInterrupted by user.')
        return 130


if __name__ == '__main__':
    sys.exit(main())
