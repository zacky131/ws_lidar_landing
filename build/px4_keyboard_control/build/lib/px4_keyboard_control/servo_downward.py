#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ───────────── YOUR DYNAMIXEL SETTINGS ─────────────
DXL_ID                      = 14
BAUDRATE                    = 1_000_000
DEVICENAME                  = '/dev/ttyUSB0'
PROTOCOL_VERSION            = 2.0

# Control table addresses
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0

# Position limits from your script
DXL_MINIMUM_POSITION_VALUE  = 1020   # initial (0°)
DXL_MAXIMUM_POSITION_VALUE  = 2040   # 90°
DXL_THIRD_POSITION_VALUE    = 3000   # 180°

# How close is “close enough” to consider the move complete
DXL_MOVING_STATUS_THRESHOLD = 20

# Map incoming key → goal position
KEY_GOALS = {
    'a': DXL_MAXIMUM_POSITION_VALUE,
    's': DXL_THIRD_POSITION_VALUE,
    'd': DXL_MINIMUM_POSITION_VALUE,
    'q': DXL_MINIMUM_POSITION_VALUE,   # 'q' returns to initial
}


class ServoDownwardNode(Node):
    def __init__(self):
        super().__init__('servo_downward')

        # ── Match the keyboard publisher's QoS ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize port handler
        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {DEVICENAME}')
            raise RuntimeError('Port open failed')
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error(f'Failed to set baudrate to {BAUDRATE}')
            raise RuntimeError('Baudrate set failed')

        # Initialize packet handler
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Enable torque: only abort on COMM failure, warn on servo error
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'Torque enable COMM failure: {self.packet_handler.getTxRxResult(result)}'
            )
            raise RuntimeError('Torque enable COMM failure')
        if error != 0:
            self.get_logger().warn(
                f'Torque enable servo error: {self.packet_handler.getRxPacketError(error)}'
            )
        else:
            self.get_logger().info('Dynamixel torque enabled successfully')

        # Subscribe to /servo_cmd
        self.create_subscription(String, '/servo_cmd', self.servo_callback, qos)
        self.get_logger().info('ServoDownwardNode ready — listening on /servo_cmd')

    def get_goal_position(self, key: str):
        return KEY_GOALS.get(key)

    def move_to(self, goal_pos: int):
        # Send goal position
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, DXL_ID, ADDR_GOAL_POSITION, goal_pos
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Write goal COMM failure: {self.packet_handler.getTxRxResult(result)}'
            )
            return
        if error != 0:
            self.get_logger().warn(
                f'Write goal servo error: {self.packet_handler.getRxPacketError(error)}'
            )

        # Poll until close enough
        while rclpy.ok():
            result, error, present_pos = self.packet_handler.read4ByteTxRx(
                self.port_handler, DXL_ID, ADDR_PRESENT_POSITION
            )
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'Read position COMM failure: {self.packet_handler.getTxRxResult(result)}'
                )
                return
            if error != 0:
                self.get_logger().warn(
                    f'Read position servo error: {self.packet_handler.getRxPacketError(error)}'
                )
                # but continue polling
            if abs(present_pos - goal_pos) < DXL_MOVING_STATUS_THRESHOLD:
                break

        self.get_logger().info(f'Servo moved to {goal_pos}')

    def servo_callback(self, msg: String):
        key = msg.data
        goal = self.get_goal_position(key)
        if goal is None:
            self.get_logger().warn(f'Unknown servo command: "{key}"')
            return

        self.get_logger().info(f'Received "{key}", moving to {goal}')
        self.move_to(goal)

        # shutdown if 'q' was pressed
        if key == 'q':
            self.get_logger().info('Received "q" → shutting down servo node')
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        # Disable torque and close port
        self.packet_handler.write1ByteTxRx(
            self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoDownwardNode()
    except RuntimeError:
        # initialization error already logged
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
