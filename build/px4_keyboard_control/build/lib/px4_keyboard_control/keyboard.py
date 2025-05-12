#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

import sys
import termios
import tty
import select
import time
import math

class PX4KeyboardHandler(Node):
    def __init__(self):
        super().__init__('px4_keyboard_handler')

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1
        )

        self.position_cmd_publisher = self.create_publisher(PoseStamped, '/position_cmd', qos_profile)
        self.keyboard_cmd_publisher = self.create_publisher(String,      '/keyboard_cmd',  qos_profile)
        self.velocity_cmd_publisher = self.create_publisher(Twist,       '/velocity_cmd',  qos_profile)
        self.servo_cmd_publisher    = self.create_publisher(String,      '/servo_cmd',     qos_profile)

    def get_key(self, timeout=0.1):
        """Non‐blocking key reader with timeout in seconds."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_velocity_command(self, x, y, z, th):
        twist = Twist()
        twist.linear.x  = x
        twist.linear.y  = y
        twist.linear.z  = z
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th
        self.velocity_cmd_publisher.publish(twist)

    def run(self):
        # save terminal settings and initialize state
        self.settings = termios.tcgetattr(sys.stdin)
        keyboard_command = String()
        vel_pos_toggle   = "position"
        x_val = y_val = z_val = yaw_val = 0.0
        speed = 0.5
        turn  = math.pi / 2

        try:
            while rclpy.ok():
                key = self.get_key()

                # prepare a reusable PoseStamped
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "world"
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                # ──────────────── POSITION MODE ────────────────
                if key == 'w' and vel_pos_toggle == 'position':
                    pose.pose.position.z = -1.0
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A move UPWARD request has been sent.')
                elif key == 's' and vel_pos_toggle == 'position':
                    pose.pose.position.z = 1.0
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A move DOWNWARD request has been sent.')
                elif key == 'a' and vel_pos_toggle == 'position':
                    pose.pose.orientation.z = -0.1
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A YAW COUNTER-CLOCKWISE request has been sent.')
                elif key == 'd' and vel_pos_toggle == 'position':
                    pose.pose.orientation.z = 0.1
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A YAW CLOCKWISE request has been sent.')
                elif key == 'i' and vel_pos_toggle == 'position':
                    pose.pose.position.y = 1.0
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A move FORWARD request has been sent.')
                elif key == 'k' and vel_pos_toggle == 'position':
                    pose.pose.position.y = -1.0
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A move BACKWARD request has been sent.')
                elif key == 'j' and vel_pos_toggle == 'position':
                    pose.pose.position.x = 1.0
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A move LEFTWARD request has been sent.')
                elif key == 'l' and vel_pos_toggle == 'position':
                    pose.pose.position.x = -1.0
                    self.position_cmd_publisher.publish(pose)
                    self.get_logger().info('A move RIGHTWARD request has been sent.')

                # ──────────────── VELOCITY MODE ────────────────
                elif key == 'w' and vel_pos_toggle == 'velocity':
                    z_val += speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 's' and vel_pos_toggle == 'velocity':
                    z_val -= speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 'a' and vel_pos_toggle == 'velocity':
                    yaw_val -= turn
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 'd' and vel_pos_toggle == 'velocity':
                    yaw_val += turn
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 'i' and vel_pos_toggle == 'velocity':
                    y_val += speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 'k' and vel_pos_toggle == 'velocity':
                    y_val -= speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 'j' and vel_pos_toggle == 'velocity':
                    x_val -= speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')
                elif key == 'l' and vel_pos_toggle == 'velocity':
                    x_val += speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'Velocity → x:{x_val}, y:{y_val}, z:{z_val}, yaw:{yaw_val}')

                # ──────────────── MODE SWITCH & ARM/DISARM ────────────────
                elif key == 'o':
                    keyboard_command.data = "OFFBOARD"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('OFFBOARD mode requested.')
                elif key == 'p':
                    keyboard_command.data = "POS"
                    vel_pos_toggle = "position"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('POSITION CONTROL mode requested.')
                elif key == 'v':
                    keyboard_command.data = "VEL"
                    vel_pos_toggle = "velocity"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('VELOCITY CONTROL mode requested.')
                elif key == 't':
                    keyboard_command.data = "ARM"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('ARM command sent.')
                elif key == 'y':
                    keyboard_command.data = "DISARM"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('DISARM command sent.')
                elif key == 'b':
                    keyboard_command.data = "BREAK"
                    x_val = y_val = z_val = yaw_val = 0.0
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('BREAK command sent.')
                elif key == 'f':
                    keyboard_command.data = "FAST-PLANNER"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('PLANNER CONTROL mode requested.')
                elif key == 'g':
                    keyboard_command.data = "LAND"
                    vel_pos_toggle = "position"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('LAND command sent.')

                # ──────────────── NEW: Servo + 360° spin ────────────────
                elif key == '9':
                   # 0) make sure offboard node enters VELOCITY mode
                    kb = String(data='VEL')
                    self.keyboard_cmd_publisher.publish(kb)
                    self.get_logger().info('Temporarily switching to VELOCITY mode')
                    time.sleep(0.1)

                    # 1) trigger the servo “a”
                    servo_msg = String(data='a')
                    self.servo_cmd_publisher.publish(servo_msg)
                    self.get_logger().info('Servo command "a" sent')

                    # 2) wait 2 seconds
                    time.sleep(2.0)

                    # full‐spin time at π/2 rad/s → 4 s
                    duration = 2 * math.pi / turn
                    start_t = time.time()
                    while rclpy.ok() and (time.time() - start_t < duration):
                        self.publish_velocity_command(0.0, 0.0, 0.0, turn)
                        time.sleep(0.02)
                    # stop spinning
                    self.publish_velocity_command(0.0, 0.0, 0.0, 0.0)
                    self.get_logger().info('Finished 360° spin')

                    # 4) (optional) switch back to POSITION mode
                    kb = String(data='POS')
                    self.keyboard_cmd_publisher.publish(kb)
                    self.get_logger().info('Restoring POSITION mode')
                # ──────────────── EXIT ────────────────
                elif key == 'q':
                    self.get_logger().info('Exiting node.')
                    break
                else:
                    if key:
                        self.get_logger().info(f'Unhandled key: {key}')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = PX4KeyboardHandler()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
