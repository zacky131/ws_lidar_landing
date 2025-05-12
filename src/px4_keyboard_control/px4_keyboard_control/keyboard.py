#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
import sys
import termios
import tty
from std_msgs.msg import String

class PX4KeyboardHandler(Node):
    def __init__(self):
        super().__init__('px4_keyboard_handler')

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.position_cmd_publisher = self.create_publisher(PoseStamped, '/position_cmd', qos_profile)
        self.keyboard_cmd_publisher = self.create_publisher(String, '/keyboard_cmd', qos_profile)
        self.velocity_cmd_publisher = self.create_publisher(Twist, '/velocity_cmd', qos_profile)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_velocity_command(self, x, y, z, th):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th
        self.velocity_cmd_publisher.publish(twist)

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)

        # keyboard command handler. will be publihed to /keyboard_command topic
        keyboard_command = String()

        # toggle switch (velocity or position mode)
        vel_pos_toggle = "position"

        # velocity handler
        x_val = 0.0
        y_val = 0.0
        z_val = 0.0
        yaw_val = 0.0
        speed = 0.5
        turn = 0.2

        try:
            while rclpy.ok():
                key = self.get_key()

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "world"
                pose_stamped.pose.orientation.x = 0.0
                pose_stamped.pose.orientation.y = 0.0
                pose_stamped.pose.orientation.z = 0.0
                pose_stamped.pose.orientation.w = 1.0

                if key == 'w' and vel_pos_toggle == 'position':
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.position.y = 0.0
                    pose_stamped.pose.position.z = -1.0
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A move UPWARD request has been sent.')
                elif key == 's' and vel_pos_toggle == 'position':
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.position.y = 0.0
                    pose_stamped.pose.position.z = 1.0
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A move DOWNWARD request has been sent.')
                elif key == 'a' and vel_pos_toggle == 'position':
                    pose_stamped.pose.orientation.z = -0.1
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A YAW COUNTER-CLOCKWISE request has been sent.')
                elif key == 'd' and vel_pos_toggle == 'position':
                    pose_stamped.pose.orientation.z = 0.1
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A YAW CLOCKWISE request has been sent.')
                elif key == 'i' and vel_pos_toggle == 'position':
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.position.y = 1.0
                    pose_stamped.pose.position.z = 0.0
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A move FORWARD request has been sent.')
                elif key == 'k' and vel_pos_toggle == 'position':
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.position.y = -1.0
                    pose_stamped.pose.position.z = 0.0
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A move BACKWARD request has been sent.')
                elif key == 'j' and vel_pos_toggle == 'position':
                    pose_stamped.pose.position.x = 1.0
                    pose_stamped.pose.position.y = 0.0
                    pose_stamped.pose.position.z = 0.0
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A move LEFTWARD request has been sent.')
                elif key == 'l' and vel_pos_toggle == 'position':
                    pose_stamped.pose.position.x = -1.0
                    pose_stamped.pose.position.y = 0.0
                    pose_stamped.pose.position.z = 0.0
                    self.position_cmd_publisher.publish(pose_stamped)
                    self.get_logger().info('A move UPWARD request has been sent.')
                elif key == 'w' and vel_pos_toggle == 'velocity':
                    z_val = z_val + speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 's' and vel_pos_toggle == 'velocity':
                    z_val = z_val - speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'a' and vel_pos_toggle == 'velocity':
                    yaw_val = yaw_val - turn
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'd' and vel_pos_toggle == 'velocity':
                    yaw_val = yaw_val + turn
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'i' and vel_pos_toggle == 'velocity':
                    y_val = y_val + speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'k' and vel_pos_toggle == 'velocity':
                    y_val = y_val - speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'j' and vel_pos_toggle == 'velocity':
                    x_val = x_val - speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'l' and vel_pos_toggle == 'velocity':
                    x_val = x_val + speed
                    self.publish_velocity_command(x_val, y_val, z_val, yaw_val)
                    self.get_logger().info(f'A velocity command sent with: x = {x_val}; y = {y_val}; z = {z_val}; yaw = {yaw_val}')
                elif key == 'o':
                    keyboard_command.data = "OFFBOARD"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A request to change to OFFBOARD mode has been sent.')
                elif key == 'p':
                    keyboard_command.data = "POS"
                    vel_pos_toggle = "position"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A request to change to POSITION CONTROL mode has been sent.')
                elif key == 'v':
                    keyboard_command.data = "VEL"
                    vel_pos_toggle = "velocity"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A request to change to VELOCITY CONTROL mode has been sent.')
                elif key == 't':
                    keyboard_command.data = "ARM"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('An ARM request has been sent.')
                elif key == 'y':
                    keyboard_command.data = "DISARM"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A DISARM request has been sent.')
                elif key == 'b':
                    keyboard_command.data = "BREAK"
                    x_val = 0.0
                    y_val = 0.0
                    z_val = 0.0
                    yaw_val = 0.0
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A BREAK request has been sent.')
                elif key == 'f':
                    keyboard_command.data = "FAST-PLANNER"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A request to change to PLANNER CONTROL mode has been sent.')
                elif key == 'g':
                    keyboard_command.data = "LAND"
                    vel_pos_toggle = "position"
                    self.keyboard_cmd_publisher.publish(keyboard_command)
                    self.get_logger().info('A request to change to LAND mode has been sent.')
                elif key == 'q':
                    self.get_logger().info('Exiting node.')
                    break
                else:
                    self.get_logger().info(f'Unhandled key: {key}')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
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