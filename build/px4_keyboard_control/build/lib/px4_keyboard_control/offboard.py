#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point
from std_msgs.msg import String
from quadrotor_msgs.msg import PositionCommand
import math
import numpy as np


class OffboardControl(Node):

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.setpoint_target_subscriber = self.create_subscription(
            PoseStamped, '/position_cmd', self.position_command_callback, qos_profile)
        self.setpoint_target_subscriber = self.create_subscription(
            Twist, '/velocity_cmd', self.velocity_command_callback, qos_profile)
        self.keyboard_command_subscriber = self.create_subscription(
            String, '/keyboard_cmd', self.keyboard_command_callback, qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.fastplanner_sub = self.create_subscription(
            PositionCommand, '/planning/pos_cmd', self.fastplanner_callback, qos_profile2)

        # Initialize variables for Position Control Mode
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0
        self.setpoint_position = PoseStamped()
        self.setpoint_position.pose.position.x = self.vehicle_local_position.x
        self.setpoint_position.pose.position.y = self.vehicle_local_position.y
        self.setpoint_position.pose.position.z = self.takeoff_height
        self.setpoint_position.pose.orientation.z = 1.57079
        self.keyboard_command = ""
        self.control_mode = "POSITION"

        # Initialize variables for Velocity Control Mode
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0

        # Fast planner params
        self.fp_position = Point()
        self.fp_position.z = -1.0
        self.fp_velocity = Vector3()
        self.fp_acceleration = Vector3()
        self.fp_yaw = 1.57079
        self.fp_yaw_dot = 0.0

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def position_command_callback(self, msg):
        self.setpoint_position.pose.position.x += msg.pose.position.x
        self.setpoint_position.pose.position.y += msg.pose.position.y
        self.setpoint_position.pose.position.z += msg.pose.position.z

        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.setpoint_position.pose.orientation.z += yaw

    def velocity_command_callback(self, msg):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def keyboard_command_callback(self, msg):
        self.keyboard_command = msg.data

    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    def fastplanner_callback(self, msg):
        self.get_logger().info(f"Fast planner callback {[msg.position.x, msg.position.y, msg.position.z]} and yaw {self.setpoint_position.pose.orientation.z}")   
        self.fp_position.x = msg.position.x
        self.fp_position.y = -msg.position.y
        self.fp_position.z = -msg.position.z
        # self.fp_velocity = msg.velocity
        # self.fp_acceleration = msg.acceleration
        self.fp_yaw = -msg.yaw
        self.fp_yaw_dot = msg.yaw_dot

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
    
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True if self.control_mode == "POSITION" else False if self.control_mode == "VELOCITY" else True
        msg.velocity = True if self.control_mode == "VELOCITY" else False
        # msg.acceleration = True if self.control_mode == "FAST-PLANNER" else False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        # cos_yaw = np.cos(self.trueYaw)
        # sin_yaw = np.sin(self.trueYaw)
        # world_x = (x * cos_yaw - y * sin_yaw)
        # world_y = (y * sin_yaw + y * cos_yaw)

        msg = TrajectorySetpoint()
        msg.position = [self.setpoint_position.pose.position.x, self.setpoint_position.pose.position.y, self.setpoint_position.pose.position.z]
        msg.yaw = self.setpoint_position.pose.orientation.z
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[self.setpoint_position.pose.position.x, self.setpoint_position.pose.position.y, self.setpoint_position.pose.position.z]} and yaw {self.setpoint_position.pose.orientation.z}")

    def publish_velocity_setpoint(self):
        # Compute velocity in the world frame
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
        velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

        # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = velocity_world_x
        trajectory_msg.velocity[1] = velocity_world_y
        trajectory_msg.velocity[2] = self.velocity.z
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = self.yaw
        self.get_logger().info(f"Publishing velocity setpoints {[velocity_world_x, velocity_world_y, self.velocity.z]} and yaw {self.yaw}")

        self.trajectory_setpoint_publisher.publish(trajectory_msg)
    
    def publish_fastplanner_setpoint(self):
        # cos_yaw = np.cos(self.trueYaw)
        # sin_yaw = np.sin(self.trueYaw)
        # world_x = (x * cos_yaw - y * sin_yaw)
        # world_y = (y * sin_yaw + y * cos_yaw)

        msg = TrajectorySetpoint()
        msg.position = [self.fp_position.x, self.fp_position.y, self.fp_position.z]
        # msg.velocity = [self.fp_velocity.x, self.fp_velocity.y, self.fp_velocity.z]
        # msg.acceleration = [self.fp_acceleration.x, self.fp_acceleration.y, self.fp_acceleration.z]
        msg.yaw = self.fp_yaw
        msg.yawspeed = self.fp_yaw_dot
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[msg.position[0], msg.position[1], msg.position[2]]} and yaw {msg.yaw}")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter >= 10 and self.keyboard_command == "OFFBOARD":
            self.engage_offboard_mode()
        elif self.keyboard_command == "ARM":
            self.arm()
        elif self.keyboard_command == "DISARM":
            self.disarm()
        elif self.keyboard_command == "LAND":
            self.land()
        elif self.keyboard_command == "POS":
            self.control_mode = "POSITION"
            self.setpoint_position.pose.position.x = self.vehicle_local_position.x
            self.setpoint_position.pose.position.y = self.vehicle_local_position.y
            self.setpoint_position.pose.position.z = self.vehicle_local_position.z
            self.setpoint_position.pose.orientation.z = self.vehicle_local_position.heading
        elif self.keyboard_command == "VEL":
            self.control_mode = "VELOCITY"
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            self.velocity.z = 0.0
        elif self.keyboard_command == "BREAK":
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            self.velocity.z = 0.0
        elif self.keyboard_command == "FAST-PLANNER":
            self.control_mode = "FAST-PLANNER"
        self.keyboard_command = ""

        if (self.vehicle_local_position.x != self.setpoint_position.pose.position.x or
            self.vehicle_local_position.y != self.setpoint_position.pose.position.y or
            self.vehicle_local_position.z != self.setpoint_position.pose.position.z or
            self.vehicle_local_position.heading != self.setpoint_position.pose.orientation.z) and \
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.control_mode == "POSITION":
                self.publish_position_setpoint()
            elif self.control_mode == "VELOCITY":
                self.publish_velocity_setpoint()
            elif self.control_mode == "FAST-PLANNER":
                self.publish_fastplanner_setpoint()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
