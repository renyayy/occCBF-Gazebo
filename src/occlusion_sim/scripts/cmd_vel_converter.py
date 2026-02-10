#!/usr/bin/env python3
"""DI velocity (vx, vy) to Unicycle command (v, omega) converter"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')

        self.declare_parameter('max_v', 0.22)
        self.declare_parameter('max_w', 2.84)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('stop_threshold_angle', 0.5)

        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.stop_threshold_angle = self.get_parameter('stop_threshold_angle').value

        self.theta = 0.0
        self.odom_received = False

        self.create_subscription(Twist, '/di_cmd_vel', self.di_cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('CmdVel Converter started')

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_received = True

    def di_cmd_cb(self, msg):
        if not self.odom_received:
            return

        vx, vy = msg.linear.x, msg.linear.y
        speed = math.hypot(vx, vy)

        cmd = Twist()
        if speed < 0.01:
            self.cmd_pub.publish(cmd)
            return

        theta_target = math.atan2(vy, vx)
        angle_error = math.atan2(
            math.sin(theta_target - self.theta),
            math.cos(theta_target - self.theta))

        omega = max(-self.max_w, min(self.max_w, self.kp_ang * angle_error))

        if abs(angle_error) > self.stop_threshold_angle:
            cmd.angular.z = omega
        else:
            cmd.linear.x = min(speed, self.max_v)
            cmd.angular.z = omega

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
