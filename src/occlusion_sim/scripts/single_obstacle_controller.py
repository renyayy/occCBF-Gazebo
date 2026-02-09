#!/usr/bin/env python3
"""Single Obstacle Controller - Chases ego robot"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class SingleObstacleController(Node):
    def __init__(self):
        super().__init__('single_obstacle_controller')

        self.v_max = 0.5
        self.obs_name = 'obs_0'

        self.ego_x = None
        self.ego_y = None
        self.obs_state = None

        self.create_subscription(Odometry, '/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, f'/{self.obs_name}/odom', self.obs_odom_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, f'/{self.obs_name}/cmd_vel', 10)
        self.state_pub = self.create_publisher(Odometry, '/obstacle/state', 10)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Single Obstacle Controller started (chase mode)')

    def ego_odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

    def obs_odom_cb(self, msg):
        self.obs_state = msg

    def control_loop(self):
        if self.ego_x is None or self.obs_state is None:
            return

        ox = self.obs_state.pose.pose.position.x
        oy = self.obs_state.pose.pose.position.y

        dx = self.ego_x - ox
        dy = self.ego_y - oy
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            vx, vy = 0.0, 0.0
        else:
            vx = self.v_max * dx / dist
            vy = self.v_max * dy / dist

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.cmd_pub.publish(cmd)

        state = Odometry()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'world'
        state.child_frame_id = self.obs_name
        state.pose.pose = self.obs_state.pose.pose
        state.twist.twist.linear.x = vx
        state.twist.twist.linear.y = vy
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SingleObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
