#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        
        # Subscriber for odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/obstacle/odom',
            self.odom_callback,
            10)
        
        # Timer for the main control loop
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        # Target and current state
        self.start_pos = [-2.0, -2.0]
        self.target_pos = [2.0, -2.0]
        self.current_pos = None
        self.is_moving = True
        self.speed = 0.1  # Linear speed in m/s
        self.tolerance = 0.1 # Distance tolerance to stop

        self.get_logger().info('Obstacle Controller Started')
        self.get_logger().info(f'Moving from {self.start_pos} to {self.target_pos}')

    def odom_callback(self, msg):
        """Callback to update the current position from odometry."""
        self.current_pos = msg.pose.pose.position

    def control_loop(self):
        """Main control loop to move the obstacle."""
        if self.current_pos is None or not self.is_moving:
            return

        # Calculate vector to target
        dx = self.target_pos[0] - self.current_pos.x
        dy = self.target_pos[1] - self.current_pos.y
        
        # Calculate distance to target
        distance = math.sqrt(dx**2 + dy**2)

        # Check if the target is reached
        if distance < self.tolerance:
            self.is_moving = False
            self.get_logger().info('Target reached.')
            # Send a final stop command
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            return

        # Normalize the direction vector
        vx = (dx / distance) * self.speed
        vy = (dy / distance) * self.speed
        
        # Create and publish the Twist message
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the obstacle stops when the node is shut down
        stop_msg = Twist()
        # The publisher might be gone if shutdown is abrupt
        if node.publisher_ and node.publisher_.is_valid:
            node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()