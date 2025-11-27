#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import os
import signal

class SimulationSupervisor(Node):
    def __init__(self):
        super().__init__('simulation_supervisor')
        
        # Subscriber for TurtleBot's odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom', # Assuming TurtleBot's odom topic is /odom
            self.odom_callback,
            10)
        
        # Goal and state
        self.goal_pos = [0.0, -4.0]
        self.threshold = 0.1  # Distance in meters to consider the goal reached
        self.goal_reached = False

        self.get_logger().info('Simulation Supervisor started.')
        self.get_logger().info(f'Monitoring for goal: {self.goal_pos}')

    def odom_callback(self, msg):
        """Callback to check TurtleBot's position."""
        if self.goal_reached:
            return

        current_pos = msg.pose.pose.position
        
        # Calculate distance to goal
        dist_to_goal = math.sqrt(
            (self.goal_pos[0] - current_pos.x)**2 + 
            (self.goal_pos[1] - current_pos.y)**2
        )

        # Check if goal is reached
        if dist_to_goal < self.threshold:
            self.goal_reached = True
            self.get_logger().info('*** GOAL REACHED! ***')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
