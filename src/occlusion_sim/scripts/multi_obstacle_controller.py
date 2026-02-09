#!/usr/bin/env python3
"""Multi Obstacle Controller - Random walk behavior matching safe_control/dynamic_env"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import random
import math


class MultiObstacleController(Node):
    def __init__(self):
        super().__init__('multi_obstacle_controller')
        
        # 移動範囲制限 (safe_control/dynamic_env準拠: 24x15m)
        self.X_MIN, self.X_MAX = 0.0, 24.0
        self.Y_MIN, self.Y_MAX = 1.0, 14.0
        
        random.seed(42)
        self.obstacles = {
            'obs_0': {'mode': 1, 'v_max': 0.5, 'theta': random.uniform(-math.pi, math.pi),
                      'state': None},
            'obs_1': {'mode': 1, 'v_max': 0.5, 'theta': random.uniform(-math.pi, math.pi),
                      'state': None},
            'obs_2': {'mode': 1, 'v_max': 0.5, 'theta': random.uniform(-math.pi, math.pi),
                      'state': None},
            'obs_3': {'mode': 1, 'v_max': 0.5, 'theta': random.uniform(-math.pi, math.pi),
                      'state': None},
            'obs_4': {'mode': 1, 'v_max': 0.5, 'theta': random.uniform(-math.pi, math.pi),
                      'state': None},
        }
        self.radius = 0.25
        
        for name in self.obstacles:
            self.create_subscription(Odometry, f'/{name}/odom', 
                                     lambda msg, n=name: self.odom_cb(msg, n), 10)
        
        self.cmd_pubs = {n: self.create_publisher(Twist, f'/{n}/cmd_vel', 10) for n in self.obstacles}
        self.state_pub = self.create_publisher(Odometry, '/obstacle/state', 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Multi Obstacle Controller started')

    def odom_cb(self, msg, name):
        self.obstacles[name]['state'] = msg

    def control_loop(self):
        for name, obs in self.obstacles.items():
            if obs['state'] is None:
                continue
            
            x = obs['state'].pose.pose.position.x
            y = obs['state'].pose.pose.position.y
            theta = obs['theta']
            v_max = obs['v_max']
            
            # Random walk: 5% chance of large turn (matches safe_control/dynamic_env)
            if obs['mode'] == 1 and random.random() < 0.05:
                theta += random.gauss(0.0, 0.2)
                obs['theta'] = theta
            
            vx = v_max * math.cos(theta)
            vy = v_max * math.sin(theta)

            # Predict next position
            dt = 0.05
            x_new = x + vx * dt
            y_new = y + vy * dt

            # Y boundary reflection (only when predicted to cross AND moving toward boundary)
            if y_new >= self.Y_MAX and vy > 0:
                obs['theta'] = -obs['theta']
                vx = v_max * math.cos(obs['theta'])
                vy = v_max * math.sin(obs['theta'])
            elif y_new <= self.Y_MIN and vy < 0:
                obs['theta'] = -obs['theta']
                vx = v_max * math.cos(obs['theta'])
                vy = v_max * math.sin(obs['theta'])

            # X boundary reflection (only when predicted to cross AND moving toward boundary)
            if x_new >= self.X_MAX and vx > 0:
                obs['theta'] = math.pi - obs['theta']
                vx = v_max * math.cos(obs['theta'])
                vy = v_max * math.sin(obs['theta'])
            elif x_new <= self.X_MIN and vx < 0:
                obs['theta'] = math.pi - obs['theta']
                vx = v_max * math.cos(obs['theta'])
                vy = v_max * math.sin(obs['theta'])

            obs['theta'] = math.atan2(math.sin(obs['theta']), math.cos(obs['theta']))
            
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            self.cmd_pubs[name].publish(cmd)
            
            state = Odometry()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'world'
            state.child_frame_id = name
            state.pose.pose = obs['state'].pose.pose
            state.twist.twist.linear.x = vx
            state.twist.twist.linear.y = vy
            self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = MultiObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
