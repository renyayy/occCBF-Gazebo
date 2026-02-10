#!/usr/bin/env python3
"""Multi Obstacle Controller - Random walk behavior matching safe_control/dynamic_env"""
import rclpy
import math
import random
import sim_config
from obstacle_controller_base import ObstacleControllerBase


class MultiObstacleController(ObstacleControllerBase):
    def __init__(self):
        random.seed(42)
        obs_names = [f'obs_{i}' for i in range(5)]
        super().__init__('multi_obstacle_controller', obs_names)

        self.obstacles = {
            name: {'mode': 1, 'v_max': sim_config.OBSTACLE_V_MAX,
                   'theta': random.uniform(-math.pi, math.pi)}
            for name in obs_names
        }
        self.get_logger().info('Multi Obstacle Controller started')

    def control_loop(self):
        for name, obs in self.obstacles.items():
            state = self.obs_states[name]
            if state is None:
                continue

            x = state.pose.pose.position.x
            y = state.pose.pose.position.y
            theta = obs['theta']
            v_max = obs['v_max']

            # Random walk: 5% chance of large turn (matches safe_control/dynamic_env)
            if obs['mode'] == 1 and random.random() < 0.05:
                theta += random.gauss(0.0, 0.2)
                obs['theta'] = theta

            vx = v_max * math.cos(theta)
            vy = v_max * math.sin(theta)

            # Predict next position
            x_new = x + vx * sim_config.DT
            y_new = y + vy * sim_config.DT

            # Y boundary reflection
            if (y_new >= sim_config.ENV_Y_MAX and vy > 0) or \
               (y_new <= sim_config.ENV_Y_MIN and vy < 0):
                obs['theta'] = -obs['theta']
                vx = v_max * math.cos(obs['theta'])
                vy = v_max * math.sin(obs['theta'])

            # X boundary reflection
            if (x_new >= sim_config.ENV_X_MAX and vx > 0) or \
               (x_new <= sim_config.ENV_X_MIN and vx < 0):
                obs['theta'] = math.pi - obs['theta']
                vx = v_max * math.cos(obs['theta'])
                vy = v_max * math.sin(obs['theta'])

            obs['theta'] = math.atan2(math.sin(obs['theta']),
                                      math.cos(obs['theta']))

            self.publish_cmd(name, vx, vy)
            self.publish_state(name, state.pose.pose, vx, vy)


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
