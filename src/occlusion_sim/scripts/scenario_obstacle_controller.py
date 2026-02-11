#!/usr/bin/env python3
"""Scenario Obstacle Controller - scenario configから障害物の挙動を読み込む汎用コントローラ

behavior types: waypoint, chase, random_walk, static
"""
import rclpy
import math
import random
from nav_msgs.msg import Odometry
import sim_config
from obstacle_controller_base import ObstacleControllerBase
from scenarios import load_scenario


class ScenarioObstacleController(ObstacleControllerBase):
    def __init__(self):
        super().__init__('scenario_obstacle_controller')

        self.declare_parameter('scenario_name', 'multi_random')
        scenario_name = self.get_parameter('scenario_name').value

        sc = load_scenario(scenario_name)
        obs_configs = sc.get('obstacles', [])
        obs_names = [o['name'] for o in obs_configs]
        self._setup_obs(obs_names)

        env_cfg = sc['env']
        random.seed(sc.get('seed', 42))

        self._obs_behaviors = {}
        self._ego_x = None
        self._ego_y = None
        needs_ego_odom = False

        for obs_cfg in obs_configs:
            name = obs_cfg['name']
            behavior = obs_cfg['behavior']
            beh = {'behavior': behavior, 'v_max': obs_cfg['v_max']}

            if behavior == 'waypoint':
                beh['waypoints'] = obs_cfg['waypoints']
                beh['wp_idx'] = 0
                beh['threshold'] = 0.2
                beh['stopped'] = False
            elif behavior == 'chase':
                needs_ego_odom = True
            elif behavior == 'random_walk':
                beh['theta'] = random.uniform(-math.pi, math.pi)
                beh['y_min'] = env_cfg['y_min']
                beh['y_max'] = env_cfg['y_max']
                beh['x_min'] = env_cfg['x_min']
                beh['x_max'] = env_cfg['x_max']

            self._obs_behaviors[name] = beh

        if needs_ego_odom:
            self.create_subscription(Odometry, '/odom', self._ego_odom_cb, 10)

        self.get_logger().info(f'Scenario Controller: {len(obs_names)} obstacles')

    def _ego_odom_cb(self, msg):
        self._ego_x = msg.pose.pose.position.x
        self._ego_y = msg.pose.pose.position.y

    def control_loop(self):
        for name, beh in self._obs_behaviors.items():
            state = self.obs_states.get(name)
            if state is None:
                continue

            ox = state.pose.pose.position.x
            oy = state.pose.pose.position.y
            behavior = beh['behavior']

            if behavior == 'waypoint':
                vx, vy = self._waypoint_control(name, ox, oy, beh)
            elif behavior == 'chase':
                vx, vy = self._chase_control(ox, oy, beh)
            elif behavior == 'random_walk':
                vx, vy = self._random_walk_control(ox, oy, beh)
            else:
                vx, vy = 0.0, 0.0

            self.publish_cmd(name, vx, vy)
            self.publish_state(name, state.pose.pose, vx, vy)

    def _waypoint_control(self, name, ox, oy, beh):
        if beh['stopped']:
            return 0.0, 0.0

        wps = beh['waypoints']
        wp_idx = beh['wp_idx']
        v_max = beh['v_max']

        tx, ty = wps[wp_idx]
        dist = math.hypot(tx - ox, ty - oy)

        if dist < beh['threshold'] and wp_idx < len(wps) - 1:
            wp_idx += 1
            beh['wp_idx'] = wp_idx
            tx, ty = wps[wp_idx]
            dist = math.hypot(tx - ox, ty - oy)

        if dist < beh['threshold'] and wp_idx == len(wps) - 1:
            beh['stopped'] = True
            self.get_logger().info(f'{name}: reached final waypoint, stopped')
            return 0.0, 0.0
        if dist < 1e-6:
            return 0.0, 0.0
        return v_max * (tx - ox) / dist, v_max * (ty - oy) / dist

    def _chase_control(self, ox, oy, beh):
        if self._ego_x is None:
            return 0.0, 0.0
        dx = self._ego_x - ox
        dy = self._ego_y - oy
        dist = math.hypot(dx, dy)
        v_max = beh['v_max']
        if dist < 1e-6:
            return 0.0, 0.0
        return v_max * dx / dist, v_max * dy / dist

    def _random_walk_control(self, ox, oy, beh):
        v_max = beh['v_max']
        theta = beh['theta']

        if random.random() < 0.05:
            theta += random.gauss(0.0, 0.2)
            beh['theta'] = theta

        vx = v_max * math.cos(theta)
        vy = v_max * math.sin(theta)

        x_new = ox + vx * sim_config.DT
        y_new = oy + vy * sim_config.DT

        if (y_new >= beh['y_max'] and vy > 0) or (y_new <= beh['y_min'] and vy < 0):
            beh['theta'] = -beh['theta']
            vx = v_max * math.cos(beh['theta'])
            vy = v_max * math.sin(beh['theta'])

        if (x_new >= beh['x_max'] and vx > 0) or (x_new <= beh['x_min'] and vx < 0):
            beh['theta'] = math.pi - beh['theta']
            vx = v_max * math.cos(beh['theta'])
            vy = v_max * math.sin(beh['theta'])

        beh['theta'] = math.atan2(math.sin(beh['theta']), math.cos(beh['theta']))
        return vx, vy


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
