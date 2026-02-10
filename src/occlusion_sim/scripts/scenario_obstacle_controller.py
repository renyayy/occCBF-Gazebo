#!/usr/bin/env python3
"""Scenario Obstacle Controller - Fixed waypoint sequence (corner pop-out)"""
import rclpy
import math
from obstacle_controller_base import ObstacleControllerBase


class ScenarioObstacleController(ObstacleControllerBase):
    def __init__(self):
        super().__init__('scenario_obstacle_controller', ['obs_0'])
        self.v_max = 0.4  # シナリオ固有速度
        self.waypoints = [(3.0, 2.0), (3.0, 1.0), (1.0, 1.0)]
        self.wp_idx = 0
        self.waypoint_threshold = 0.2
        self.get_logger().info('Scenario Obstacle Controller started (waypoint mode)')

    def control_loop(self):
        state = self.obs_states['obs_0']
        if state is None:
            return

        ox = state.pose.pose.position.x
        oy = state.pose.pose.position.y

        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - ox, ty - oy
        dist = math.hypot(dx, dy)

        if dist < self.waypoint_threshold and self.wp_idx < len(self.waypoints) - 1:
            self.wp_idx += 1
            tx, ty = self.waypoints[self.wp_idx]
            dx, dy = tx - ox, ty - oy
            dist = math.hypot(dx, dy)

        if dist < self.waypoint_threshold and self.wp_idx == len(self.waypoints) - 1:
            vx, vy = 0.0, 0.0
        elif dist < 1e-6:
            vx, vy = 0.0, 0.0
        else:
            vx = self.v_max * dx / dist
            vy = self.v_max * dy / dist

        self.publish_cmd('obs_0', vx, vy)
        self.publish_state('obs_0', state.pose.pose, vx, vy)


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
