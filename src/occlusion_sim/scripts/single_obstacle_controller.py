#!/usr/bin/env python3
"""Single Obstacle Controller - Chases ego robot"""
import rclpy
from nav_msgs.msg import Odometry
import math
import sim_config
from obstacle_controller_base import ObstacleControllerBase


class SingleObstacleController(ObstacleControllerBase):
    def __init__(self):
        super().__init__('single_obstacle_controller', ['obs_0'])
        self.v_max = sim_config.OBSTACLE_V_MAX
        self.ego_x = None
        self.ego_y = None
        self.create_subscription(Odometry, '/odom', self.ego_odom_cb, 10)
        self.get_logger().info('Single Obstacle Controller started (chase mode)')

    def ego_odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

    def control_loop(self):
        state = self.obs_states['obs_0']
        if self.ego_x is None or state is None:
            return

        ox = state.pose.pose.position.x
        oy = state.pose.pose.position.y
        dx = self.ego_x - ox
        dy = self.ego_y - oy
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            vx, vy = 0.0, 0.0
        else:
            vx = self.v_max * dx / dist
            vy = self.v_max * dy / dist

        self.publish_cmd('obs_0', vx, vy)
        self.publish_state('obs_0', state.pose.pose, vx, vy)


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
