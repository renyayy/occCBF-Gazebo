#!/usr/bin/env python3
"""Base class for obstacle controllers."""
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sim_config


class ObstacleControllerBase(Node):
    """障害物コントローラ共通基底: odom購読, cmd_vel/state発行, タイマー"""

    def __init__(self, node_name, obs_names):
        super().__init__(node_name)
        self.obs_states = {n: None for n in obs_names}
        self.cmd_pubs = {}
        for name in obs_names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n), 10)
            self.cmd_pubs[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10)
        self.state_pub = self.create_publisher(Odometry, '/obstacle/state', 10)
        self.create_timer(sim_config.DT, self.control_loop)

    def _odom_cb(self, msg, name):
        self.obs_states[name] = msg

    def publish_cmd(self, name, vx, vy):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        self.cmd_pubs[name].publish(cmd)

    def publish_state(self, name, pose, vx, vy):
        state = Odometry()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'world'
        state.child_frame_id = name
        state.pose.pose = pose
        state.twist.twist.linear.x = float(vx)
        state.twist.twist.linear.y = float(vy)
        self.state_pub.publish(state)

    def control_loop(self):
        raise NotImplementedError
