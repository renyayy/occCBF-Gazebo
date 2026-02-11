#!/usr/bin/env python3
"""Base class for obstacle controllers."""
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sim_config


class ObstacleControllerBase(Node):
    """障害物コントローラ共通基底: odom購読, cmd_vel/state発行, タイマー"""

    def __init__(self, node_name, obs_names=None):
        super().__init__(node_name)
        if obs_names is not None:
            self._setup_obs(obs_names)

    def _setup_obs(self, obs_names):
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
        """Publish cmd_vel. vx/vy are world-frame; convert to body-frame for planar_move plugin."""
        state = self.obs_states.get(name)
        if state is not None:
            q = state.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            vx_body = vx * cos_y + vy * sin_y
            vy_body = -vx * sin_y + vy * cos_y
        else:
            vx_body, vy_body = vx, vy
        cmd = Twist()
        cmd.linear.x = float(vx_body)
        cmd.linear.y = float(vy_body)
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
