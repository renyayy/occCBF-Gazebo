#!/usr/bin/env python3
"""CBF Wrapper Node"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

from occlusion_sim.safe_control.robots.double_integrator2D import DoubleIntegrator2D
from occlusion_sim.safe_control.position_control.cbf_qp import CBFQP
from occlusion_sim.safe_control.position_control.backup_cbf_qp import BackupCBFQP
from occlusion_sim.safe_control.position_control.mpc_cbf import MPCCBF
from occlusion_sim.safe_control.position_control.optimal_decay_cbf_qp import OptimalDecayCBFQP
from occlusion_sim.safe_control.position_control.optimal_decay_mpc_cbf import OptimalDecayMPCCBF


class CBFWrapperNode(Node):
    def __init__(self):
        super().__init__('cbf_wrapper_node')

        # 設定
        self.dt = 0.05
        self.goal = np.array([[6.0], [-2.0], [0.0]])
        self.obstacle_radius = 0.3
        
        self.robot_spec = {
            'model': 'DoubleIntegrator2D',
            'v_max': 1.0,
            'a_max': 1.0,
            'radius': 0.25,
            'sensing_range': 10.0,
            'backup_cbf': {'T_horizon': 3.0, 'dt_backup': 0.05, 'alpha': 2.0},
        }

        self.robot = DoubleIntegrator2D(self.dt, self.robot_spec)

        # Controller選択 (使用する Controller をコメント解除) 
        # self.controller = CBFQP(self.robot, self.robot_spec, num_obs=10)              # 基本CBF-QP
        self.controller = BackupCBFQP(self.robot, self.robot_spec, num_obs=10)      # バックアップCBF-QP (遮蔽対応)
        # self.controller = MPCCBF(self.robot, self.robot_spec)                       # MPC-CBF
        # self.controller = OptimalDecayCBFQP(self.robot, self.robot_spec)            # 最適減衰CBF-QP
        # self.controller = OptimalDecayMPCCBF(self.robot, self.robot_spec)           # 最適減衰MPC-CBF

        # 状態
        self.X = np.zeros((4, 1))
        self.odom_received = False
        self.obs_list = np.empty((0, 5))
        self.obstacle_states = {}

        # ROS通信
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('CBF Wrapper Node started')

    def odom_cb(self, msg):
        self.X[0, 0] = msg.pose.pose.position.x
        self.X[1, 0] = msg.pose.pose.position.y
        self.X[2, 0] = msg.twist.twist.linear.x
        self.X[3, 0] = msg.twist.twist.linear.y
        self.odom_received = True

    def obs_cb(self, msg):
        name = msg.child_frame_id or 'obs_0'
        self.obstacle_states[name] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.obstacle_radius,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ]
        self.obs_list = np.array(list(self.obstacle_states.values()))

    def control_loop(self):
        if not self.odom_received:
            return

        # ゴール到達チェック
        dist = np.hypot(self.goal[0, 0] - self.X[0, 0], self.goal[1, 0] - self.X[1, 0])
        if dist < 0.3:
            self.cmd_pub.publish(Twist())
            return

        # Nominal input
        u_ref = self.robot.nominal_input(self.X, self.goal)

        # CBF-QP
        try:
            u = self.controller.solve_control_problem(self.X, {'u_ref': u_ref}, self.obs_list)
            if u is None or self.controller.status != 'optimal':
                u = np.zeros((2, 1))
        except Exception as e:
            self.get_logger().warn(f'QP failed: {e}', throttle_duration_sec=1.0)
            u = np.zeros((2, 1))

        # 加速度→速度
        vx = self.X[2, 0] + float(u[0, 0]) * self.dt
        vy = self.X[3, 0] + float(u[1, 0]) * self.dt

        # 速度制限
        v_max = self.robot_spec['v_max']
        v_norm = np.hypot(vx, vy)
        if v_norm > v_max:
            vx, vy = vx / v_norm * v_max, vy / v_norm * v_max

        # Publish
        twist = Twist()
        twist.linear.x, twist.linear.y = float(vx), float(vy)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CBFWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()