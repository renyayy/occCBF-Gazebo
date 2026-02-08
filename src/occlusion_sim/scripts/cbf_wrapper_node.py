#!/usr/bin/env python3
"""CBF Wrapper Node"""
import os
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
safe_control_path = os.path.join(current_dir, '..', 'safe_control')
sys.path.append(safe_control_path)

from robots.double_integrator2D import DoubleIntegrator2D
from position_control.cbf_qp import CBFQP
from position_control.backup_cbf_qp import BackupCBFQP
from position_control.mpc_cbf import MPCCBF
from position_control.optimal_decay_cbf_qp import OptimalDecayCBFQP
from position_control.optimal_decay_mpc_cbf import OptimalDecayMPCCBF

from utils.occlusion import OcclusionUtils
from attitude_control.velocity_tracking_yaw import VelocityTrackingYaw
from scipy.spatial.transform import Rotation as R

class CBFWrapperNode(Node):
    def __init__(self):
        super().__init__('cbf_wrapper_node')

        # 制御パラメータ
        self.dt = 0.05
        self.goal = np.array([[20.0], [7.5], [0.0]])
        self.obstacle_radius = 0.25

        self.robot_spec = {
            'model': 'DoubleIntegrator2D',
            'v_max': 1.0,
            'a_max': 1.0,
            'radius': 0.25,
            'sensing_range': 10.0,
            'backup_cbf': {'T_horizon': 3.0, 'dt_backup': 0.05, 'alpha': 2.0},
        }

        # ロボットモデル
        self.robot = DoubleIntegrator2D(self.dt, self.robot_spec)

        # オクルージョン管理 (センシング範囲内かつ遮蔽されていない障害物のみ抽出)
        self.occlusion_manager = OcclusionUtils(
            robot=self.robot,
            robot_spec=self.robot_spec,
            sensing_range=self.robot_spec['sensing_range'],
            barrier_fn=None
        )

        # 向き制御 (速度ベクトル方向に向きを追従)
        self.robot_spec['w_max'] = 1.0
        self.attitude_controller = VelocityTrackingYaw(self.robot, self.robot_spec, kp=1.5)
        self.current_yaw = 0.0

        # Controller選択 (使用するControllerをコメント解除)
        # self.controller = CBFQP(self.robot, self.robot_spec, num_obs=10)              # 基本CBF-QP
        self.controller = BackupCBFQP(self.robot, self.robot_spec, num_obs=10)        # バックアップCBF-QP (遮蔽対応)
        # self.controller = MPCCBF(self.robot, self.robot_spec)                         # MPC-CBF
        # self.controller = OptimalDecayCBFQP(self.robot, self.robot_spec)              # 最適減衰CBF-QP
        # self.controller = OptimalDecayMPCCBF(self.robot, self.robot_spec)             # 最適減衰MPC-CBF

        # 状態変数
        self.X = np.zeros((4, 1))  # [x, y, vx, vy]
        self.odom_received = False
        self.obs_list = np.empty((0, 5))  # [x, y, radius, vx, vy]
        self.obstacle_states = {}
        self.goal_reached = False

        # ROS通信
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('CBF Wrapper Node started')

    def odom_cb(self, msg):
        """エゴロボットの状態 (位置・速度・姿勢) を更新"""
        self.X[0, 0] = msg.pose.pose.position.x
        self.X[1, 0] = msg.pose.pose.position.y
        self.X[2, 0] = msg.twist.twist.linear.x
        self.X[3, 0] = msg.twist.twist.linear.y

        # クォータニオン → Yaw角 (向き制御用)
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        rot = R.from_quat(quat)
        self.current_yaw = rot.as_euler('xyz')[2]

        self.odom_received = True

    def obs_cb(self, msg):
        """障害物の状態を更新"""
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
        """メイン制御ループ (dt周期で実行)"""
        if not self.odom_received:
            return

        # ゴール到達判定
        dist = np.hypot(self.goal[0, 0] - self.X[0, 0], self.goal[1, 0] - self.X[1, 0])
        if dist < 0.3:
            if not self.goal_reached:
                self.get_logger().info(f'Goal reached! (distance: {dist:.3f}m)')
                self.goal_reached = True
            self.cmd_pub.publish(Twist())
            return

        # 可視障害物のフィルタリング (センシング範囲 & オクルージョン考慮)
        visible_obs, _ = self.occlusion_manager._filter_visible_and_build_occ(
            self.X, self.obs_list
        )

        if len(visible_obs) > 0:
            visible_obs_np = np.array(visible_obs)
        else:
            visible_obs_np = np.empty((0, 5))

        # ノミナル入力 (ゴール方向への加速度)
        u_ref = self.robot.nominal_input(self.X, self.goal)

        # CBF-QP: 安全制約を満たす制御入力を計算
        try:
            u = self.controller.solve_control_problem(self.X, {'u_ref': u_ref}, visible_obs_np)
            if u is None or self.controller.status != 'optimal':
                u = np.zeros((2, 1))
        except Exception as e:
            self.get_logger().warn(f'QP failed: {e}', throttle_duration_sec=1.0)
            u = np.zeros((2, 1))

        # 向き制御: 現在の速度ベクトル方向に向きを追従
        current_vel = self.X[2:4]
        u_yaw = self.attitude_controller.solve_control_problem(
            robot_state=self.X,
            current_yaw=self.current_yaw,
            u=current_vel
        )

        angular_z = float(u_yaw[0, 0])

        # 加速度 → 速度
        vx = self.X[2, 0] + float(u[0, 0]) * self.dt
        vy = self.X[3, 0] + float(u[1, 0]) * self.dt

        # 速度制限
        v_max = self.robot_spec['v_max']
        v_norm = np.hypot(vx, vy)
        if v_norm > v_max:
            vx, vy = vx / v_norm * v_max, vy / v_norm * v_max

        # 指令値をパブリッシュ
        twist = Twist()
        twist.linear.x, twist.linear.y = float(vx), float(vy)
        twist.angular.z = angular_z
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