#!/usr/bin/env python3
"""CBF Wrapper Node"""
import json
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float64MultiArray, MultiArrayDimension
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import sim_config  # safe_control パスも設定される

from robots.double_integrator2D import DoubleIntegrator2D
from position_control.cbf_qp import CBFQP
from position_control.backup_cbf_qp import BackupCBFQP
from position_control.mpc_cbf import MPCCBF
from position_control.optimal_decay_cbf_qp import OptimalDecayCBFQP
from position_control.optimal_decay_mpc_cbf import OptimalDecayMPCCBF

from utils.occlusion import OcclusionUtils
from scenarios import load_scenario

class CBFWrapperNode(Node):
    def __init__(self):
        super().__init__('cbf_wrapper_node')

        # ROS パラメータ
        self.declare_parameter('v_max', sim_config.ROBOT_V_MAX)
        self.declare_parameter('a_max', sim_config.ROBOT_A_MAX)
        self.declare_parameter('robot_radius', sim_config.ROBOT_RADIUS)
        self.declare_parameter('goal_x', sim_config.DEFAULT_GOAL[0])
        self.declare_parameter('goal_y', sim_config.DEFAULT_GOAL[1])
        self.declare_parameter('body_frame_odom', False)
        self.declare_parameter('sensing_range', sim_config.SENSING_RANGE)
        self.declare_parameter('scenario_name', 'multi_random')
        self.declare_parameter('auto_shutdown', False)
        self.declare_parameter('sim_timeout', 30.0)
        self.declare_parameter('result_file', '')

        # 制御パラメータ
        self.dt = sim_config.DT
        self.last_time = None
        self.goal = np.array([
            [self.get_parameter('goal_x').value],
            [self.get_parameter('goal_y').value],
            [0.0]
        ])
        self.body_frame_odom = self.get_parameter('body_frame_odom').value

        # 障害物名→半径マップ (シナリオ設定から構築)
        sc = load_scenario(self.get_parameter('scenario_name').value)
        self._obs_radius_map = {o['name']: o['radius'] for o in sc.get('obstacles', [])}
        self._obs_type_map = {
            o['name']: (0 if o.get('behavior') == 'static' else 1)
            for o in sc.get('obstacles', [])
        }
        self._expected_obs_count = len(sc.get('obstacles', []))
        self._all_obs_ready = False

        v_max = self.get_parameter('v_max').value
        a_max = self.get_parameter('a_max').value
        robot_radius = self.get_parameter('robot_radius').value
        sensing_range = self.get_parameter('sensing_range').value
        self.robot_spec = sim_config.make_robot_spec(v_max, a_max, radius=robot_radius,
                                                     sensing_range=sensing_range)

        # ロボットモデル
        self.robot = DoubleIntegrator2D(self.dt, self.robot_spec)

        # オクルージョン管理 (センシング範囲内かつ遮蔽されていない障害物のみ抽出)
        self.occlusion_manager = OcclusionUtils(
            robot=self.robot,
            robot_spec=self.robot_spec,
            sensing_range=self.robot_spec['sensing_range'],
            barrier_fn=None
        )

        # Controller選択 (使用するControllerをコメント解除)
        # self.controller = CBFQP(self.robot, self.robot_spec, num_obs=10)              # 基本CBF-QP
        self.controller = BackupCBFQP(self.robot, self.robot_spec, num_obs=10)        # バックアップCBF-QP (遮蔽対応)
        # self.controller = MPCCBF(self.robot, self.robot_spec)                         # MPC-CBF
        # self.controller = OptimalDecayCBFQP(self.robot, self.robot_spec)              # 最適減衰CBF-QP
        # self.controller = OptimalDecayMPCCBF(self.robot, self.robot_spec)             # 最適減衰MPC-CBF

        # 状態変数
        self.X = np.zeros((4, 1))  # [x, y, vx, vy]
        self.odom_received = False
        self.obs_list = np.empty((0, 8))  # [x, y, radius, vx, vy, _, _, obs_type]
        self.obstacle_states = {}
        self.goal_reached = False
        self.sim_start_time = None

        # ROS通信
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, '/cbf_debug_info', 10)
        _latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sim_start_pub = self.create_publisher(Empty, '/sim_start', _latch_qos)
        self._sim_started = False
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('CBF Wrapper Node started')

    def odom_cb(self, msg):
        """エゴロボットの状態 (位置・速度) を更新"""
        self.X[0, 0] = msg.pose.pose.position.x
        self.X[1, 0] = msg.pose.pose.position.y

        if self.body_frame_odom:
            # Unicycle: body frame → world frame 変換
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            v_body = msg.twist.twist.linear.x
            self.X[2, 0] = v_body * math.cos(yaw)
            self.X[3, 0] = v_body * math.sin(yaw)
        else:
            self.X[2, 0] = msg.twist.twist.linear.x
            self.X[3, 0] = msg.twist.twist.linear.y
        self.odom_received = True

    def obs_cb(self, msg):
        """障害物の状態を更新"""
        name = msg.child_frame_id or 'obs_0'
        obs_radius = self._obs_radius_map.get(name, sim_config.OBSTACLE_RADIUS)
        obs_type = self._obs_type_map.get(name, 1)  # 0: static, 1: dynamic
        self.obstacle_states[name] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            obs_radius,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            0.0, 0.0,
            float(obs_type),
        ]
        self.obs_list = np.array(list(self.obstacle_states.values()))

    def _write_result_and_exit(self, outcome, duration):
        result_file = self.get_parameter('result_file').value
        if result_file:
            os.makedirs(os.path.dirname(result_file), exist_ok=True)
            with open(result_file, 'w') as f:
                json.dump({'outcome': outcome, 'duration': duration}, f)
        self.get_logger().info(f'Auto-shutdown: {outcome} (duration={duration:.2f}s)')
        self.cmd_pub.publish(Twist())
        raise SystemExit

    def control_loop(self):
        """メイン制御ループ (dt周期で実行)"""
        if not self.odom_received:
            return

        # 全障害物の /obstacle/state 受信を待機（ゼロ速度で静止維持）
        if not self._all_obs_ready:
            self.cmd_pub.publish(Twist())
            if len(self.obstacle_states) < self._expected_obs_count:
                self.get_logger().info(
                    f'Waiting for obstacles: {len(self.obstacle_states)}/{self._expected_obs_count}',
                    throttle_duration_sec=1.0)
                return
            self._all_obs_ready = True

        # /sim_start を発行してエゴ・障害物を同時開始
        # rosbag recorder が /cbf_debug_info を購読するまで待機
        if not self._sim_started:
            if self.debug_pub.get_subscription_count() == 0:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    'Waiting for /cbf_debug_info subscriber (rosbag)...',
                    throttle_duration_sec=1.0)
                return
            self.sim_start_pub.publish(Empty())
            self.sim_start_time = self.get_clock().now()
            self._sim_started = True
            self.last_time = self.sim_start_time
            self.get_logger().info('Published /sim_start — ego + obstacles start simultaneously')
            # 初期状態を記録（制御前の位置）
            now = self.sim_start_time
            debug_msg = Float64MultiArray()
            debug_msg.layout.dim = [MultiArrayDimension(label='cbf_debug', size=17, stride=17)]
            debug_msg.data = [
                float(now.nanoseconds) * 1e-9,           # 0: stamp_sec
                float('inf'),                              # 1: h_min
                float('inf'),                              # 2: min_dist
                0.0, 0.0, 0.0,                            # 3-5: constraints, qp_time, intervention
                0.0, 0.0, 0.0, 0.0,                       # 6-9: u_x, u_y, u_ref_x, u_ref_y
                float(self.X[0, 0]), float(self.X[1, 0]), # 10-11: robot_x, robot_y
                float(self.X[2, 0]), float(self.X[3, 0]), # 12-13: robot_vx, robot_vy
                1.0,                                       # 14: status_ok
                0.0, float(self._expected_obs_count),      # 15-16: visible, total
            ]
            self.debug_pub.publish(debug_msg)
            return

        # sim time ベースで dt を動的計算
        now = self.get_clock().now()
        if self.last_time is not None:
            dt_ns = (now - self.last_time).nanoseconds
            dt = dt_ns * 1e-9
            if dt > 0.0:
                self.dt = dt
                self.robot.dt = dt
        self.last_time = now

        auto_shutdown = self.get_parameter('auto_shutdown').value
        sim_elapsed = (now - self.sim_start_time).nanoseconds * 1e-9 if self.sim_start_time else 0.0

        # ゴール到達判定
        dist = np.hypot(self.goal[0, 0] - self.X[0, 0], self.goal[1, 0] - self.X[1, 0])
        if dist < sim_config.GOAL_THRESHOLD:
            if not self.goal_reached:
                self.get_logger().info(f'Goal reached! (distance: {dist:.3f}m)')
                self.goal_reached = True
            self.cmd_pub.publish(Twist())
            if auto_shutdown:
                self._write_result_and_exit('goal_reached', sim_elapsed)
            return

        if auto_shutdown:
            # 衝突検知
            R_ego = self.robot_spec['radius']
            for obs in self.obs_list:
                d = np.hypot(self.X[0, 0] - obs[0], self.X[1, 0] - obs[1])
                if d < R_ego + obs[2]:
                    self._write_result_and_exit('collision', sim_elapsed)

            # タイムアウト検知
            if sim_elapsed > self.get_parameter('sim_timeout').value:
                self._write_result_and_exit('timeout', sim_elapsed)

        # 可視障害物のフィルタリング (センシング範囲 & オクルージョン考慮)
        visible_obs, _ = self.occlusion_manager._filter_visible_and_build_occ(
            self.X, self.obs_list
        )

        total_obs = len(self.obs_list)
        visible_count = len(visible_obs)

        if len(visible_obs) > 0:
            visible_obs_np = np.array(visible_obs)
        else:
            visible_obs_np = np.empty((0, 8))

        # ノミナル入力 (ゴール方向への加速度)
        u_ref = self.robot.nominal_input(self.X, self.goal)

        # CBF-QP: 安全制約を満たす制御入力を計算
        try:
            u = self.controller.solve_control_problem(self.X, {'u_ref': u_ref}, visible_obs_np)
            if u is None or np.any(np.isnan(u)) or self.controller.status != 'optimal':
                u = self.robot.stop(self.X)
        except Exception as e:
            self.get_logger().warn(f'QP failed: {e}', throttle_duration_sec=1.0)
            u = self.robot.stop(self.X)

        # CBF効果ログ
        u_diff = float(np.linalg.norm(u - u_ref))
        min_dist = float(np.min(np.hypot(
            visible_obs_np[:, 0] - self.X[0, 0],
            visible_obs_np[:, 1] - self.X[1, 0]
        ))) if len(visible_obs_np) > 0 else float('inf')
        intervention = getattr(self.controller, 'last_intervention', '-')
        status = getattr(self.controller, 'status', '-')
        n_constraints = getattr(self.controller, 'last_num_constraints', '-')
        self.get_logger().info(
            f'Obs: {visible_count}/{total_obs}'
            f' | CBF: {intervention} [{status}]'
            f' | Δu: {u_diff:.3f}'
            f' | MinDist: {min_dist:.2f}'
            f' | Constraints: {n_constraints}',
            throttle_duration_sec=0.1
        )

        # CBFデバッグ情報をパブリッシュ
        # h(x) = ||p_rel||^2 - d_min^2 (backup_cbf_qp.py:232 と同一)
        R_robot = self.robot_spec['radius']
        h_min = float('inf')
        for obs in visible_obs_np:
            p_rel = np.array([self.X[0, 0] - obs[0], self.X[1, 0] - obs[1]])
            d_min = obs[2] + R_robot
            h_min = min(h_min, float(p_rel @ p_rel - d_min**2))

        debug_msg = Float64MultiArray()
        debug_msg.layout.dim = [MultiArrayDimension(label='cbf_debug', size=17, stride=17)]
        debug_msg.data = [
            float(now.nanoseconds) * 1e-9,            # 0: stamp_sec
            h_min,                                      # 1: h_min
            min_dist,                                   # 2: min_dist
            float(n_constraints if n_constraints != '-' else 0),  # 3: num_constraints
            float(getattr(self.controller, 'last_qp_solve_time_ms', 0.0) or 0.0),  # 4: qp_solve_time_ms
            1.0 if intervention == 'backup_qp' else 0.0,  # 5: intervention
            float(u[0, 0]), float(u[1, 0]),             # 6-7: u_x, u_y
            float(u_ref[0, 0]), float(u_ref[1, 0]),     # 8-9: u_ref_x, u_ref_y
            float(self.X[0, 0]), float(self.X[1, 0]),   # 10-11: robot_x, robot_y
            float(self.X[2, 0]), float(self.X[3, 0]),   # 12-13: robot_vx, robot_vy
            1.0 if status == 'optimal' else 0.0,        # 14: status_ok
            float(visible_count),                        # 15: num_visible_obs
            float(total_obs),                            # 16: num_total_obs
        ]
        self.debug_pub.publish(debug_msg)

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
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CBFWrapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()