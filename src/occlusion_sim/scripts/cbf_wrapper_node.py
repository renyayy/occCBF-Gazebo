#!/usr/bin/env python3
"""
CBF Wrapper Node

Control Barrier Function (CBF) を使用してエゴロボットを安全に制御するノード。
障害物を回避しながらゴールに向かいます。

障害物情報は /obstacle/state トピックから取得します。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

# safe_control ライブラリをインポート
from occlusion_sim.safe_control.robots.double_integrator2D import DoubleIntegrator2D
from occlusion_sim.safe_control.position_control.cbf_qp import CBFQP


class CBFWrapperNode(Node):
    def __init__(self):
        super().__init__('cbf_wrapper_node')

        # === パラメータ宣言 ===
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('a_max', 1.0)
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('obstacle_radius', 0.3)
        self.declare_parameter('num_obstacles', 10)
        
        # === パラメータ取得 ===
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        self.dt = self.get_parameter('dt').value
        v_max = self.get_parameter('v_max').value
        a_max = self.get_parameter('a_max').value
        robot_radius = self.get_parameter('robot_radius').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        num_obs = self.get_parameter('num_obstacles').value
        
        self.goal = np.array([[goal_x], [goal_y], [0.0]])
        
        # === Robot Model の初期化 (Double Integrator) ===
        robot_spec = {
            'model': 'DoubleIntegrator2D',
            'v_max': v_max,
            'a_max': a_max,
            'radius': robot_radius,
        }
        self.robot_spec = robot_spec
        self.robot_model = DoubleIntegrator2D(self.dt, robot_spec)
        
        # === Controller の初期化 (CBF-QP) ===
        self.controller = CBFQP(self.robot_model, robot_spec, num_obs=num_obs)

        # === 現在の状態 ===
        # X = [x, y, vx, vy]^T
        self.current_X = np.zeros((4, 1))
        self.ego_odom_received = False
        
        # 障害物リスト: [[x, y, r, vx, vy], ...]
        self.obs_list = np.empty((0, 5))
        self.obstacle_states = {}  # 複数障害物対応用 dict

        # === ROS 2 通信設定 ===
        # エゴロボットのオドメトリ
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 障害物の状態
        self.sub_obstacle_state = self.create_subscription(
            Odometry, '/obstacle/state', self.obstacle_state_callback, 10)
        
        # 速度指令
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # === 制御ループタイマー ===
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # === 初期化状態 ===
        self.initialized = False
        self.init_wait_count = 0
        self.init_wait_max = 50  # 50回 * dt秒 待機
        
        self.get_logger().info(
            f'CBF Wrapper Node Started.\n'
            f'  Goal: ({goal_x}, {goal_y})\n'
            f'  Robot radius: {robot_radius} m\n'
            f'  Obstacle radius: {self.obstacle_radius} m\n'
            f'  Max velocity: {v_max} m/s\n'
            f'  Max acceleration: {a_max} m/s^2'
        )

    def odom_callback(self, msg):
        """エゴロボットのオドメトリ受信"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        
        self.current_X[0, 0] = x
        self.current_X[1, 0] = y
        self.current_X[2, 0] = vx
        self.current_X[3, 0] = vy
        self.ego_odom_received = True

    def obstacle_state_callback(self, msg):
        """
        障害物の状態受信（/obstacle/state トピック）
        複数障害物に対応するため、child_frame_id をキーとして保存
        """
        obs_name = msg.child_frame_id if msg.child_frame_id else 'obstacle_0'
        
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        ovx = msg.twist.twist.linear.x
        ovy = msg.twist.twist.linear.y
        
        # safe_control 形式: [x, y, r, vx, vy]
        self.obstacle_states[obs_name] = [ox, oy, self.obstacle_radius, ovx, ovy]
        
        # obs_list を更新
        if self.obstacle_states:
            self.obs_list = np.array(list(self.obstacle_states.values()))
        else:
            self.obs_list = np.empty((0, 5))

    def control_loop(self):
        """メイン制御ループ"""
        
        # 初期化待ち（エゴオドメトリが来るまで待機）
        if not self.ego_odom_received:
            self.init_wait_count += 1
            if self.init_wait_count % 20 == 0:
                self.get_logger().info('Waiting for ego robot odometry...')
            return
        
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('Ego odometry received. Starting control.')
        
        # ゴール到達チェック
        dist_to_goal = np.hypot(
            self.goal[0, 0] - self.current_X[0, 0],
            self.goal[1, 0] - self.current_X[1, 0]
        )
        if dist_to_goal < 0.3:
            # ゴール到達 → 停止
            twist = Twist()
            self.pub_cmd_vel.publish(twist)
            self.get_logger().info('Goal reached! Stopping.', throttle_duration_sec=5.0)
            return
        
        # 1. Nominal Input (ゴールに向かうための入力) の計算
        u_ref = self.robot_model.nominal_input(self.current_X, self.goal)
        
        control_ref = {
            'u_ref': u_ref
        }

        # 2. CBF-QP で安全な制御入力 u (加速度) を計算
        u_safe = None
        try:
            u_safe = self.controller.solve_control_problem(
                self.current_X, control_ref, self.obs_list
            )
            # ステータスチェック
            if hasattr(self.controller, 'status') and self.controller.status != 'optimal':
                self.get_logger().warn(
                    f"QP Status: {self.controller.status}", 
                    throttle_duration_sec=1.0
                )
        except Exception as e:
            self.get_logger().warn(f"QP Solver Failed: {e}", throttle_duration_sec=1.0)
            u_safe = None

        # 3. 加速度 u_safe を速度指令 cmd_vel に変換
        if u_safe is not None and u_safe.shape == (2, 1):
            ax = float(u_safe[0, 0])
            ay = float(u_safe[1, 0])
        else:
            ax, ay = 0.0, 0.0  # 失敗時は停止
        
        cmd_vx = self.current_X[2, 0] + ax * self.dt
        cmd_vy = self.current_X[3, 0] + ay * self.dt
        
        # 速度制限 (Clip)
        v_norm = np.hypot(cmd_vx, cmd_vy)
        max_v = self.robot_spec['v_max']
        if v_norm > max_v:
            cmd_vx = (cmd_vx / v_norm) * max_v
            cmd_vy = (cmd_vy / v_norm) * max_v

        # 4. Publish
        twist = Twist()
        twist.linear.x = float(cmd_vx)
        twist.linear.y = float(cmd_vy)
        twist.angular.z = 0.0
        
        self.pub_cmd_vel.publish(twist)


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