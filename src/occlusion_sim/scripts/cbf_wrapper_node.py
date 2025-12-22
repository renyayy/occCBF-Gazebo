#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
import math

# 配置した safe_control ライブラリをインポート
# (フォルダ構成に合わせてパスを調整してください)
from occlusion_sim.safe_control.robots.double_integrator2D import DoubleIntegrator2D
from occlusion_sim.safe_control.position_control.cbf_qp import CBFQP

class CBFWrapperNode(Node):
    def __init__(self):
        super().__init__('cbf_wrapper_node')

        # --- パラメータ設定 ---
        self.dt = 0.05  # 制御周期 (20Hz)
        self.goal = np.array([[5.0], [0.0], [0.0]]) # 仮のゴール (x=5, y=0)
        
        # --- Robot Model の初期化 (Double Integrator) ---
        robot_spec = {
            'model': 'DoubleIntegrator2D',
            'v_max': 1.0,
            'a_max': 1.0,
            'radius': 0.25, # ロボットの半径
        }
        # Pythonライブラリ内のロボットクラス
        self.robot_model = DoubleIntegrator2D(self.dt, robot_spec)
        
        # --- Controller の初期化 (CBF-QP) ---
        # num_obs は最大障害物数
        self.controller = CBFQP(self.robot_model, robot_spec, num_obs=10)

        # --- 現在の状態 ---
        # X = [x, y, vx, vy]^T
        self.current_X = np.zeros((4, 1))
        self.obs_list = [] # [[x, y, r, vx, vy], ...]

        # --- ROS 2 通信設定 ---
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Gazeboから全モデルの位置を取得 (MVP用のチート)
        self.sub_model_states = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 制御ループタイマー
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("CBF Wrapper Node Started.")

    def odom_callback(self, msg):
        # ロボットの状態更新
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Planar move plugin の odom は body frame ではなく global frame の twist を返す前提
        # 必要に応じて座標変換が必要だが、今回は簡易的にそのまま使用
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        
        self.current_X[0, 0] = x
        self.current_X[1, 0] = y
        self.current_X[2, 0] = vx
        self.current_X[3, 0] = vy

    def model_states_callback(self, msg):
        # 障害物情報の抽出
        # 名前が 'unit_cylinder' や 'obstacle' を含むモデルを障害物とみなす
        obs_data = []
        
        for i, name in enumerate(msg.name):
            if 'unit_cylinder' in name or 'MovingCylinder' in name: # 障害物の名前に合わせて変更してください
                ox = msg.pose[i].position.x
                oy = msg.pose[i].position.y
                ovx = msg.twist[i].linear.x
                ovy = msg.twist[i].linear.y
                radius = 0.3 # 障害物の半径 (SDFに合わせて設定)
                
                # safe_control が期待する形式: [x, y, r, vx, vy]
                obs_data.append([ox, oy, radius, ovx, ovy])
        
        if obs_data:
            self.obs_list = np.array(obs_data)
        else:
            self.obs_list = np.empty((0, 5))

    def control_loop(self):
        # 1. Nominal Input (ゴールに向かうための入力) の計算
        # Double Integrator の nominal_input は加速度を返す
        u_ref = self.robot_model.nominal_input(self.current_X, self.goal)
        
        control_ref = {
            'u_ref': u_ref
        }

        # 2. CBF-QP で安全な制御入力 u (加速度) を計算
        try:
            # nearest_multi_obs: 近くの障害物リスト (numpy array)
            # 全障害物を渡して、内部でソートしてもらうか、ここで絞る
            u_safe = self.controller.solve_control_problem(
                self.current_X, control_ref, self.obs_list
            )
        except Exception as e:
            self.get_logger().warn(f"QP Solver Failed: {e}")
            u_safe = np.zeros((2, 1)) # 停止

        # status check
        if self.controller.status != 'optimal':
             self.get_logger().warn(f"QP Status: {self.controller.status}")

        # 3. 加速度 u_safe を 速度指令 cmd_vel に変換
        # v_cmd = v_curr + a * dt
        ax = u_safe[0, 0]
        ay = u_safe[1, 0]
        
        cmd_vx = self.current_X[2, 0] + ax * self.dt
        cmd_vy = self.current_X[3, 0] + ay * self.dt
        
        # 速度制限 (Clip)
        v_norm = np.hypot(cmd_vx, cmd_vy)
        max_v = 1.0
        if v_norm > max_v:
            cmd_vx = (cmd_vx / v_norm) * max_v
            cmd_vy = (cmd_vy / v_norm) * max_v

        # 4. Publish
        twist = Twist()
        twist.linear.x = float(cmd_vx)
        twist.linear.y = float(cmd_vy)
        twist.angular.z = 0.0 # 全方向移動なので回転は一旦無視
        
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