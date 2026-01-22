---
title: "姿勢制御とセンサーフィルタリングの実装が必要"
category: architecture
severity: high
status: open
created: 2026-01-23
---

## 概要

Double Integrator モデルを使用しているが、FoV (視野角) を持つセンサーを扱うには姿勢制御が必須。現在は姿勢制御が未実装のため、FoV が固定方向を向いたままになっている。また、センサー情報のフィルタリングも未実装のため、BackupCBF の効果が検証できない。

## 該当箇所

- `src/occlusion_sim/scripts/cbf_wrapper_node.py` - 姿勢制御とセンサーフィルタリングが未実装
- `src/occlusion_sim/scripts/sensor_visualizer_node.py:67` - yaw をオドメトリから取得しているが更新されない

## 問題の詳細

### 1. Double Integrator と姿勢制御の関係

**誤解**: Double Integrator は位置のみを制御するため、向き（yaw）は不要

**正解**: FoV があるセンサーを使う場合、姿勢は別途管理が必要

#### safe_control の実装方式

```python
# safe_control/robots/double_integrator2D.py
class DoubleIntegrator2D:
    '''
        X: [x, y, vx, vy]      # 位置制御状態
        theta: yaw angle        # 姿勢状態（別管理）
        U: [ax, ay]            # 位置制御入力
        U_attitude: [yaw_rate]  # 姿勢制御入力
    '''
```

safe_control では：
1. **位置制御**: Double Integrator (X = [x, y, vx, vy])
2. **姿勢制御**: 別途 `theta` (yaw角) を管理
3. **姿勢コントローラ**: `VelocityTrackingYaw` で**速度方向に yaw を追従**

```python
# safe_control/attitude_control/velocity_tracking_yaw.py:41-52
elif self.model == 'DoubleIntegrator2D':
    vx = robot_state[2, 0]
    vy = robot_state[3, 0]

desired_yaw = np.arctan2(vy, vx)  # 速度方向を向く
yaw_err = angle_normalize(desired_yaw - current_yaw)
u_att = self.kp * yaw_err  # P制御でyaw追従
```

つまり、**ロボットは常に移動方向を向く**ようになっており、FoV の中心も移動方向に向く。

### 2. 現在の実装の問題点

#### cbf_wrapper_node.py

```python
# 現状
class CBFWrapperNode(Node):
    def __init__(self):
        self.X = np.zeros((4, 1))  # 位置状態のみ
        # ❌ yaw 状態がない
        # ❌ 姿勢コントローラがない
```

#### sensor_visualizer_node.py

```python
# 現状: sensor_visualizer_node.py:60-68
def odom_cb(self, msg):
    self.robot_x = msg.pose.pose.position.x
    self.robot_y = msg.pose.pose.position.y
    # quaternion から yaw を取得
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
```

**問題**: Gazebo の holonomic robot モデルは yaw を更新しないため、`self.robot_yaw` は初期値（0）のまま固定される。

結果：
- FoV が**固定方向（初期向き）を向いたまま**
- ロボットが移動しても FoV は回転しない
- 可視化と実際の動きが不整合

### 3. センサー情報フィルタリングの欠如

#### 現状の実装

```python
# cbf_wrapper_node.py:65-74
def obs_cb(self, msg):
    name = msg.child_frame_id or 'obs_0'
    self.obstacle_states[name] = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        self.obstacle_radius,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y
    ]
    self.obs_list = np.array(list(self.obstacle_states.values()))  # 全て使用
```

**問題**: 全障害物情報を CBF コントローラに渡している

#### 必要なフィルタリング

1. **センシング範囲外**: `sensing_range=10m` 外の障害物は検知不可
2. **FoV 外**: `fov_angle=70°` 外の障害物はカメラで検知不可
3. **オクルージョン**: 他の障害物の影に隠れた障害物は検知不可

### 4. BackupCBF の効果が見えない理由

**現状**:
- 全障害物情報を受信 → BackupCBF が全てを事前に回避
- 遮蔽領域があっても、その背後の障害物も見えている
- **BackupCBF の本来の意義が発揮されない**

**本来の BackupCBF の動作**:
1. FoV 外や遮蔽領域から突然障害物が現れる
2. 通常の CBF では対処不可（反応時間が足りない）
3. BackupCBF は遮蔽領域に対して**事前減速**してバックアップ安全集合を確保
4. 突然の出現にも対応可能

## 影響

### 実装しない場合
- FoV が固定方向を向き、現実的なセンサー制約を再現できない
- BackupCBF の有効性を検証できない
- センサー制約下での動的障害物回避の研究目的が達成できない

### 実装した場合の予想される動き

#### フィルタリング前（現状）
- ロボットは全障害物を「見える」
- 全てを回避しながらゴールへ直進
- BackupCBF の効果が見えにくい

#### フィルタリング後
1. **FoV 外の障害物**: 検知できない → 横や後ろから接近する障害物に反応が遅れる可能性
2. **遮蔽された障害物**: 手前の障害物の影に隠れた障害物が見えない
3. **BackupCBF の効果**:
   - 遮蔽領域に対して**事前減速**
   - 遮蔽領域から突然出現する障害物に対応
   - 通常の CBF より**保守的な動き**（安全マージン大）
4. **yaw が速度方向を追従**:
   - 移動方向に FoV が向く
   - 進行方向の障害物は検知可能
   - 横から接近する障害物は見えにくい

## 実装提案

### 変更1: cbf_wrapper_node.py に姿勢制御を追加

```python
#!/usr/bin/env python3
"""CBF Wrapper Node with Attitude Control"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

from occlusion_sim.safe_control.robots.double_integrator2D import DoubleIntegrator2D, angle_normalize
from occlusion_sim.safe_control.position_control.backup_cbf_qp import BackupCBFQP
from occlusion_sim.safe_control.attitude_control.velocity_tracking_yaw import VelocityTrackingYaw


class CBFWrapperNode(Node):
    def __init__(self):
        super().__init__('cbf_wrapper_node')

        # 設定 (safe_control/dynamic_env準拠)
        self.dt = 0.05
        self.goal = np.array([[20.0], [7.5], [0.0]])
        self.obstacle_radius = 0.25
        self.fov_angle = np.deg2rad(70.0)  # FoV 追加

        self.robot_spec = {
            'model': 'DoubleIntegrator2D',
            'v_max': 1.0,
            'a_max': 1.0,
            'radius': 0.25,
            'sensing_range': 10.0,
            'w_max': 0.5,  # yaw rate の最大値を追加
            'backup_cbf': {'T_horizon': 3.0, 'dt_backup': 0.05, 'alpha': 2.0},
        }

        self.robot = DoubleIntegrator2D(self.dt, self.robot_spec)
        self.controller = BackupCBFQP(self.robot, self.robot_spec, num_obs=10)

        # 姿勢コントローラを追加
        self.att_controller = VelocityTrackingYaw(self.robot, self.robot_spec)

        # 状態
        self.X = np.zeros((4, 1))
        self.yaw = 0.0  # yaw 状態を追加
        self.odom_received = False
        self.obs_list = np.empty((0, 5))
        self.obstacle_states = {}

        # ROS通信
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('CBF Wrapper Node with Attitude Control started')

    def odom_cb(self, msg):
        self.X[0, 0] = msg.pose.pose.position.x
        self.X[1, 0] = msg.pose.pose.position.y
        self.X[2, 0] = msg.twist.twist.linear.x
        self.X[3, 0] = msg.twist.twist.linear.y

        # yaw を quaternion から取得
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

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

    def filter_observable_obstacles(self):
        """FoV、センシング範囲、遮蔽によるフィルタリング"""
        if len(self.obs_list) == 0:
            return np.empty((0, 5))

        robot_pos = self.X[:2, 0]
        observable = []

        for obs in self.obs_list:
            ox, oy, radius = obs[0], obs[1], obs[2]
            dist = np.hypot(ox - robot_pos[0], oy - robot_pos[1])

            # 1. センシング範囲チェック
            if dist > self.robot_spec['sensing_range']:
                continue

            # 2. FoV チェック（yaw を使用）
            angle_to_obs = np.arctan2(oy - robot_pos[1], ox - robot_pos[0])
            angle_diff = abs(angle_normalize(angle_to_obs - self.yaw))
            if angle_diff > self.fov_angle / 2:
                continue

            # 3. オクルージョンチェック（他の障害物による遮蔽）
            occluded = False
            for other_obs in observable:
                if self.is_occluded_by(robot_pos, obs[:2], other_obs[:3]):
                    occluded = True
                    break

            if not occluded:
                observable.append(obs)

        return np.array(observable) if observable else np.empty((0, 5))

    def is_occluded_by(self, robot_pos, target_pos, occluder):
        """target_pos が occluder によって遮蔽されているかチェック"""
        ox, oy, radius = occluder[0], occluder[1], occluder[2]
        occluder_center = np.array([ox, oy])

        # ロボットから遮蔽物体までの距離
        dist_to_occluder = np.linalg.norm(occluder_center - robot_pos)

        # ロボットからターゲットまでの距離
        dist_to_target = np.linalg.norm(target_pos - robot_pos)

        # 遮蔽物体がターゲットより遠ければ遮蔽しない
        if dist_to_occluder >= dist_to_target:
            return False

        # ロボット→ターゲットの方向ベクトル
        to_target = target_pos - robot_pos
        to_target_norm = to_target / np.linalg.norm(to_target)

        # ロボット→遮蔽物体の方向ベクトル
        to_occluder = occluder_center - robot_pos

        # ターゲット方向への投影距離
        proj_dist = np.dot(to_occluder, to_target_norm)

        # 投影距離が遮蔽物体の距離より大きければ遮蔽しない
        if proj_dist > dist_to_occluder:
            return False

        # 投影点から遮蔽物体中心までの距離
        proj_point = robot_pos + proj_dist * to_target_norm
        perp_dist = np.linalg.norm(proj_point - occluder_center)

        # 垂直距離が半径以下なら遮蔽している
        return perp_dist <= radius

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

        # センサーフィルタリングを適用
        filtered_obs = self.filter_observable_obstacles()

        # CBF-QP
        try:
            u = self.controller.solve_control_problem(self.X, {'u_ref': u_ref}, filtered_obs)
            if u is None or self.controller.status != 'optimal':
                u = np.zeros((2, 1))
        except Exception as e:
            self.get_logger().warn(f'QP failed: {e}', throttle_duration_sec=1.0)
            u = np.zeros((2, 1))

        # 姿勢制御を追加（速度方向に yaw を追従）
        u_att = self.att_controller.solve_control_problem(self.X, self.yaw, u)
        self.yaw = self.robot.step_rotate(self.yaw, u_att)

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
```

### 変更2: sensor_visualizer_node.py に yaw 購読を追加（オプション）

cbf_wrapper_node から yaw を publish して、sensor_visualizer が使用する方法もあるが、現状の実装では sensor_visualizer が独自に速度から yaw を計算することも可能：

```python
# sensor_visualizer_node.py:60-70 を修正
def odom_cb(self, msg):
    self.robot_x = msg.pose.pose.position.x
    self.robot_y = msg.pose.pose.position.y

    # 速度から yaw を計算（VelocityTrackingYaw と同様）
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    speed = np.hypot(vx, vy)

    if speed > 1e-2:
        self.robot_yaw = np.arctan2(vy, vx)  # 速度方向を向く
    # else: yaw を維持（停止中）

    self.odom_received = True
```

## 検証方法

### 1. 姿勢制御の確認
- RViz2 で FoV (黄色の楔形) がロボットの移動方向に追従するか確認
- ロボットが曲がるとき、FoV も一緒に回転するか確認

### 2. センサーフィルタリングの確認
- ロボットの横や後ろにいる障害物が検知されない（遮蔽領域に赤い楔形が表示されない）
- FoV 外の障害物に向かって移動すると、近づくにつれて FoV 内に入り検知される

### 3. BackupCBF の効果確認
- 遮蔽領域に接近するとき、事前に減速する動作が見られる
- 遮蔽領域から突然障害物が現れても衝突しない

## 実装の優先度

| 項目 | 優先度 | 理由 |
|------|--------|------|
| 姿勢制御（VelocityTrackingYaw） | **高** | FoV の向きを正しく制御するために必須 |
| センサーフィルタリング | **高** | BackupCBF の効果検証に必須 |
| オクルージョンチェック | 中 | より現実的なセンサーモデル |
| Gazebo での yaw 可視化 | 低 | RViz2 での可視化で十分 |

## まとめ

| 項目 | 現状 | 実装後 |
|------|------|--------|
| 位置制御 | ✅ Double Integrator | ✅ そのまま |
| 姿勢制御 | ❌ なし（固定向き） | ✅ VelocityTrackingYaw で速度方向追従 |
| センサー情報 | ❌ 全障害物が見える | ✅ FoV/範囲/遮蔽でフィルタ |
| BackupCBF 効果 | ❌ 見えにくい | ✅ 遮蔽領域での減速が観察可能 |

**結論**: Double Integrator でも姿勢制御を追加すれば、FoV ありの BackupCBF シミュレーションは十分に可能。safe_control の実装がまさにそれを示している。
