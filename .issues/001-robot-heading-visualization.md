---
title: "ロボット・障害物の進行方向可視化が未実装"
category: architecture
severity: medium
status: open
created: 2026-01-22
---

## 概要

Gazeboシミュレーションでは、エゴロボットと動的障害物の進行方向（向き）が可視化されていない。safe_controlでは赤い線分や矢印で進行方向を表示しているが、現在のGazebo実装では常に同じ向きを向いている。

## 該当箇所

- `src/occlusion_sim/scripts/cbf_wrapper_node.py` - エゴロボット制御
- `src/occlusion_sim/scripts/multi_obstacle_controller.py` - 障害物制御
- `src/occlusion_sim/urdf/simple_holonomic_robot.urdf` - ロボットモデル

## 問題の詳細

### 現状
- Double Integrator（ホロノミック）モデルを使用
- 位置制御のみで向き制御は未実装
- GazeboのURDFモデルは回転しない

### safe_controlでの実装方法

1. **ロボット向き制御**: ステートマシン（stop → rotate → track）で目標方向への回転を実施
   ```python
   # main.py:257-264
   if self.state_machine == 'rotate':
       goal_angle = np.arctan2(self.goal[1] - self.robot.X[1, 0],
                               self.goal[0] - self.robot.X[0, 0])
       self.u_att = self.robot.rotate_to(goal_angle)
   ```

2. **向きの独立管理**: 位置制御と向き制御を分離し、yaw角を別変数で管理
   ```python
   # robots/double_integrator2D.py:113-115
   def step_rotate(self, theta, U_attitude):
       theta = angle_normalize(theta + U_attitude[0, 0] * self.dt)
       return theta
   ```

3. **視覚化**: 赤い線分で進行方向を表示
   ```python
   # robots/robot.py:489-492
   self.axis.set_xdata([self.X[0, 0], self.X[0, 0] + vis_orient_len*np.cos(self.yaw)])
   self.axis.set_ydata([self.X[1, 0], self.X[1, 0] + vis_orient_len*np.sin(self.yaw)])
   ```

4. **障害物向き**: 速度ベクトル(vx, vy)からarctan2で自動計算し、オレンジ矢印で表示

## 影響

- FoVの可視化はロボットのyawに連動しているが、Gazeboでは常に初期向きのまま
- safe_controlとの視覚的な一貫性がない
- 動的障害物の移動方向が視覚的にわからない

## 改善提案

### 案1: RViz2でのみ向きを可視化
- sensor_visualizer_node.pyで速度ベクトルから向きを計算
- Arrow Markerで進行方向を表示
- Gazeboモデルは変更不要

### 案2: Gazeboモデルの回転制御を追加
- URDFに方向表示用のリンク追加
- cbf_wrapper_node.pyでyaw角を計算・publish
- GazeboのモデルをTwist.angular.zで回転制御

### 案3: URDFに矢印モデルを追加
- simple_holonomic_robot.urdfに矢印形状を追加
- 速度ベクトル方向に回転するジョイントを追加
