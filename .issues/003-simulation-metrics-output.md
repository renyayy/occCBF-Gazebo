---
title: "シミュレーション終了判定とメトリクス出力機能の追加"
category: architecture
severity: medium
status: open
created: 2026-01-22
---

## 概要

ゴール到達時や障害物との接触時にシミュレーションを自動終了し、評価メトリクスをファイル出力する機能が未実装。現状ではゴール到達時に停止するのみで、結果の記録や衝突検知がない。

## 該当箇所

- `src/occlusion_sim/scripts/cbf_wrapper_node.py:80-84` - ゴール到達判定（停止のみ）

## 現状の実装

```python
# cbf_wrapper_node.py:80-84
dist = np.hypot(self.goal[0, 0] - self.X[0, 0], self.goal[1, 0] - self.X[1, 0])
if dist < 0.3:
    self.cmd_pub.publish(Twist())  # 停止するのみ
    return
```

## 必要な機能

### 1. 終了条件
- **ゴール到達**: 現状の `dist < 0.3` を使用
- **衝突検知**: エゴロボットと障害物の距離が `robot_radius + obstacle_radius` 未満

### 2. メトリクス

| メトリクス | 説明 | 計算方法 |
|-----------|------|---------|
| Success rate | ゴール到達率 | goal_reached / total_trials |
| Number of collisions | 衝突回数 | 障害物との接触をカウント |
| Time to goal | ゴール到達時間 | シミュレーション開始からの経過時間 |
| Minimum distance to obstacles | 最小障害物距離 | 全時刻での最小距離を記録 |
| Control input smoothness | 制御入力の滑らかさ | Σ\|u(t) - u(t-1)\|² / N |

### 3. 出力フォーマット例

```yaml
# results/simulation_2026-01-22_12-00-00.yaml
simulation:
  start_time: "2026-01-22T12:00:00"
  end_time: "2026-01-22T12:01:30"
  duration_sec: 90.0
  termination_reason: "goal_reached"  # or "collision"

metrics:
  success: true
  num_collisions: 0
  time_to_goal_sec: 85.3
  min_distance_to_obstacles: 0.42
  control_smoothness: 0.0023

parameters:
  controller: "BackupCBFQP"
  robot_radius: 0.25
  obstacle_radius: 0.25
  sensing_range: 10.0
  num_obstacles: 5
```

## 改善提案

### 新規ノード: simulation_supervisor_node.py

```python
class SimulationSupervisor(Node):
    def __init__(self):
        super().__init__('simulation_supervisor')

        # State
        self.start_time = self.get_clock().now()
        self.min_dist = float('inf')
        self.prev_u = np.zeros(2)
        self.smoothness_sum = 0.0
        self.control_count = 0
        self.collisions = 0

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        # Timer
        self.create_timer(0.05, self.check_termination)

    def check_termination(self):
        # ゴール到達チェック
        if self.check_goal_reached():
            self.terminate("goal_reached")

        # 衝突チェック
        if self.check_collision():
            self.collisions += 1
            self.terminate("collision")

    def terminate(self, reason):
        self.save_metrics(reason)
        # シャットダウンリクエスト
        rclpy.shutdown()
```

### 統合方法

1. `multi_obstacle_simulation.launch.py` に supervisor ノードを追加
2. 結果は `results/` ディレクトリに自動保存
3. 複数回実行のバッチ処理スクリプトを追加（オプション）
