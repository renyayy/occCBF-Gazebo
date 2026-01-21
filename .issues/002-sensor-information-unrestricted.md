---
title: "センサー情報が制限されていない（全障害物情報を取得可能）"
category: architecture
severity: high
status: open
created: 2026-01-22
---

## 概要

FoV、センサーリミット、オクルージョンの可視化は実装されているが、CBF制御では実際にはすべての障害物の位置・速度情報を取得している。センサー範囲やオクルージョンによる情報制限が制御に反映されていない。

## 該当箇所

- `src/occlusion_sim/scripts/cbf_wrapper_node.py:52, 65-74`
- `src/occlusion_sim/scripts/multi_obstacle_controller.py:88-95`

## 問題の詳細

### 現状のデータフロー
```
multi_obstacle_controller.py
    ↓ publish: /obstacle/state (全障害物の位置・速度)
cbf_wrapper_node.py
    ↓ subscribe: /obstacle/state
    ↓ self.obs_list に全障害物を格納
    ↓ controller.solve_control_problem(X, u_ref, obs_list)  # 全障害物を渡す
```

### コード例
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

### センサー制限が必要な場面
1. **センサーリミット外**: sensing_range=10m 外の障害物は検知不可
2. **FoV外**: fov_angle=70° 外の障害物はカメラで検知不可
3. **オクルージョン**: 他の障害物の影に隠れた障害物は検知不可

## 影響

- 可視化と実際の制御に不整合がある
- 現実のセンサー制約を反映したCBF制御の評価ができない
- safe_controlの「オクルージョン対応」機能の検証ができない

## 改善提案

### cbf_wrapper_node.pyに以下のフィルタリングを追加

```python
def filter_observable_obstacles(self, obs_list):
    robot_pos = self.X[:2, 0]
    robot_yaw = self.get_robot_yaw()

    observable = []
    for obs in obs_list:
        ox, oy = obs[0], obs[1]
        dist = np.hypot(ox - robot_pos[0], oy - robot_pos[1])

        # 1. センサーリミットチェック
        if dist > self.robot_spec['sensing_range']:
            continue

        # 2. FoVチェック（カメラ使用時）
        angle_to_obs = np.arctan2(oy - robot_pos[1], ox - robot_pos[0])
        angle_diff = angle_normalize(angle_to_obs - robot_yaw)
        if abs(angle_diff) > self.fov_angle / 2:
            continue

        # 3. オクルージョンチェック
        if self.is_occluded(robot_pos, obs, observable):
            continue

        observable.append(obs)

    return np.array(observable)
```

### 注意点
- BackupCBFQPはオクルージョン対応済みだが、入力データ自体がフィルタリングされていない
- センサー制限を有効にすると、未検知障害物との衝突リスクが発生する（これがBackupCBFの意義）
