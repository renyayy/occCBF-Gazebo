# Gazebo simulator (Occlusion-CBF)
## Enter container
```bash
xhost +local:docker

docker run -itd \
--net=host \
-e DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /home/renyayy/Dev/OccCBF/Gazebo_ws:/root/Gazebo_ws \
--name Gazebo_ws \
gazebo-ros2_nav2
```

## Build
```bash
cd ~/root/Gazebo_ws
colcon build --packages-select occlusion_sim --symlink-install
source install/setup.bash
```

## Execute
```bash
# Launch
ros2 launch occlusion_sim multi_obstacle_simulation.launch.py

# 個別実行
ros2 run occlusion_sim cbf_wrapper_node.py
```

## Select Controller 
`cbf_wrapper_node.py` 内でコメントアウトを切り替え:

```python
# Controller選択 (使用する Controller をコメント解除) 
self.controller = CBFQP(...)              # 基本CBF-QP
# self.controller = BackupCBFQP(...)      # バックアップCBF-QP (遮蔽対応)
# self.controller = MPCCBF(...)           # MPC-CBF
# self.controller = OptimalDecayCBFQP(...)# 最適減衰CBF-QP
# self.controller = OptimalDecayMPCCBF(...)# 最適減衰MPC-CBF
```

| コントローラー | 特徴 |
|--------------|------|
| CBFQP | 基本CBF-QP |
| BackupCBFQP | 遮蔽対応、動的障害物向け |
| MPCCBF | MPC+CBF |
| OptimalDecayCBFQP | 安全マージン動的調整 |
| OptimalDecayMPCCBF | MPC版最適減衰 |

## Experiment (データ収集 + 解析)

### 1. 実験実行（シミュレーション + bag自動記録）
```bash
# multi obstacle シナリオ（デフォルト）
ros2 launch occlusion_sim experiment.launch.py experiment_id:=test_001

# single obstacle シナリオ
ros2 launch occlusion_sim experiment.launch.py experiment_id:=test_001 scenario:=single
```

bag は `experiment_bags/<experiment_id>/` に保存される。

### 2. 解析（プロット生成）
```bash
python3 src/occlusion_sim/analysis/plot_experiment.py experiment_bags/<experiment_id>

# 出力先を指定する場合
python3 src/occlusion_sim/analysis/plot_experiment.py experiment_bags/<experiment_id> --output <output_dir>
```

デフォルトではbagディレクトリ内に以下のPNGが生成される:

| ファイル | 内容 |
|---------|------|
| `h_trajectory.png` | CBF安全性関数 h(x) の時系列 |
| `min_distance.png` | 障害物との最小距離の時系列 |
| `tracking_error.png` | 制御追従誤差 \|\|u - u_ref\|\| |
| `summary.png` | 上記3つ + ロボット軌跡の統合プロット |

### 3. 記録トピック

| トピック | 型 | 内容 |
|---------|---|------|
| `/cmd_vel` | Twist | 制御入力指令値 |
| `/odom` | Odometry | ロボットオドメトリ |
| `/obstacle/state` | Odometry | 障害物状態 |
| `/cbf_debug_info` | Float64MultiArray | CBF内部値（下表参照） |
| `/tf` | TFMessage | 座標変換 |

#### `/cbf_debug_info` フィールドレイアウト

| Index | フィールド | 内容 |
|-------|-----------|------|
| 0 | stamp_sec | タイムスタンプ(秒) |
| 1 | h_min | 最小CBF値 h(x) |
| 2 | min_dist | 最近傍障害物距離 |
| 3 | num_constraints | アクティブ制約数 |
| 4 | qp_solve_time_ms | QP計算時間(ms) |
| 5 | intervention | 0.0=u_ref, 1.0=backup_qp |
| 6-7 | u_x, u_y | 制御出力(加速度) |
| 8-9 | u_ref_x, u_ref_y | ノミナル制御入力 |
| 10-11 | robot_x, robot_y | ロボット位置 |
| 12-13 | robot_vx, robot_vy | ロボット速度 |
| 14 | status_ok | 1.0=optimal, 0.0=infeasible |
| 15 | num_visible_obs | 可視障害物数 |
| 16 | num_total_obs | 全障害物数 |
