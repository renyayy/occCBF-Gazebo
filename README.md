# Gazebo simulator (Occlusion-CBF)
## Run container
```bash
xhost +local:docker

docker run -itd \
--net=host \
-e DISPLAY \
--gpus all \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /home/renyayy/Dev/OccCBF/Gazebo_ws:/root/Gazebo_ws \
--name Gazebo_ws \
gazebo-ros2_nav2
```

## Setup (コンテナ初回)
```bash
cd /root/Gazebo_ws
git submodule update --init --recursive
# source .venv/bin/activate
pip install -e src/occlusion_sim/safe_control
export TURTLEBOT3_MODEL=burger
```

## Enter container
```bash
docker exec -it Gazebo_ws_tb3 bash
```

## Build
```bash
cd /root/Gazebo_ws
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

出力先ディレクトリ規約: `experiments/<platform>_<model>/<experiment_id>/`

| platform | model | 説明 |
|----------|-------|------|
| `python` | `di` | Python数値シミュレーション / Double Integrator |
| `gazebo` | `di` | Gazebo物理シミュレーション / Double Integrator |
| `gazebo` | `unicycle` | Gazebo / TurtleBot3 Unicycle |
| `real` | `unicycle` | 実機 / TurtleBot3 |

### 1a. Gazebo実験（シミュレーション + bag自動記録）
```bash
# multi obstacle シナリオ（デフォルト → experiments/gazebo_di/<id>/ に保存）
ros2 launch occlusion_sim experiment.launch.py experiment_id:=test_001

# single obstacle シナリオ
ros2 launch occlusion_sim experiment.launch.py experiment_id:=test_001 scenario:=single
```

### 1a'. DI vs Unicycle 比較実験（Corner Pop-out シナリオ）

壁の死角から障害物が飛び出すシナリオで、DI（理想モデル）と Unicycle（TurtleBot3）の挙動を比較する。

```bash
# DI モード（ホロノミックロボット）→ experiments/gazebo_di/<id>/
ros2 launch occlusion_sim experiment_comparison.launch.py mode:=di experiment_id:=corner_001

# Unicycle モード（TurtleBot3 Burger）→ experiments/gazebo_unicycle/<id>/
ros2 launch occlusion_sim experiment_comparison.launch.py mode:=unicycle experiment_id:=corner_001

# bag記録なし
ros2 launch occlusion_sim experiment_comparison.launch.py mode:=unicycle record_bag:=false
```

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `mode` | `di` | `di`=ホロノミック, `unicycle`=TurtleBot3 |
| `experiment_id` | タイムスタンプ | 実験ID（bag保存先サブディレクトリ名） |
| `record_bag` | `true` | rosbag自動記録の有無 |

**Unicycle モードの構成:**
- CBFコントローラ → `/di_cmd_vel` (DI座標系の速度指令)
- `cmd_vel_converter` → `/cmd_vel` (Unicycle入力 $v, \omega$ に変換)
- 角度誤差が大きい場合はその場旋回し、向きが揃ってから前進する

### 1b. Python数値シミュレーション
```bash
python3 src/occlusion_sim/analysis/run_numerical_sim.py -o experiments/python_di/test_001

# シミュレーション時間を指定
python3 src/occlusion_sim/analysis/run_numerical_sim.py -o experiments/python_di/test_001 --tf 120
```

CSV + `result.json`（outcome/duration）が出力される。

### 2. 解析（プロット生成）
```bash
# Gazebo bag
python3 src/occlusion_sim/analysis/plot_experiment.py experiments/gazebo_di/test_001

# Python sim CSV
python3 src/occlusion_sim/analysis/plot_experiment.py experiments/python_di/test_001/cbf_debug.csv

# 出力先を指定
python3 src/occlusion_sim/analysis/plot_experiment.py <bag or csv> --output <output_dir>
```

rosbag/CSV を自動判定。結果判定（goal_reached / collision / timeout）付き。

| ファイル | 内容 |
|---------|------|
| `h_trajectory.png` | CBF安全性関数 h(x) の時系列 |
| `min_distance.png` | 障害物との最小距離の時系列 |
| `tracking_error.png` | 制御追従誤差 \|\|u - u_ref\|\| |

### 3. 比較（Python sim vs Gazebo）
```bash
python3 src/occlusion_sim/analysis/compare_experiments.py \
  experiments/python_di/test_001/cbf_debug.csv \
  experiments/gazebo_di/test_001 \
  --labels "Python Sim" "Gazebo" \
  --output experiments/comparison/di_test_001
```

| ファイル | 内容 |
|---------|------|
| `h_comparison.png` | h(x) 重ね描画 |
| `distance_comparison.png` | 最小距離 重ね描画 |
| `tracking_error_comparison.png` | 制御追従誤差 重ね描画 |
| `trajectory_comparison.png` | XY軌跡 重ね描画 |

### 4. 記録トピック（Gazebo bag）

| トピック | 型 | 内容 |
|---------|---|------|
| `/cmd_vel` | Twist | 制御入力指令値 |
| `/odom` | Odometry | ロボットオドメトリ |
| `/obstacle/state` | Odometry | 障害物状態 |
| `/cbf_debug_info` | Float64MultiArray | CBF内部値（下表参照） |
| `/tf` | TFMessage | 座標変換 |

#### `/cbf_debug_info` フィールドレイアウト（Gazebo bag / Python CSV 共通）

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
