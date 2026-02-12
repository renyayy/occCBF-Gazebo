# Gazebo simulator (Occlusion-CBF)

Occlusion-aware CBF シミュレータ。エゴロボットがCBF-QP最適化で動的障害物を回避しつつゴールへ移動する。

## Setup

### コンテナ起動
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

### 初回セットアップ
```bash
docker exec -it Gazebo_ws_tb3 bash
cd /root/Gazebo_ws
git submodule update --init --recursive
pip install -e src/occlusion_sim/safe_control
export TURTLEBOT3_MODEL=burger
```

### ビルド
```bash
colcon build --packages-select occlusion_sim --symlink-install
source install/setup.bash
```

## 実行

```bash
ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=<scenario> mode:=<mode>
```

### モード

| mode | ロボット | radius | v_max | 外観 |
|------|---------|--------|-------|------|
| `di` (default) | Double Integrator (ホロノミック) | 0.25m | 1.0 m/s | 紫円柱 |
| `unicycle` | Unicycle (DI同一パラメータ) | 0.25m | 1.0 m/s | オレンジ円柱 |
| `unicycle-tb3` | TurtleBot3 Burger (実機制約) | 0.105m | 0.22 m/s | TB3メッシュ |

`di` と `unicycle` はパラメータが統一されており、運動モデルのみ異なる。`unicycle-tb3` は TurtleBot3 の実機制約を使用。

### シナリオ

| scenario | 説明 | フィールド |
|----------|------|-----------|
| `corner_popout` (default) | 死角から障害物が飛び出す | 5×5m |

追加方法: [scenarios/README.md](src/occlusion_sim/scripts/scenarios/README.md)

### 実行例
```bash
# DI モード (デフォルト: corner_popout)
ros2 launch occlusion_sim gazebo_sim.launch.py

# Unicycle モード
ros2 launch occlusion_sim gazebo_sim.launch.py mode:=unicycle

# TurtleBot3 モード
ros2 launch occlusion_sim gazebo_sim.launch.py mode:=unicycle-tb3

# bag録画なし / 実験ID指定
ros2 launch occlusion_sim gazebo_sim.launch.py record_bag:=false experiment_id:=test_001
```

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `scenario` | `corner_popout` | シナリオ名 |
| `mode` | `di` | `di` / `unicycle` / `unicycle-tb3` |
| `record_bag` | `true` | rosbag自動記録 |
| `experiment_id` | タイムスタンプ | bag保存先サブディレクトリ名 |

## コントローラ選択

`cbf_wrapper_node.py` 内でコメントアウトを切り替え:

| コントローラー | 特徴 |
|--------------|------|
| CBFQP | 基本CBF-QP |
| **BackupCBFQP** | 遮蔽対応 (デフォルト) |
| MPCCBF | MPC+CBF |
| OptimalDecayCBFQP | 安全マージン動的調整 |
| OptimalDecayMPCCBF | MPC版最適減衰 |

## 実験・解析

出力先: `experiments/<platform>_<model>/<experiment_id>/`

| platform | model | 説明 |
|----------|-------|------|
| `python` | `di` | Python数値シミュレーション |
| `gazebo` | `di` | Gazebo / DI |
| `gazebo` | `unicycle` | Gazebo / Unicycle |
| `gazebo` | `unicycle_tb3` | Gazebo / TurtleBot3 |

### Python数値シミュレーション
```bash
python3 src/occlusion_sim/analysis/run_numerical_sim.py --scenario corner_popout -o experiments/python_di/corner_001
```

### プロット生成
```bash
# rosbag (シナリオからgoal・半径を自動取得)
python3 src/occlusion_sim/analysis/plot_experiment.py experiments/gazebo_di/test_001

# CSV
python3 src/occlusion_sim/analysis/plot_experiment.py experiments/python_di/test_001/cbf_debug.csv

# シナリオ・半径を明示指定する場合
python3 src/occlusion_sim/analysis/plot_experiment.py experiments/gazebo_unicycle_tb3/test_001 --robot-radius 0.105
```

出力: `h_trajectory.png`, `min_distance.png`, `tracking_error.png`, `trajectory.png`

### バッチ実験 (`run_batch_experiment.py`)

複数モード・複数回の実験を連続実行し、結果を集計する。

```bash
# di vs unicycle を各3回（統計比較）
python3 src/occlusion_sim/scripts/run_batch_experiment.py --modes di,unicycle --runs 3

# モード比較（1回ずつ）
python3 src/occlusion_sim/scripts/run_batch_experiment.py --modes di,unicycle

# 実験ID指定 + 複数モード × 5回
python3 src/occlusion_sim/scripts/run_batch_experiment.py --modes di,unicycle --runs 5 --experiment-id stat_001
```

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `--scenario` | `corner_popout` | シナリオ名 |
| `--modes` | `di,unicycle` | カンマ区切りのモードリスト |
| `--runs` | `1` | 各モードの実行回数 |
| `--experiment-id` | `batch_<timestamp>` | 実験ID |
| `--timeout` | `30.0` | シミュレーションタイムアウト(秒) |
| `--gui` | off | Gazebo GUI表示 |

`--runs 1` は各実験後に `plot_experiment.py` を自動実行。`--runs > 1` はプロットをスキップし、集計統計（成功率・衝突率・平均所要時間±標準偏差）を表示する。結果は `experiments/<experiment_id>_summary.csv` に出力。

出力ディレクトリ構造:
```
# --runs 1
experiments/gazebo_di/<experiment_id>/

# --runs N>1
experiments/gazebo_di/<experiment_id>/run_001/
experiments/gazebo_di/<experiment_id>/run_002/
...
experiments/<experiment_id>_summary.csv
```

### 比較
```bash
python3 src/occlusion_sim/analysis/compare_experiments.py \
  experiments/python_di/test_001/cbf_debug.csv \
  experiments/gazebo_di/test_001 \
  --labels "Python Sim" "Gazebo" \
  --output experiments/comparison/di_test_001
```
