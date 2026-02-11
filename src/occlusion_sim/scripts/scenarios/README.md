# Scenarios

Python / Gazebo 共通のシナリオ定義。各シナリオファイルが `SCENARIO` dict をエクスポートする。

## 使い方

### Gazebo シミュレーション

```bash
# DI (Double Integrator) モデル
ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout
ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=multi_random

# Unicycle (TurtleBot3 Burger) モデル
ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout mode:=unicycle

# rosbag録画なし
ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout record_bag:=false

# 実験ID指定
ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout experiment_id:=test_001
```

### Launch 引数一覧

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `scenario` | `multi_random` | シナリオ名 |
| `mode` | `di` | `di` / `unicycle` / `unicycle-tb3` |
| `record_bag` | `true` | rosbag録画の有無 |
| `experiment_id` | タイムスタンプ | 実験ID (bag出力サブディレクトリ) |
| `bag_output_dir` | `/root/Gazebo_ws/experiments` | bag出力ルート |
| `auto_shutdown` | `false` | ゴール到達/衝突/タイムアウトで自動終了 |
| `sim_timeout` | `30.0` | 自動終了のタイムアウト秒数 (sim time) |
| `gui` | `true` | `false` で RViz2 を起動しない |

### バッチ実験 (di vs unicycle 自動比較)

`run_batch_experiment.py` で複数モードを順次実行し、結果を比較できる。

```bash
# 基本 (di → unicycle を順次実行、各30秒タイムアウト、GUI無し)
python3 src/occlusion_sim/scripts/run_batch_experiment.py \
  --scenario corner_popout --timeout 30

# 実験ID指定・GUI有り
python3 src/occlusion_sim/scripts/run_batch_experiment.py \
  --scenario corner_popout --timeout 60 --experiment-id exp_001 --gui

# モード指定 (unicycle-tb3 を含める)
python3 src/occlusion_sim/scripts/run_batch_experiment.py \
  --scenario corner_popout --modes di,unicycle,unicycle-tb3

# multi_random シナリオ
python3 src/occlusion_sim/scripts/run_batch_experiment.py \
  --scenario multi_random --timeout 120
```

**CLI引数:**

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `--scenario` | `corner_popout` | シナリオ名 |
| `--experiment-id` | `batch_<timestamp>` | 実験ID (全モード共通) |
| `--timeout` | `30.0` | sim time タイムアウト秒数 |
| `--modes` | `di,unicycle` | カンマ区切りのモード一覧 |
| `--gui` | off | 指定すると RViz2 を表示 |
| `--experiments-dir` | `/root/Gazebo_ws/experiments` | 出力ルート |

**出力ディレクトリ構造:**

```
experiments/
├── gazebo_di/<experiment_id>/
│   ├── result.json           # {"outcome": "goal_reached", "duration": 10.8}
│   ├── <experiment_id>_0.db3 # rosbag
│   ├── metadata.yaml
│   ├── h_trajectory.png
│   ├── min_distance.png
│   └── tracking_error.png
└── gazebo_unicycle/<experiment_id>/
    ├── result.json           # {"outcome": "collision", "duration": 6.8}
    └── ...
```

**自動終了条件** (`auto_shutdown:=true` 時):

| 条件 | 判定 |
|------|------|
| ゴール到達 | ゴールまでの距離 < 0.3m |
| 衝突 | 障害物との中心間距離 < robot_radius + obs_radius |
| タイムアウト | sim time 経過 > sim_timeout |

### Python 数値シミュレーション

```bash
python3 src/occlusion_sim/analysis/run_numerical_sim.py \
  --scenario corner_popout \
  -o experiments/python_di/corner_001

python3 src/occlusion_sim/analysis/run_numerical_sim.py \
  --scenario multi_random \
  -o experiments/python_di/multi_001
```

## 利用可能なシナリオ

| 名前 | 説明 | フィールド |
|------|------|-----------|
| `multi_random` | 5障害物ランダムウォーク | 24x13m |
| `corner_popout` | 障害物がエゴ経路を斜めに横切る | 5x5m |

## 新しいシナリオの作り方

### 1. シナリオ定義ファイルを作成

`scripts/scenarios/my_scenario.py`:

```python
SCENARIO = {
    'env': {
        'x_min': 0.0, 'x_max': 10.0,
        'y_min': 0.0, 'y_max': 10.0,
    },
    'robot': {
        'start': (1.0, 5.0),
        'goal': (9.0, 5.0),
        'radius': 0.25,
        'v_max': 1.0,
        'a_max': 1.0,
        'sensing_range': 10.0,
    },
    'obstacles': [
        {
            'name': 'obs_0',
            'position': (5.0, 5.0),     # 初期位置
            'radius': 0.3,
            'v_max': 0.5,
            'behavior': 'random_walk',   # random_walk | chase | waypoint | static
        },
    ],
    'cbf': {
        'T_horizon': 2.0,
        'dt_backup': 0.05,
        'alpha': 1.0,
    },
    'seed': 42,                           # random_walk の再現性用
    'gazebo': {
        'world_file': 'my_world.world',   # worlds/ 内のファイル名
    },
}
```

### 2. レジストリに登録

`scripts/scenarios/__init__.py` に追加:

```python
from scenarios import my_scenario
SCENARIOS['my_scenario'] = my_scenario.SCENARIO
```

### 3. Gazebo用 world ファイルを作成

`worlds/my_world.world` を作成。`experiment_corner.world` を参考に。

### 4. CMakeLists.txt は変更不要

`scripts/scenarios/` ディレクトリ全体がインストールされるため、新ファイルの追加登録は不要。

## 障害物 behavior 一覧

| behavior | Gazebo | Python sim | 説明 |
|----------|--------|------------|------|
| `random_walk` | 反射境界付きランダムウォーク | `step_dyn_obs` mode=1 | 5%の確率で大きな方向転換 |
| `chase` | エゴロボットの `/odom` を追従 | 毎ステップ速度更新 | 常にエゴ方向へ移動 |
| `waypoint` | ウェイポイント順次追従 | 毎ステップ速度更新 | 最終WP到達で停止 |
| `static` | 静止 | vx=vy=0 | CBF制約のみに影響 |

## config 構造

```
SCENARIO
├── env          # 環境範囲 (x_min, x_max, y_min, y_max)
├── robot        # エゴロボット (start, goal, radius, v_max, a_max, sensing_range)
├── obstacles[]  # 動的障害物リスト (name, position, radius, v_max, behavior, ...)
├── cbf          # CBFパラメータ (T_horizon, dt_backup, alpha)
├── seed         # 乱数シード (optional, default=42)
└── gazebo       # Gazebo固有設定 (world_file)
```
