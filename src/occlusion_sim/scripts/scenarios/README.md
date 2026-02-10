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
| `mode` | `di` | `di` or `unicycle` |
| `record_bag` | `true` | rosbag録画の有無 |
| `experiment_id` | タイムスタンプ | 実験ID (bag出力サブディレクトリ) |
| `bag_output_dir` | `/root/Gazebo_ws/experiments` | bag出力ルート |

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
| `corner_popout` | 壁の死角から障害物が飛び出す | 5x5m |

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
    'walls': [
        {
            'name': 'wall_1',
            'center': (5.0, 7.0),        # Gazebo box の中心
            'size': (3.0, 1.0, 1.0),     # Gazebo box のサイズ (x, y, z)
            'circles': [                  # Python sim 用の円近似 (衝突+遮蔽)
                (4.0, 6.5, 0.3),
                (5.0, 6.5, 0.3),
                (6.0, 6.5, 0.3),
            ],
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

### 3. Gazebo用 world ファイルを作成 (壁がある場合)

`worlds/my_world.world` に壁の box モデルを配置。`experiment_corner.world` を参考に。

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
├── walls[]      # 静的壁リスト (name, center, size, circles)
├── cbf          # CBFパラメータ (T_horizon, dt_backup, alpha)
├── seed         # 乱数シード (optional, default=42)
└── gazebo       # Gazebo固有設定 (world_file)
```
