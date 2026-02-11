# Parameter Reference

## Ego Robot

| param | di / unicycle | unicycle-tb3 | 定義 |
|-------|:------------:|:------------:|------|
| radius | 0.25 m | 0.105 m | `sim_config.py` |
| v_max | 1.0 m/s | 0.22 m/s | `sim_config.py` |
| a_max | 1.0 m/s² | 1.0 m/s² | `sim_config.py` |
| sensing_range | 10.0 m | 10.0 m | `sim_config.py` |
| max_omega | - | 2.84 rad/s | `sim_config.py` (TB3のみ) |
| URDF | holonomic (紫) / unicycle (橙) | TB3 SDF | `urdf/` |

## Control

| param | value | 定義 |
|-------|-------|------|
| dt | 0.05 s (20 Hz) | `sim_config.py` |
| goal_threshold | 0.3 m | `sim_config.py` |
| controller | BackupCBFQP | `cbf_wrapper_node.py:68` |
| T_horizon | 2.0 s | `sim_config.py` BACKUP_CBF_PARAMS |
| dt_backup | 0.05 s | `sim_config.py` BACKUP_CBF_PARAMS |
| alpha | 1.0 | `sim_config.py` BACKUP_CBF_PARAMS |

## Collision Distance

```
d_min = r_robot + r_obs
h(x) = ||p_rel||² - d_min²    (h > 0 = safe)
```

| scenario | r_robot | r_obs | d_min |
|----------|---------|-------|-------|
| multi_random (di/uni) | 0.25 | 0.30 | 0.55 m |
| multi_random (tb3) | 0.105 | 0.30 | 0.405 m |
| corner_popout (di/uni) | 0.25 | 0.25 | 0.50 m |
| corner_popout (tb3) | 0.105 | 0.25 | 0.355 m |

計算箇所: `backup_cbf_qp.py:230`, `double_integrator2D.py:173`, `cbf_wrapper_node.py:193`

## Scenarios

### multi_random

| param | value |
|-------|-------|
| field | 24 x 13 m (X:[0,24], Y:[1,14]) |
| start | (1.0, 7.5) |
| goal | (20.0, 7.5) |
| world | `multi_obstacle.world` |
| obstacles | 5 x random_walk |
| obs radius | 0.3 m |
| obs v_max | 0.5 m/s |
| seed | 42 |

### corner_popout

| param | value |
|-------|-------|
| field | 5 x 5 m (X:[0,5], Y:[0,5]) |
| start | (0.5, 2.5) |
| goal | (4.5, 2.5) |
| world | `experiment_corner.world` |
| obstacles | 1 x waypoint, 4 x static |
| obs radius | 0.25 m |
| obs v_max | 0.3 m/s (waypoint), 0.0 (static) |
| waypoints | (3.0,4.0) → (3.0,3.0) → (2.5,2.5) → (0.5,2.5) |
| static walls | x=2.5, y=3.5/4.0/4.5/5.0 (縦一列) |

## Gazebo Physics

| param | experiment_corner | multi_obstacle |
|-------|:-----------------:|:--------------:|
| solver | ODE quick | ODE (default) |
| max_step_size | 0.004 s | 0.001 s |
| real_time_update_rate | 250 Hz | 1000 Hz |
| gravity | -9.8066 | -9.8 |
| planar_move update_rate | 100 Hz | 100 Hz |

## Parameter Flow

```
scenarios/*.py
  ├→ launch: re.sub で SDF <radius> 置換 → Gazebo 物理モデル
  ├→ launch: ROS param → cbf_wrapper_node (robot_radius, v_max, a_max, scenario_name)
  ├→ launch: ROS param → sensor_visualizer_node (robot_radius, robot_model, scenario_name)
  └→ launch: ROS param → scenario_obstacle_controller (scenario_name)

cbf_wrapper_node
  ├→ make_robot_spec(radius=...) → robot_spec['radius'] → backup_cbf_qp (d_min計算)
  └→ load_scenario() → _obs_radius_map[name] → 障害物ごとの半径
```
