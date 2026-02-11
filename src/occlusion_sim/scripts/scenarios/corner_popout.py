"""Corner Pop-out scenario - dynamic obstacle emerges from behind a wall (5x5m).

References: .docs/plans/実験環境.md
- 5m x 5m field
- Wall at x:[2.5,4.5], y:[1.5,3.5] creates occlusion
- Dynamic obstacle starts behind wall, follows waypoints toward ego
"""

SCENARIO = {
    'env': {
        'x_min': 0.0, 'x_max': 5.0,
        'y_min': 0.0, 'y_max': 5.0,
    },
    'robot': {
        'start': (0.5, 1.0),
        'goal': (4.5, 1.0),
        'radius': 0.25,
        'v_max': 1.0,
        'a_max': 1.0,
        'sensing_range': 10.0,
    },
    'obstacles': [
        {
            'name': 'obs_0',
            'position': (3.0, 3.7),
            'radius': 0.15,
            'v_max': 0.3,
            'behavior': 'waypoint',
            'waypoints': [
                (3.0, 3.7),    # Behind wall (above wall y<3.5)
                (2.3, 2.5),    # Down along wall left edge
                (2.3, 1.3),    # Emerge below wall (pop-out!)
                (2.0, 1.0),    # Cross ego nominal path
                (1.5, 0.8),    # Arc below nominal path
                (1.0, 0.9),    # Rising back
                (0.5, 1.0),    # Ego start position (stop)
            ],
        },
    ],
    'walls': [
        {
            'name': 'corner_wall',
            'center': (3.5, 2.5),
            'size': (2.0, 2.0, 1.0),
            # Python sim用の円近似: 壁のエゴ側の辺に沿って配置 (衝突+遮蔽)
            'circles': [
                # 下辺 (y=1.5)
                (2.5, 1.5, 0.3), (3.0, 1.5, 0.3), (3.5, 1.5, 0.3), (4.0, 1.5, 0.3), (4.5, 1.5, 0.3),
                # 左辺 (x=2.5)
                (2.5, 2.0, 0.3), (2.5, 2.5, 0.3), (2.5, 3.0, 0.3), (2.5, 3.5, 0.3),
            ],
        },
    ],
    'cbf': {
        'T_horizon': 2.0,
        'dt_backup': 0.05,
        'alpha': 1.0,
    },
    'gazebo': {
        'world_file': 'experiment_corner.world',
    },
}
