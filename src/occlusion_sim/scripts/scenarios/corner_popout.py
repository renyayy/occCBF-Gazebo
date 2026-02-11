"""Corner Pop-out scenario - dynamic obstacle crosses ego path perpendicularly (5x5m).

References: .docs/plans/実験環境.md
- 5m x 5m field
- Dynamic obstacle follows waypoints from upper-right toward ego start
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
            'position': (3.0, 2.5),
            'radius': 0.25,
            'v_max': 0.3,
            'behavior': 'waypoint',
            'waypoints': [
                (3.0, 2.5),    # Start above ego path midpoint
                (3.0, 2.0),    # Cross ego path perpendicularly
                (3.0, 1.5),    # Continue below
                (3.0, 1.3),    # Move aside
                (2.8, 1.2),    # Move aside
                (2.8, 1.0),    # Move aside
                (1.0, 1.0),    # Move aside
                (0.5, 1.0),    # Move aside
            ],
        },
        {'name': 'wall_0', 'position': (2.5, 2.0), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
        {'name': 'wall_1', 'position': (2.5, 2.5), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
        {'name': 'wall_2', 'position': (2.5, 3.0), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
        {'name': 'wall_3', 'position': (2.5, 3.5), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
        {'name': 'wall_4', 'position': (2.5, 4.0), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
        {'name': 'wall_5', 'position': (2.5, 4.5), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
        {'name': 'wall_6', 'position': (2.5, 5.0), 'radius': 0.25, 'v_max': 0.0, 'behavior': 'static'},
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
