"""Multi Random Walk scenario - 5 obstacles with random walk behavior (24x13m)."""

SCENARIO = {
    'env': {
        'x_min': 0.0, 'x_max': 24.0,
        'y_min': 1.0, 'y_max': 14.0,
    },
    'robot': {
        'start': (1.0, 7.5),
        'goal': (20.0, 7.5),
        'radius': 0.25,
        'v_max': 1.0,
        'a_max': 1.0,
        'sensing_range': 10.0,
    },
    'obstacles': [
        {'name': 'obs_0', 'position': (8.0, 5.0), 'radius': 0.3, 'v_max': 0.5, 'behavior': 'random_walk'},
        {'name': 'obs_1', 'position': (10.0, 9.0), 'radius': 0.3, 'v_max': 0.5, 'behavior': 'random_walk'},
        {'name': 'obs_2', 'position': (12.0, 3.0), 'radius': 0.3, 'v_max': 0.5, 'behavior': 'random_walk'},
        {'name': 'obs_3', 'position': (14.0, 11.0), 'radius': 0.3, 'v_max': 0.5, 'behavior': 'random_walk'},
        {'name': 'obs_4', 'position': (16.0, 7.0), 'radius': 0.3, 'v_max': 0.5, 'behavior': 'random_walk'},
    ],
    'walls': [],
    'cbf': {
        'T_horizon': 2.0,
        'dt_backup': 0.05,
        'alpha': 1.0,
    },
    'seed': 42,
    'gazebo': {
        'world_file': 'multi_obstacle.world',
    },
}
