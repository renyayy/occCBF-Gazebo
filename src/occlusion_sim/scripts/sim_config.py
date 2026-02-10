#!/usr/bin/env python3
"""Shared simulation parameters for occlusion_sim."""

# --- Environment ---
ENV_X_MIN, ENV_X_MAX = 0.0, 24.0
ENV_Y_MIN, ENV_Y_MAX = 1.0, 14.0

# --- Robot ---
ROBOT_RADIUS = 0.25
ROBOT_V_MAX = 1.0
ROBOT_A_MAX = 1.0
SENSING_RANGE = 10.0

# --- Obstacle ---
OBSTACLE_RADIUS = 0.3
OBSTACLE_V_MAX = 0.5

# --- Control ---
DT = 0.05  # 20Hz

# --- Default Goal/Start ---
DEFAULT_GOAL = (20.0, 7.5)
DEFAULT_START = (1.0, 7.5)

# --- Collision ---
COLLISION_DIST = ROBOT_RADIUS + OBSTACLE_RADIUS
GOAL_THRESHOLD = 0.3

# --- Backup CBF ---
BACKUP_CBF_PARAMS = {'T_horizon': 2.0, 'dt_backup': 0.05, 'alpha': 1.0}


def make_robot_spec(v_max=ROBOT_V_MAX, a_max=ROBOT_A_MAX):
    """robot_spec dict を生成（cbf_wrapper / sensor_visualizer 共用）"""
    return {
        'model': 'DoubleIntegrator2D',
        'v_max': v_max, 'a_max': a_max,
        'radius': ROBOT_RADIUS,
        'sensing_range': SENSING_RANGE,
        'backup_cbf': dict(BACKUP_CBF_PARAMS),
    }
