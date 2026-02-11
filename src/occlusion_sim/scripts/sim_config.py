#!/usr/bin/env python3
"""Shared simulation parameters for occlusion_sim."""
import os as _os, sys as _sys

_safe_control = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'safe_control')
_safe_control = _os.path.abspath(_safe_control)
if _safe_control not in _sys.path:
    _sys.path.insert(0, _safe_control)

# --- Environment ---
ENV_X_MIN, ENV_X_MAX = 0.0, 24.0
ENV_Y_MIN, ENV_Y_MAX = 1.0, 14.0

# --- Robot (DI / Unicycle 共通) ---
ROBOT_RADIUS = 0.25
ROBOT_V_MAX = 1.0
ROBOT_A_MAX = 1.0
SENSING_RANGE = 10.0

# --- TurtleBot3 Burger ---
TB3_ROBOT_RADIUS = 0.105
TB3_V_MAX = 0.22
TB3_A_MAX = 1.0
TB3_MAX_OMEGA = 2.84

# --- Obstacle ---
OBSTACLE_RADIUS = 0.3
OBSTACLE_V_MAX = 0.5

# --- Control ---
DT = 0.05  # 20Hz

# --- Default Goal/Start ---
DEFAULT_GOAL = (20.0, 7.5)
DEFAULT_START = (1.0, 7.5)

# --- Collision ---
GOAL_THRESHOLD = 0.3

# --- Backup CBF ---
BACKUP_CBF_PARAMS = {'T_horizon': 2.0, 'dt_backup': 0.05, 'alpha': 1.0}


def collision_dist(robot_radius, obstacle_radius):
    return robot_radius + obstacle_radius


def make_robot_spec(v_max=ROBOT_V_MAX, a_max=ROBOT_A_MAX, radius=ROBOT_RADIUS):
    """robot_spec dict を生成（cbf_wrapper / sensor_visualizer 共用）"""
    return {
        'model': 'DoubleIntegrator2D',
        'v_max': v_max, 'a_max': a_max,
        'radius': radius,
        'sensing_range': SENSING_RANGE,
        'backup_cbf': dict(BACKUP_CBF_PARAMS),
    }
