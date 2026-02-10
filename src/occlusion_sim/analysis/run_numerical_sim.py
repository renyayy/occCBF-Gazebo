#!/usr/bin/env python3
"""Run Python numerical simulation and export CBF debug data to CSV.

Matches Gazebo multi_obstacle scenario for Phase 1 comparison.
"""
import argparse
import csv
import json
import math
import os
import random
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# sim_config 経由で safe_control パスを設定
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import sim_config  # noqa: E402

from dynamic_env.main import LocalTrackingControllerDyn
from utils import plotting, env


class DataCollectingController(LocalTrackingControllerDyn):
    """Subclass that logs per-step CBF debug data without modifying safe_control."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.data_log = []
        self._step_count = 0

    def control_step(self):
        pre_X = self.robot.X.copy()
        ret = super().control_step()
        self._log_step(pre_X)
        self._step_count += 1
        return ret

    def _log_step(self, pre_X):
        pc = self.pos_controller
        R_robot = self.robot_spec['radius']

        # h_min: h = ||p_rel||^2 - d_min^2 (backup_cbf_qp.py:232)
        h_min = float('inf')
        obs_for_cbf = self.nearest_multi_obs
        if obs_for_cbf is not None and len(obs_for_cbf) > 0:
            for obs in obs_for_cbf:
                p_rel = pre_X[:2, 0] - obs[:2]
                d_min = obs[2] + R_robot
                h_min = min(h_min, float(p_rel @ p_rel - d_min**2))

        # min_dist (center-to-center)
        if obs_for_cbf is not None and len(obs_for_cbf) > 0:
            dists = np.linalg.norm(obs_for_cbf[:, :2] - pre_X[:2, 0], axis=1)
            min_dist = float(np.min(dists))
        else:
            min_dist = float('inf')

        n_constraints = getattr(pc, 'last_num_constraints', 0) or 0
        qp_ms = getattr(pc, 'last_qp_solve_time_ms', 0.0) or 0.0
        intervention = getattr(pc, 'last_intervention', 'u_ref')
        last_u = getattr(pc, 'last_u', np.zeros((2, 1)))
        last_u_ref = getattr(pc, 'last_u_ref', np.zeros((2, 1)))
        status = getattr(pc, 'status', 'unknown')
        n_visible = len(obs_for_cbf) if obs_for_cbf is not None else 0

        self.data_log.append([
            self._step_count * self.dt,
            h_min, min_dist,
            float(n_constraints), float(qp_ms),
            1.0 if intervention == 'backup_qp' else 0.0,
            float(last_u[0, 0]), float(last_u[1, 0]),
            float(last_u_ref[0, 0]), float(last_u_ref[1, 0]),
            float(pre_X[0, 0]), float(pre_X[1, 0]),
            float(pre_X[2, 0]), float(pre_X[3, 0]),
            1.0 if status == 'optimal' else 0.0,
            float(n_visible), float(len(self.obs)),
        ])


CSV_HEADER = [
    'stamp_sec', 'h_min', 'min_dist', 'num_constraints', 'qp_solve_time_ms',
    'intervention', 'u_x', 'u_y', 'u_ref_x', 'u_ref_y',
    'robot_x', 'robot_y', 'robot_vx', 'robot_vy',
    'status_ok', 'num_visible_obs', 'num_total_obs',
]


def run_simulation(args):
    dt = 0.05

    robot_spec = {
        'model': 'DoubleIntegrator2D',
        'v_max': 1.0,
        'a_max': 1.0,
        'radius': 0.25,
        'sensing_range': 10.0,
        'backup_cbf': {'T_horizon': 2.0},
    }

    waypoints = np.array([[1.0, 7.5, 0.0], [20.0, 7.5, 0.0]])
    x_init = waypoints[0]

    # Obstacle setup matching Gazebo multi_obstacle scenario
    # Use Python random (same as multi_obstacle_controller.py) for initial headings
    obs_positions = [(8.0, 5.0), (10.0, 9.0), (12.0, 3.0), (14.0, 11.0), (16.0, 7.0)]
    obs_radius = 0.3
    v_max_obs = 0.5
    y_min, y_max = 1.0, 14.0

    random.seed(42)
    obs_data = []
    obs_meta = []
    for x, y in obs_positions:
        theta = random.uniform(-math.pi, math.pi)
        vx = v_max_obs * math.cos(theta)
        vy = v_max_obs * math.sin(theta)
        obs_data.append([x, y, obs_radius, vx, vy, y_min, y_max, 1])
        obs_meta.append({'mode': 1, 'v_max': v_max_obs, 'theta': theta})

    known_obs = np.array(obs_data, dtype=float)

    env_width, env_height = 24.0, 15.0
    plot_handler = plotting.Plotting(width=env_width, height=env_height, known_obs=known_obs)
    ax, fig = plot_handler.plot_grid("")
    env_handler = env.Env()

    controller = DataCollectingController(
        x_init, robot_spec,
        controller_type={'pos': 'backup_cbf_qp'},
        dt=dt,
        show_animation=False,
        save_animation=False,
        ax=ax, fig=fig,
        env=env_handler,
        rand_seed=42,
    )

    controller.obs = known_obs
    controller.set_obs_meta(obs_meta)
    controller.set_waypoints(waypoints)

    # Run simulation
    outcome = 'timeout'
    for _ in range(int(args.tf / dt)):
        ret = controller.control_step()
        if ret == -1:
            outcome = 'goal_reached'
            break
        elif ret == -2:
            outcome = 'collision'
            break

    # Save CSV
    os.makedirs(args.output, exist_ok=True)
    csv_path = os.path.join(args.output, 'cbf_debug.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        writer.writerows(controller.data_log)

    # Save result
    duration = len(controller.data_log) * dt
    result = {'outcome': outcome, 'duration': round(duration, 2),
              'total_steps': len(controller.data_log)}
    with open(os.path.join(args.output, 'result.json'), 'w') as f:
        json.dump(result, f, indent=2)

    print(f'Outcome: {outcome}')
    print(f'Duration: {duration:.2f}s ({len(controller.data_log)} steps)')
    print(f'CSV: {csv_path}')

    plt.close('all')


def main():
    parser = argparse.ArgumentParser(description='Run numerical simulation with data collection')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--tf', type=float, default=300.0, help='Simulation time limit (s)')
    args = parser.parse_args()
    run_simulation(args)


if __name__ == '__main__':
    main()
