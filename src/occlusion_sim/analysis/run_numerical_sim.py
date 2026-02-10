#!/usr/bin/env python3
"""Run Python numerical simulation and export CBF debug data to CSV.

Supports scenario-based configuration for unified Python/Gazebo experiments.
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

from scenarios import load_scenario, list_scenarios
from dynamic_env.main import LocalTrackingControllerDyn
from utils import plotting, env


class DataCollectingController(LocalTrackingControllerDyn):
    """Subclass that logs per-step CBF debug data without modifying safe_control."""

    def __init__(self, *args, scenario_obs_behaviors=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.data_log = []
        self._step_count = 0
        # シナリオ固有の障害物behavior情報 (waypoint / chase 用)
        self._scenario_obs_behaviors = scenario_obs_behaviors or []

    def control_step(self):
        pre_X = self.robot.X.copy()
        ret = super().control_step()
        self._update_scenario_obs()
        self._log_step(pre_X)
        self._step_count += 1
        return ret

    def _update_scenario_obs(self):
        """waypoint / chase behavior の障害物速度を毎ステップ更新"""
        for beh in self._scenario_obs_behaviors:
            idx = beh['obs_index']
            if idx >= len(self.obs):
                continue

            if beh['behavior'] == 'waypoint':
                self._update_waypoint_obs(idx, beh)
            elif beh['behavior'] == 'chase':
                self._update_chase_obs(idx, beh)

    def _update_waypoint_obs(self, idx, beh):
        ox, oy = float(self.obs[idx, 0]), float(self.obs[idx, 1])
        wps = beh['waypoints']
        wp_idx = beh.get('_wp_idx', 0)
        threshold = 0.2

        tx, ty = wps[wp_idx]
        dist = math.hypot(tx - ox, ty - oy)
        if dist < threshold and wp_idx < len(wps) - 1:
            wp_idx += 1
            beh['_wp_idx'] = wp_idx
            tx, ty = wps[wp_idx]
            dist = math.hypot(tx - ox, ty - oy)

        v_max = beh['v_max']
        if dist < threshold and wp_idx == len(wps) - 1:
            vx, vy = 0.0, 0.0
        elif dist < 1e-6:
            vx, vy = 0.0, 0.0
        else:
            vx = v_max * (tx - ox) / dist
            vy = v_max * (ty - oy) / dist

        self.obs[idx, 3] = vx
        self.obs[idx, 4] = vy

    def _update_chase_obs(self, idx, beh):
        ox, oy = float(self.obs[idx, 0]), float(self.obs[idx, 1])
        rx, ry = float(self.robot.X[0, 0]), float(self.robot.X[1, 0])
        dx, dy = rx - ox, ry - oy
        dist = math.hypot(dx, dy)
        v_max = beh['v_max']
        if dist < 1e-6:
            vx, vy = 0.0, 0.0
        else:
            vx = v_max * dx / dist
            vy = v_max * dy / dist
        self.obs[idx, 3] = vx
        self.obs[idx, 4] = vy

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


def build_from_scenario(scenario_name):
    """シナリオconfigからシミュレーション用パラメータを構築"""
    sc = load_scenario(scenario_name)
    dt = 0.05

    robot_cfg = sc['robot']
    robot_spec = {
        'model': 'DoubleIntegrator2D',
        'v_max': robot_cfg['v_max'],
        'a_max': robot_cfg['a_max'],
        'radius': robot_cfg['radius'],
        'sensing_range': robot_cfg['sensing_range'],
        'backup_cbf': {'T_horizon': sc['cbf']['T_horizon']},
    }

    start = robot_cfg['start']
    goal = robot_cfg['goal']
    waypoints = np.array([[start[0], start[1], 0.0], [goal[0], goal[1], 0.0]])

    env_cfg = sc['env']
    env_width = env_cfg['x_max'] - env_cfg['x_min']
    env_height = env_cfg['y_max'] - env_cfg['y_min']

    seed = sc.get('seed', 42)
    random.seed(seed)

    obs_data = []
    obs_meta = []
    scenario_obs_behaviors = []  # waypoint/chase の追加情報
    obs_index = 0

    for obs_cfg in sc.get('obstacles', []):
        x, y = obs_cfg['position']
        r = obs_cfg['radius']
        v_max = obs_cfg['v_max']
        behavior = obs_cfg['behavior']
        y_min = env_cfg['y_min']
        y_max = env_cfg['y_max']

        if behavior == 'random_walk':
            theta = random.uniform(-math.pi, math.pi)
            vx = v_max * math.cos(theta)
            vy = v_max * math.sin(theta)
            obs_data.append([x, y, r, vx, vy, y_min, y_max, 1])
            obs_meta.append({'mode': 1, 'v_max': v_max, 'theta': theta})
        elif behavior == 'waypoint':
            # 初速度: 最初のWP→次のWP方向
            wps = obs_cfg['waypoints']
            if len(wps) >= 2:
                dx, dy = wps[1][0] - wps[0][0], wps[1][1] - wps[0][1]
                d = math.hypot(dx, dy)
                vx = v_max * dx / d if d > 1e-6 else 0.0
                vy = v_max * dy / d if d > 1e-6 else 0.0
            else:
                vx, vy = 0.0, 0.0
            obs_data.append([x, y, r, vx, vy, y_min, y_max, 1])
            obs_meta.append({'mode': 0, 'v_max': v_max, 'theta': math.atan2(vy, vx)})
            scenario_obs_behaviors.append({
                'obs_index': obs_index, 'behavior': 'waypoint',
                'waypoints': wps, 'v_max': v_max, '_wp_idx': 0,
            })
        elif behavior == 'chase':
            obs_data.append([x, y, r, 0.0, 0.0, y_min, y_max, 1])
            obs_meta.append({'mode': 0, 'v_max': v_max, 'theta': 0.0})
            scenario_obs_behaviors.append({
                'obs_index': obs_index, 'behavior': 'chase', 'v_max': v_max,
            })
        else:  # static
            obs_data.append([x, y, r, 0.0, 0.0, y_min, y_max, 0])
            obs_meta.append({'mode': 0, 'v_max': 0.0, 'theta': 0.0})
        obs_index += 1

    # 壁の円近似 → 静的障害物として追加
    for wall in sc.get('walls', []):
        y_min = env_cfg['y_min']
        y_max = env_cfg['y_max']
        for cx, cy, cr in wall.get('circles', []):
            obs_data.append([cx, cy, cr, 0.0, 0.0, y_min, y_max, 0])
            obs_meta.append({'mode': 0, 'v_max': 0.0, 'theta': 0.0})

    known_obs = np.array(obs_data, dtype=float) if obs_data else np.empty((0, 8))

    return {
        'dt': dt,
        'robot_spec': robot_spec,
        'waypoints': waypoints,
        'known_obs': known_obs,
        'obs_meta': obs_meta,
        'scenario_obs_behaviors': scenario_obs_behaviors,
        'env_width': env_width,
        'env_height': env_height,
        'seed': seed,
    }


def run_simulation(args):
    params = build_from_scenario(args.scenario)
    dt = params['dt']
    robot_spec = params['robot_spec']
    waypoints = params['waypoints']
    known_obs = params['known_obs']
    obs_meta = params['obs_meta']
    x_init = waypoints[0]

    plot_handler = plotting.Plotting(
        width=params['env_width'], height=params['env_height'], known_obs=known_obs)
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
        rand_seed=params['seed'],
        scenario_obs_behaviors=params['scenario_obs_behaviors'],
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
              'total_steps': len(controller.data_log),
              'scenario': args.scenario}
    with open(os.path.join(args.output, 'result.json'), 'w') as f:
        json.dump(result, f, indent=2)

    print(f'Scenario: {args.scenario}')
    print(f'Outcome: {outcome}')
    print(f'Duration: {duration:.2f}s ({len(controller.data_log)} steps)')
    print(f'CSV: {csv_path}')

    plt.close('all')


def main():
    parser = argparse.ArgumentParser(description='Run numerical simulation with data collection')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--scenario', '-s', default='multi_random',
                        help=f'Scenario name (available: {list_scenarios()})')
    parser.add_argument('--tf', type=float, default=300.0, help='Simulation time limit (s)')
    args = parser.parse_args()
    run_simulation(args)


if __name__ == '__main__':
    main()
