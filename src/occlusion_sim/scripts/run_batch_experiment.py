#!/usr/bin/env python3
"""Batch experiment runner: sequential ros2 launch execution for mode comparison."""
import argparse
import csv
import json
import os
import subprocess
import sys
import time
from datetime import datetime

import numpy as np


def _bag_subdir(mode):
    if mode == 'unicycle-tb3':
        return 'gazebo_unicycle_tb3'
    elif mode == 'unicycle':
        return 'gazebo_unicycle'
    return 'gazebo_di'


def run_single(scenario, mode, experiment_id, timeout, gui, experiments_dir):
    cmd = [
        'ros2', 'launch', 'occlusion_sim', 'gazebo_sim.launch.py',
        f'scenario:={scenario}', f'mode:={mode}',
        f'experiment_id:={experiment_id}',
        f'auto_shutdown:=true', f'sim_timeout:={timeout}',
        f'gui:={"true" if gui else "false"}',
        f'bag_output_dir:={experiments_dir}',
    ]
    print(f'\n{"="*60}')
    print(f'  Mode: {mode} | Scenario: {scenario} | ID: {experiment_id}')
    print(f'  Command: {" ".join(cmd)}')
    print(f'{"="*60}\n')

    proc = subprocess.run(cmd)
    print(f'\nProcess exited with code {proc.returncode}')

    bag_subdir = _bag_subdir(mode)
    result_path = os.path.join(experiments_dir, bag_subdir, experiment_id, 'result.json')
    result = None
    if os.path.exists(result_path):
        with open(result_path) as f:
            result = json.load(f)
        print(f'  Result: {result["outcome"]} ({result["duration"]:.2f}s)')
    else:
        print(f'  Result file not found: {result_path}')

    return bag_subdir, result


def run_analysis(experiments_dir, bag_subdir, experiment_id, robot_radius=None):
    exp_path = os.path.join(experiments_dir, bag_subdir, experiment_id)
    if not os.path.isdir(exp_path):
        print(f'  Skip analysis: {exp_path} not found')
        return
    cmd = ['python3', 'src/occlusion_sim/analysis/plot_experiment.py', exp_path]
    if robot_radius:
        cmd += ['--robot-radius', str(robot_radius)]
    print(f'  Analysis: {" ".join(cmd)}')
    subprocess.run(cmd)


def print_summary(modes, all_results, scenario, experiment_id, runs, experiments_dir):
    print(f'\n{"="*60}')
    print(f'  Summary: {scenario} / {experiment_id} (runs={runs})')
    print(f'{"="*60}')
    print(f'  {"Mode":<16} {"Run":<6} {"Outcome":<16} {"Duration (s)":>12}')
    print(f'  {"-"*52}')

    csv_rows = []
    for mode, run_idx, result in all_results:
        run_label = str(run_idx) if runs > 1 else '-'
        if result:
            print(f'  {mode:<16} {run_label:<6} {result["outcome"]:<16} {result["duration"]:>12.2f}')
            csv_rows.append({'mode': mode, 'run': run_idx, 'outcome': result['outcome'],
                             'duration': result['duration']})
        else:
            print(f'  {mode:<16} {run_label:<6} {"N/A":<16} {"N/A":>12}')
            csv_rows.append({'mode': mode, 'run': run_idx, 'outcome': 'N/A', 'duration': ''})

    if runs > 1:
        print(f'\n  {"="*52}')
        print(f'  Aggregate Statistics')
        print(f'  {"-"*52}')
        print(f'  {"Mode":<16} {"Success%":>8} {"Collision%":>10} {"Timeout%":>9} {"Dur (mean±std)":>16}')
        print(f'  {"-"*52}')
        for mode in modes:
            mode_results = [r for m, _, r in all_results if m == mode and r]
            n = len(mode_results)
            if n == 0:
                print(f'  {mode:<16} {"N/A":>8} {"N/A":>10} {"N/A":>9} {"N/A":>16}')
                continue
            outcomes = [r['outcome'] for r in mode_results]
            durations = [r['duration'] for r in mode_results]
            n_success = outcomes.count('goal_reached')
            n_collision = outcomes.count('collision')
            n_timeout = outcomes.count('timeout')
            print(f'  {mode:<16} {n_success/n*100:>7.1f}% {n_collision/n*100:>9.1f}% '
                  f'{n_timeout/n*100:>8.1f}% {np.mean(durations):>7.2f}±{np.std(durations):.2f}')

    # CSV出力
    csv_path = os.path.join(experiments_dir, f'{experiment_id}_summary.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['mode', 'run', 'outcome', 'duration'])
        writer.writeheader()
        writer.writerows(csv_rows)
    print(f'\n  CSV saved: {csv_path}')
    print()


def main():
    parser = argparse.ArgumentParser(description='Batch experiment runner')
    parser.add_argument('--scenario', default='corner_popout')
    parser.add_argument('--experiment-id', default=None)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--modes', default='di,unicycle')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--experiments-dir', default='/root/Gazebo_ws/experiments')
    args = parser.parse_args()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    experiment_id = args.experiment_id or f'batch_{timestamp}'
    modes = [m.strip() for m in args.modes.split(',')]
    all_results = []  # [(mode, run_idx, result), ...]

    for mode in modes:
        for run_idx in range(1, args.runs + 1):
            run_id = f'{experiment_id}/run_{run_idx:03d}' if args.runs > 1 else experiment_id
            bag_subdir, result = run_single(
                args.scenario, mode, run_id, args.timeout, args.gui, args.experiments_dir)
            all_results.append((mode, run_idx, result))
            print('Waiting 5s for cleanup...')
            time.sleep(5)

    # 解析（runs=1のみ）
    if args.runs == 1:
        print(f'\n{"="*60}')
        print('  Running analysis...')
        print(f'{"="*60}')
        for mode, _, result in all_results:
            radius = 0.105 if mode == 'unicycle-tb3' else None
            run_analysis(args.experiments_dir, _bag_subdir(mode), experiment_id, radius)

    print_summary(modes, all_results, args.scenario, experiment_id, args.runs, args.experiments_dir)


if __name__ == '__main__':
    main()
