#!/usr/bin/env python3
"""Batch experiment runner: sequential ros2 launch execution for mode comparison."""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime


def run_single(scenario, mode, experiment_id, timeout, gui, experiments_dir):
    """単一モードのシミュレーション実行"""
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

    # result.json 読み取り
    if mode == 'unicycle-tb3':
        bag_subdir = 'gazebo_unicycle_tb3'
    elif mode == 'unicycle':
        bag_subdir = 'gazebo_unicycle'
    else:
        bag_subdir = 'gazebo_di'

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
    """plot_experiment.py を実行"""
    exp_path = os.path.join(experiments_dir, bag_subdir, experiment_id)
    if not os.path.isdir(exp_path):
        print(f'  Skip analysis: {exp_path} not found')
        return
    cmd = ['python3', 'src/occlusion_sim/analysis/plot_experiment.py', exp_path]
    if robot_radius:
        cmd += ['--robot-radius', str(robot_radius)]
    print(f'  Analysis: {" ".join(cmd)}')
    subprocess.run(cmd)


def main():
    parser = argparse.ArgumentParser(description='Batch experiment runner')
    parser.add_argument('--scenario', default='corner_popout')
    parser.add_argument('--experiment-id', default=None)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--modes', default='di,unicycle')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--experiments-dir', default='/root/Gazebo_ws/experiments')
    args = parser.parse_args()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    experiment_id = args.experiment_id or f'batch_{timestamp}'
    modes = [m.strip() for m in args.modes.split(',')]
    results = {}

    # 順次実行
    for mode in modes:
        bag_subdir, result = run_single(
            args.scenario, mode, experiment_id, args.timeout, args.gui, args.experiments_dir)
        results[mode] = {'bag_subdir': bag_subdir, 'result': result}
        print('Waiting 5s for cleanup...')
        time.sleep(5)

    # 解析
    print(f'\n{"="*60}')
    print('  Running analysis...')
    print(f'{"="*60}')
    for mode, info in results.items():
        radius = 0.105 if mode == 'unicycle-tb3' else None
        run_analysis(args.experiments_dir, info['bag_subdir'], experiment_id, radius)

    # サマリー
    print(f'\n{"="*60}')
    print(f'  Summary: {args.scenario} / {experiment_id}')
    print(f'{"="*60}')
    print(f'  {"Mode":<16} {"Outcome":<16} {"Duration (s)":>12}')
    print(f'  {"-"*44}')
    for mode in modes:
        r = results[mode]['result']
        if r:
            print(f'  {mode:<16} {r["outcome"]:<16} {r["duration"]:>12.2f}')
        else:
            print(f'  {mode:<16} {"N/A":<16} {"N/A":>12}')
    print()


if __name__ == '__main__':
    main()
