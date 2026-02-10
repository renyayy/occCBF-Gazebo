#!/usr/bin/env python3
"""Compare two experiment datasets (rosbag or CSV) with overlay plots."""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from plot_experiment import (
    load_experiment_data, detect_outcome,
    STAMP, H_MIN, MIN_DIST, N_CONSTRAINTS, QP_TIME_MS,
    INTERVENTION, U_X, U_Y, U_REF_X, U_REF_Y,
    ROBOT_X, ROBOT_Y, ROBOT_VX, ROBOT_VY, STATUS_OK,
    COLLISION_DIST, GOAL_THRESHOLD,
)

COLORS = ['tab:blue', 'tab:orange']


def compute_stats(t, data):
    h = data[:, H_MIN]
    dist = data[:, MIN_DIST]
    finite_dist = dist[np.isfinite(dist)]
    qp_time = data[:, QP_TIME_MS]
    interventions = data[:, INTERVENTION]
    u_err = np.hypot(data[:, U_X] - data[:, U_REF_X], data[:, U_Y] - data[:, U_REF_Y])
    return {
        'duration': t[-1] - t[0],
        'samples': len(data),
        'h_min': float(np.min(h)),
        'min_dist': float(np.min(finite_dist)) if len(finite_dist) > 0 else float('inf'),
        'qp_mean': float(np.mean(qp_time)),
        'qp_max': float(np.max(qp_time)),
        'tracking_err_mean': float(np.mean(u_err)),
        'tracking_err_max': float(np.max(u_err)),
        'backup_rate': float(np.mean(interventions > 0.5) * 100),
        'violations': int(np.sum(h < 0)),
    }


def plot_h_comparison(datasets, output_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    for (t, data, label, outcome), color in zip(datasets, COLORS):
        ax.plot(t, data[:, H_MIN], color=color, linewidth=0.8, label=f'{label} ({outcome})')
    ax.axhline(0, color='r', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('h_min(x)')
    ax.set_title('CBF Safety Function h(x) — Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'h_comparison.png'), dpi=150)
    plt.close(fig)


def plot_distance_comparison(datasets, output_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    for (t, data, label, outcome), color in zip(datasets, COLORS):
        d = data[:, MIN_DIST]
        mask = np.isfinite(d)
        ax.plot(t[mask], d[mask], color=color, linewidth=0.8, label=f'{label} ({outcome})')
    ax.axhline(COLLISION_DIST, color='r', linestyle='--', linewidth=1, alpha=0.5,
               label=f'collision ({COLLISION_DIST:.2f}m)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Min Distance [m]')
    ax.set_title('Minimum Distance to Obstacles — Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'distance_comparison.png'), dpi=150)
    plt.close(fig)


def plot_tracking_error_comparison(datasets, output_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    for (t, data, label, outcome), color in zip(datasets, COLORS):
        u_err = np.hypot(data[:, U_X] - data[:, U_REF_X], data[:, U_Y] - data[:, U_REF_Y])
        ax.plot(t, u_err, color=color, linewidth=0.8, alpha=0.7, label=f'{label} ({outcome})')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('||u - u_ref||')
    ax.set_title('Control Tracking Error — Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'tracking_error_comparison.png'), dpi=150)
    plt.close(fig)


def plot_trajectory_comparison(datasets, output_dir):
    fig, ax = plt.subplots(figsize=(10, 8))
    for (t, data, label, outcome), color in zip(datasets, COLORS):
        ax.plot(data[:, ROBOT_X], data[:, ROBOT_Y], color=color, linewidth=1, label=f'{label} ({outcome})')
        ax.plot(data[0, ROBOT_X], data[0, ROBOT_Y], 'o', color=color, markersize=8)
        ax.plot(data[-1, ROBOT_X], data[-1, ROBOT_Y], '*', color=color, markersize=10)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Robot Trajectory — Comparison')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'trajectory_comparison.png'), dpi=150)
    plt.close(fig)


def print_comparison(labels, stats_list, outcomes):
    col_w = max(len(l) for l in labels) + 2
    header = f'{"":20s}' + ''.join(f'{l:>{col_w}s}' for l in labels)
    print(f'\n=== Comparison ===')
    print(header)
    print('-' * len(header))

    rows = [
        ('Outcome', [o for o in outcomes]),
        ('Duration (s)', [f'{s["duration"]:.2f}' for s in stats_list]),
        ('h_min (global)', [f'{s["h_min"]:.6f}' for s in stats_list]),
        ('Min distance (m)', [f'{s["min_dist"]:.4f}' for s in stats_list]),
        ('QP time mean (ms)', [f'{s["qp_mean"]:.2f}' for s in stats_list]),
        ('QP time max (ms)', [f'{s["qp_max"]:.2f}' for s in stats_list]),
        ('Tracking error', [f'{s["tracking_err_mean"]:.4f}' for s in stats_list]),
        ('Backup QP rate', [f'{s["backup_rate"]:.1f}%' for s in stats_list]),
        ('Violations', [f'{s["violations"]}' for s in stats_list]),
    ]
    for name, vals in rows:
        print(f'{name:20s}' + ''.join(f'{v:>{col_w}s}' for v in vals))


def main():
    parser = argparse.ArgumentParser(description='Compare two experiment datasets')
    parser.add_argument('paths', nargs=2, help='Paths to rosbag dirs or CSV files')
    parser.add_argument('--labels', nargs=2, default=['Experiment A', 'Experiment B'])
    parser.add_argument('--output', '-o', default='comparison_output', help='Output directory')
    parser.add_argument('--goal', nargs=2, type=float, default=[20.0, 7.5],
                        metavar=('X', 'Y'))
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    goal = np.array(args.goal)

    datasets = []
    stats_list = []
    outcomes = []
    for path, label in zip(args.paths, args.labels):
        data = load_experiment_data(path)
        if data is None:
            print(f'Failed to load: {path}')
            return
        t = data[:, STAMP]
        t = t - t[0]
        outcome, _ = detect_outcome(data, goal)
        datasets.append((t, data, label, outcome))
        stats_list.append(compute_stats(t, data))
        outcomes.append(outcome)

    print_comparison(args.labels, stats_list, outcomes)

    plot_h_comparison(datasets, args.output)
    plot_distance_comparison(datasets, args.output)
    plot_tracking_error_comparison(datasets, args.output)
    plot_trajectory_comparison(datasets, args.output)

    print(f'\nPlots saved to: {args.output}/')


if __name__ == '__main__':
    main()
