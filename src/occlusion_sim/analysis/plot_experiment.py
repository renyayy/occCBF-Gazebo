#!/usr/bin/env python3
"""Experiment analysis: read rosbag2 and generate evaluation plots.

/cbf_debug_info (Float64MultiArray) field layout:
  0: stamp_sec        タイムスタンプ(秒)
  1: h_min            最小CBF値 h(x)
  2: min_dist         最近傍障害物距離
  3: num_constraints   アクティブ制約数
  4: qp_solve_time_ms  QP計算時間(ms)
  5: intervention      0.0=u_ref, 1.0=backup_qp
  6: u_x              制御出力 ax
  7: u_y              制御出力 ay
  8: u_ref_x          ノミナル制御 ax
  9: u_ref_y          ノミナル制御 ay
  10: robot_x          ロボット位置 x
  11: robot_y          ロボット位置 y
  12: robot_vx         ロボット速度 vx
  13: robot_vy         ロボット速度 vy
  14: status_ok        1.0=optimal
  15: num_visible_obs   可視障害物数
  16: num_total_obs     全障害物数
"""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray

# Field indices
STAMP = 0; H_MIN = 1; MIN_DIST = 2; N_CONSTRAINTS = 3
QP_TIME_MS = 4; INTERVENTION = 5; U_X = 6; U_Y = 7
U_REF_X = 8; U_REF_Y = 9; ROBOT_X = 10; ROBOT_Y = 11
ROBOT_VX = 12; ROBOT_VY = 13; STATUS_OK = 14
N_VISIBLE = 15; N_TOTAL = 16

COLLISION_DIST = 0.25 + 0.3  # robot_radius + obstacle_radius


def read_cbf_debug(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    filter_ = rosbag2_py.StorageFilter(topics=['/cbf_debug_info'])
    reader.set_filter(filter_)

    records = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, Float64MultiArray)
        records.append(np.array(msg.data))

    if not records:
        print('No /cbf_debug_info messages found in bag.')
        return None
    return np.array(records)


def plot_h_trajectory(t, h, output_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, h, 'b-', linewidth=0.8)
    ax.axhline(0, color='r', linestyle='--', linewidth=1, label='h=0 (safety boundary)')
    ax.fill_between(t, h, 0, where=(h < 0), color='red', alpha=0.3, label='violation')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('h_min(x)')
    ax.set_title(f'CBF Safety Function h(x) — min={np.min(h):.4f}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'h_trajectory.png'), dpi=150)
    plt.close(fig)


def plot_min_distance(t, dist, output_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, dist, 'b-', linewidth=0.8)
    ax.axhline(COLLISION_DIST, color='r', linestyle='--', linewidth=1,
               label=f'collision threshold ({COLLISION_DIST:.2f}m)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Min Distance [m]')
    global_min = np.min(dist[np.isfinite(dist)]) if np.any(np.isfinite(dist)) else float('inf')
    ax.set_title(f'Minimum Distance to Obstacles — min={global_min:.4f}m')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'min_distance.png'), dpi=150)
    plt.close(fig)


def plot_tracking_error(t, u_err, interventions, output_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    colors = np.where(interventions > 0.5, 'red', 'blue')
    ax.scatter(t, u_err, c=colors, s=2, alpha=0.6)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('||u - u_ref||')
    ax.set_title('Control Tracking Error (blue=u_ref, red=backup_qp)')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'tracking_error.png'), dpi=150)
    plt.close(fig)


def plot_summary(t, data, output_dir):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # h(x)
    ax = axes[0, 0]
    ax.plot(t, data[:, H_MIN], 'b-', linewidth=0.8)
    ax.axhline(0, color='r', linestyle='--', linewidth=1)
    ax.fill_between(t, data[:, H_MIN], 0, where=(data[:, H_MIN] < 0), color='red', alpha=0.3)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('h_min(x)')
    ax.set_title('CBF Safety Function')
    ax.grid(True, alpha=0.3)

    # min distance
    ax = axes[0, 1]
    dist = data[:, MIN_DIST]
    finite_mask = np.isfinite(dist)
    ax.plot(t[finite_mask], dist[finite_mask], 'b-', linewidth=0.8)
    ax.axhline(COLLISION_DIST, color='r', linestyle='--', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Min Distance [m]')
    ax.set_title('Minimum Distance')
    ax.grid(True, alpha=0.3)

    # tracking error
    ax = axes[1, 0]
    u_err = np.hypot(data[:, U_X] - data[:, U_REF_X], data[:, U_Y] - data[:, U_REF_Y])
    ax.plot(t, u_err, 'b-', linewidth=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('||u - u_ref||')
    ax.set_title('Control Tracking Error')
    ax.grid(True, alpha=0.3)

    # trajectory
    ax = axes[1, 1]
    ax.plot(data[:, ROBOT_X], data[:, ROBOT_Y], 'b-', linewidth=1, label='trajectory')
    ax.plot(data[0, ROBOT_X], data[0, ROBOT_Y], 'go', markersize=8, label='start')
    ax.plot(data[-1, ROBOT_X], data[-1, ROBOT_Y], 'r*', markersize=10, label='end')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Robot Trajectory')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle('Experiment Summary', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'summary.png'), dpi=150)
    plt.close(fig)


def print_stats(t, data):
    h = data[:, H_MIN]
    dist = data[:, MIN_DIST]
    qp_time = data[:, QP_TIME_MS]
    interventions = data[:, INTERVENTION]
    u_err = np.hypot(data[:, U_X] - data[:, U_REF_X], data[:, U_Y] - data[:, U_REF_Y])

    finite_dist = dist[np.isfinite(dist)]
    duration = t[-1] - t[0]
    violations = np.sum(h < 0)
    intervention_rate = np.mean(interventions > 0.5) * 100

    print(f'=== Experiment Statistics ===')
    print(f'Duration:           {duration:.2f} s  ({len(data)} samples)')
    print(f'h_min global min:   {np.min(h):.6f}  (at t={t[np.argmin(h)]:.2f}s)')
    print(f'Safety violations:  {violations} / {len(h)}  ({violations/len(h)*100:.1f}%)')
    print(f'Min distance:       {np.min(finite_dist):.4f} m' if len(finite_dist) > 0 else 'Min distance: N/A')
    print(f'QP solve time:      mean={np.mean(qp_time):.2f} ms, max={np.max(qp_time):.2f} ms')
    print(f'Tracking error:     mean={np.mean(u_err):.4f}, max={np.max(u_err):.4f}')
    print(f'Backup QP rate:     {intervention_rate:.1f}%')


def main():
    parser = argparse.ArgumentParser(description='Analyze experiment rosbag')
    parser.add_argument('bag_path', help='Path to rosbag2 directory')
    parser.add_argument('--output', '-o', default=None, help='Output directory for plots')
    args = parser.parse_args()

    output_dir = args.output or args.bag_path
    os.makedirs(output_dir, exist_ok=True)

    data = read_cbf_debug(args.bag_path)
    if data is None:
        return

    t = data[:, STAMP]
    t = t - t[0]  # 0起点に正規化

    print_stats(t, data)
    plot_h_trajectory(t, data[:, H_MIN], output_dir)
    plot_min_distance(t, data[:, MIN_DIST], output_dir)

    u_err = np.hypot(data[:, U_X] - data[:, U_REF_X], data[:, U_Y] - data[:, U_REF_Y])
    plot_tracking_error(t, u_err, data[:, INTERVENTION], output_dir)
    plot_summary(t, data, output_dir)

    print(f'\nPlots saved to: {output_dir}/')


if __name__ == '__main__':
    main()
