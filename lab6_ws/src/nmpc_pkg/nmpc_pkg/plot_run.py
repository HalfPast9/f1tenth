#!/usr/bin/env python3
"""Plot NMPC diagnostic log. Run after a session to see what happened.

Usage: python3 plot_run.py [run_log.json]
"""
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import csv


def load_raceline(path='waypoints.csv'):
    pts = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row['x']), float(row['y'])])
    return np.array(pts)


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else 'run_log.json'
    with open(log_path) as f:
        entries = json.load(f)

    t = np.array([e['t'] for e in entries])
    px = np.array([e['px'] for e in entries])
    py = np.array([e['py'] for e in entries])
    phi = np.array([e['phi'] for e in entries])
    speed_cmd = np.array([e['speed_cmd'] for e in entries])
    steer_cmd = np.array([e['steer_cmd'] for e in entries])
    speed_limit = np.array([e['speed_limit'] for e in entries])
    solve_ms = np.array([e['solve_ms'] for e in entries])
    status = [e['status'] for e in entries]

    infeasible = np.array([s != 'Solve_Succeeded' for s in status])

    try:
        raceline = load_raceline()
    except Exception:
        raceline = None

    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('NMPC Run Diagnostics', fontsize=14, fontweight='bold')

    # 1. Track map: actual path vs raceline + crash point
    ax = axes[0, 0]
    if raceline is not None:
        ax.plot(raceline[:, 0], raceline[:, 1], 'k-', alpha=0.3,
                linewidth=1, label='Raceline')
    ax.plot(px, py, 'b-', linewidth=1.5, label='Actual path')
    ax.plot(px[0], py[0], 'go', markersize=8, label='Start')
    ax.plot(px[-1], py[-1], 'rx', markersize=10, markeredgewidth=3,
            label='End/Crash')
    if np.any(infeasible):
        ax.plot(px[infeasible], py[infeasible], 'r.', markersize=3,
                alpha=0.5, label='Infeasible')
    # Plot last predicted trajectory
    last = entries[-1]
    ax.plot(last['pred_x'], last['pred_y'], 'm--', linewidth=2,
            label='Last predicted traj')
    ax.plot(last['ref_x'][:16], last['ref_y'][:16], 'g+', markersize=6,
            label='Last ref points')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Track Map')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')

    # 2. Zoomed view around crash
    ax = axes[0, 1]
    n = len(px)
    tail = max(0, n - 60)
    if raceline is not None:
        ax.plot(raceline[:, 0], raceline[:, 1], 'k-', alpha=0.3, linewidth=1)
    ax.plot(px[tail:], py[tail:], 'b-o', linewidth=1.5, markersize=2)
    ax.plot(px[-1], py[-1], 'rx', markersize=12, markeredgewidth=3)
    ax.plot(last['pred_x'], last['pred_y'], 'm--o', linewidth=2,
            markersize=4, label='Predicted')
    ax.plot(last['ref_x'][:16], last['ref_y'][:16], 'g+', markersize=10,
            label='Reference')
    margin = 2.0
    ax.set_xlim(px[-1] - margin, px[-1] + margin)
    ax.set_ylim(py[-1] - margin, py[-1] + margin)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Zoomed: Last 2s')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')

    # 3. Speed: commanded vs limit
    ax = axes[1, 0]
    ax.plot(t, speed_cmd, 'b-', label='Speed cmd')
    ax.plot(t, speed_limit, 'r--', label='Speed limit (raceline)')
    if np.any(infeasible):
        for ti in t[infeasible]:
            ax.axvline(x=ti, color='red', alpha=0.1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Speed [m/s]')
    ax.set_title('Speed Command vs Raceline Limit')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Steering
    ax = axes[1, 1]
    ax.plot(t, np.degrees(steer_cmd), 'b-', label='Steering cmd')
    ax.axhline(y=24.0, color='r', linestyle='--', alpha=0.5, label='±24° limit')
    ax.axhline(y=-24.0, color='r', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Steering [deg]')
    ax.set_title('Steering Command')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 5. Solve time
    ax = axes[2, 0]
    colors = ['green' if s == 'Solve_Succeeded' else 'red' for s in status]
    ax.scatter(t, solve_ms, c=colors, s=8, alpha=0.7)
    ax.axhline(y=33, color='orange', linestyle='--', label='33ms budget')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Solve time [ms]')
    ax.set_title('Solver Performance (red = infeasible)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6. Heading + reference heading error
    ax = axes[2, 1]
    ref_heading = []
    for e in entries:
        dx = e['ref_x'][1] - e['ref_x'][0] if len(e['ref_x']) > 1 else 0
        dy = e['ref_y'][1] - e['ref_y'][0] if len(e['ref_y']) > 1 else 0
        ref_heading.append(np.arctan2(dy, dx))
    ref_heading = np.array(ref_heading)
    heading_err = np.arctan2(np.sin(phi - ref_heading),
                             np.cos(phi - ref_heading))
    ax.plot(t, np.degrees(heading_err), 'b-', label='Heading error')
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Heading error [deg]')
    ax.set_title('Heading Error (car vs reference)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = log_path.replace('.json', '.png')
    plt.savefig(out_path, dpi=150)
    print(f'Saved to {out_path}')
    plt.close()


if __name__ == '__main__':
    main()
