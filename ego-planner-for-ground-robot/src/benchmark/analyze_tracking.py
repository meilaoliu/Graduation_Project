#!/usr/bin/env python3
"""
Analyze MPC tracking performance from cmd_profile.csv.

Computes lateral (cross-track) error, heading error, segmented stats,
and generates publication-quality figures.

Usage:
  python3 analyze_tracking.py                          # default csv
  python3 analyze_tracking.py /path/to/cmd_profile.csv
"""
import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.family'] = 'DejaVu Sans'
rcParams['mathtext.fontset'] = 'dejavusans'
rcParams['axes.unicode_minus'] = False
rcParams['font.size'] = 11


def parse_args():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser()
    p.add_argument('csv_path', nargs='?',
                   default=os.path.join(script_dir, 'cmd_profile.csv'))
    p.add_argument('--curv-thresh', type=float, default=0.3,
                   help='Curvature threshold to distinguish straight/curve')
    p.add_argument('--speed-thresh', type=float, default=0.3,
                   help='Min speed to include in stats (skip start/stop transients)')
    p.add_argument('--output-dir', type=str, default=None)
    return p.parse_args()


def lateral_error_direct(ref_x, ref_y, odom_x, odom_y, ref_vx, ref_vy, speed):
    """Signed lateral error using reference tangent direction."""
    lat_err = np.full_like(ref_x, np.nan)
    valid = speed > 0.05
    dx = odom_x - ref_x
    dy = odom_y - ref_y
    tx = ref_vx / np.maximum(speed, 1e-6)
    ty = ref_vy / np.maximum(speed, 1e-6)
    lat_err[valid] = (-ty[valid] * dx[valid] + tx[valid] * dy[valid])
    return lat_err


def heading_error(ref_yaw, odom_yaw, speed, speed_thresh=0.15):
    """Heading error in degrees, masked at low speed."""
    err = np.full_like(ref_yaw, np.nan)
    valid = speed > speed_thresh
    diff = odom_yaw[valid] - ref_yaw[valid]
    err[valid] = np.degrees(np.arctan2(np.sin(diff), np.cos(diff)))
    return err


def print_stats_table(name, lat, hdg):
    valid_lat = lat[~np.isnan(lat)]
    valid_hdg = hdg[~np.isnan(hdg)]
    if len(valid_lat) == 0:
        print(f"  {name}: no valid samples")
        return
    print(f"  {name}:")
    print(f"    Lateral  - mean: {np.mean(np.abs(valid_lat))*100:.2f} cm, "
          f"max: {np.max(np.abs(valid_lat))*100:.2f} cm")
    if len(valid_hdg) > 0:
        print(f"    Heading  - mean: {np.mean(np.abs(valid_hdg)):.2f} deg, "
              f"max: {np.max(np.abs(valid_hdg)):.2f} deg")


def main():
    args = parse_args()
    csv_path = args.csv_path
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        sys.exit(1)

    out_dir = args.output_dir or os.path.dirname(os.path.abspath(csv_path))
    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    t = data['time']
    n = len(t)

    has_ref_pos = 'ref_x' in data.dtype.names
    if has_ref_pos:
        ref_x = data['ref_x']
        ref_y = data['ref_y']
        ref_yaw = data['ref_yaw']
    else:
        print("WARNING: CSV has no ref_x/ref_y, using velocity integration (less accurate)")
        vx, vy = data['ref_vx'], data['ref_vy']
        ref_x = np.zeros_like(t)
        ref_y = np.zeros_like(t)
        ref_x[0] = data['odom_x'][0]
        ref_y[0] = data['odom_y'][0]
        for i in range(1, n):
            dt = t[i] - t[i-1]
            ref_x[i] = ref_x[i-1] + vx[i-1] * dt
            ref_y[i] = ref_y[i-1] + vy[i-1] * dt
        spd = data['ref_speed']
        ref_yaw = np.where(spd > 0.05, np.arctan2(vy, vx), data['odom_yaw'])

    speed = data['ref_speed']

    lat_err = lateral_error_direct(ref_x, ref_y, data['odom_x'], data['odom_y'],
                                   data['ref_vx'], data['ref_vy'], speed)
    hdg_err = heading_error(ref_yaw, data['odom_yaw'], speed, args.speed_thresh)

    # omega_ref from differential flatness: omega = curvature * speed
    omega_ref = data['ref_curvature'] * speed

    moving = speed > args.speed_thresh
    abs_curv = np.abs(data['ref_curvature'])
    straight = moving & (abs_curv < args.curv_thresh)
    curve = moving & (abs_curv >= args.curv_thresh)

    print("=" * 55)
    print("  MPC Tracking Performance Statistics")
    print("=" * 55)
    print(f"  Total samples: {n}, duration: {t[-1]-t[0]:.2f}s")
    print(f"  Reference source: {'direct (ref_x/ref_y)' if has_ref_pos else 'velocity integration'}")
    print_stats_table("Straight", lat_err[straight], hdg_err[straight])
    print_stats_table("Curve", lat_err[curve], hdg_err[curve])
    print_stats_table("Overall (moving)", lat_err[moving], hdg_err[moving])
    print("=" * 55)

    # ======== Figure 1: XY trajectory comparison ========
    fig1, ax1 = plt.subplots(figsize=(7, 7))
    ax1.plot(ref_x, ref_y, 'b-', linewidth=1.5, label='Reference trajectory')
    ax1.plot(data['odom_x'], data['odom_y'], 'r--', linewidth=1.2,
             label='Actual trajectory', alpha=0.8)
    ax1.set_xlabel('$x$ (m)')
    ax1.set_ylabel('$y$ (m)')
    ax1.set_aspect('equal')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Trajectory Tracking (XY plane)')
    fig1.tight_layout()
    fig1.savefig(os.path.join(out_dir, 'tracking_xy.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: tracking_xy.png")

    # ======== Figure 2: velocity & angular velocity commands ========
    fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)

    # Linear velocity: use |cmd_v| vs ref_speed for fair comparison
    ax2a.plot(t, np.abs(data['cmd_v']), 'b-', linewidth=0.8, label='$|v_{cmd}|$')
    ax2a.plot(t, speed, 'r--', linewidth=0.8, alpha=0.7, label='$v_{ref}$')
    ax2a.set_ylabel('$v$ (m/s)')
    ax2a.legend(loc='upper right')
    ax2a.grid(True, alpha=0.3)

    # Angular velocity: cmd_w vs omega_ref
    ax2b.plot(t, data['cmd_w'], 'b-', linewidth=0.8, label='$\\omega_{cmd}$')
    omega_ref_plot = np.where(speed > 0.1, omega_ref, np.nan)
    ax2b.plot(t, omega_ref_plot, 'r--', linewidth=0.8, alpha=0.7, label='$\\omega_{ref}$')
    ax2b.set_ylabel('$\\omega$ (rad/s)')
    ax2b.set_xlabel('$t$ (s)')
    ax2b.legend(loc='upper right')
    ax2b.grid(True, alpha=0.3)

    fig2.tight_layout()
    fig2.savefig(os.path.join(out_dir, 'tracking_cmd.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: tracking_cmd.png")

    # ======== Figure 3: lateral error and heading error ========
    fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)

    lat_cm = lat_err * 100
    ax3a.plot(t, lat_cm, 'b-', linewidth=0.8)
    ax3a.set_ylabel('Lateral error (cm)')
    ax3a.grid(True, alpha=0.3)
    valid_lat = lat_cm[~np.isnan(lat_cm)]
    if len(valid_lat) > 0:
        bound = max(10, np.percentile(np.abs(valid_lat), 99) * 1.3)
        ax3a.set_ylim(-bound, bound)

    # Heading error: already NaN-masked at low speed, no more spikes
    ax3b.plot(t, hdg_err, 'r-', linewidth=0.8)
    ax3b.set_ylabel('Heading error (deg)')
    ax3b.set_xlabel('$t$ (s)')
    ax3b.grid(True, alpha=0.3)
    valid_hdg = hdg_err[~np.isnan(hdg_err)]
    if len(valid_hdg) > 0:
        bound_h = max(5, np.percentile(np.abs(valid_hdg), 99) * 1.3)
        ax3b.set_ylim(-bound_h, bound_h)

    fig3.tight_layout()
    fig3.savefig(os.path.join(out_dir, 'tracking_error.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: tracking_error.png")

    plt.close('all')
    print("\nDone.")


if __name__ == '__main__':
    main()
