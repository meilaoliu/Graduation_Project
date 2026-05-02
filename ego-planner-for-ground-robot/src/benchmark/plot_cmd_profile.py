#!/usr/bin/env python3
"""
Plot velocity and curvature profiles from traj_server control-loop log.
Data source: src/log/cmd_profile.csv (SUPER-style, naturally continuous)

Usage:
  python3 plot_cmd_profile.py                   # default: same directory
  python3 plot_cmd_profile.py /path/to/data.csv # custom file
"""
import os
import argparse
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.family'] = 'DejaVu Sans'
rcParams['mathtext.fontset'] = 'dejavusans'
rcParams['axes.unicode_minus'] = False
rcParams['font.size'] = 12


def parse_args(script_dir):
    parser = argparse.ArgumentParser(description='Plot velocity and curvature profiles from cmd_profile.csv')
    parser.add_argument('csv_path', nargs='?', default=os.path.join(script_dir, 'cmd_profile.csv'),
                        help='Input CSV path, default: src/log/cmd_profile.csv')
    parser.add_argument('--v-max', type=float, default=1.5, help='Velocity limit shown in plot')
    parser.add_argument('--k-max', type=float, default=1.5, help='Curvature limit shown in plot')
    parser.add_argument('--curv-min-speed', type=float, default=0.25,
                        help='Treat curvature as undefined when |v| is below this threshold')
    parser.add_argument('--curv-gap-mode', type=str, default='mask', choices=['mask', 'interp', 'zero'],
                        help='How to display low-speed curvature gaps: mask, interp, or zero')
    parser.add_argument('--interp-max-gap', type=float, default=0.5,
                        help='Maximum low-speed gap duration (s) allowed for display interpolation')
    parser.add_argument('--output', type=str, default=os.path.join(script_dir, 'cmd_profile.png'), help='Output figure path')
    return parser.parse_args()


def fill_curvature_gaps_for_display(time_axis, curvature, valid_mask, mode, max_gap_sec):
    curv_plot = curvature.copy()

    if mode == 'zero':
        curv_plot[~valid_mask] = 0.0
        return curv_plot

    curv_plot[~valid_mask] = np.nan
    if mode != 'interp':
        return curv_plot

    valid_indices = np.flatnonzero(valid_mask)
    if valid_indices.size < 2:
        return curv_plot

    for left_idx, right_idx in zip(valid_indices[:-1], valid_indices[1:]):
        if right_idx <= left_idx + 1:
            continue

        gap_slice = slice(left_idx + 1, right_idx)
        if np.any(valid_mask[gap_slice]):
            continue

        gap_duration = time_axis[right_idx] - time_axis[left_idx]
        if gap_duration > max_gap_sec:
            continue

        curv_plot[gap_slice] = np.interp(
            time_axis[gap_slice],
            [time_axis[left_idx], time_axis[right_idx]],
            [curvature[left_idx], curvature[right_idx]],
        )

    return curv_plot


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    args = parse_args(script_dir)
    csv_path = args.csv_path
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        sys.exit(1)

    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    t = data['time']
    ref_speed = data['ref_speed']
    ref_curv = data['ref_curvature']
    v_max = args.v_max
    k_max = args.k_max
    curv_min_speed = args.curv_min_speed
    curv_gap_mode = args.curv_gap_mode

    # Curvature is numerically ill-conditioned and physically meaningless near zero speed.
    # Statistics always ignore low-speed samples; display can optionally interpolate short gaps.
    valid = ref_speed > curv_min_speed
    curv_plot = fill_curvature_gaps_for_display(t, ref_curv, valid, curv_gap_mode, args.interp_max_gap)

    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)

    # --- Velocity ---
    ax1 = axes[0]
    ax1.plot(t, ref_speed, 'b-', linewidth=1.0, label='$v(t)$')
    ax1.axhline(y=v_max, color='r', linestyle='--', linewidth=1.0,
                label=f'$v_{{max}}={v_max}$ m/s')
    ax1.text(-0.01, 1.02, '$v$ (m/s)', transform=ax1.transAxes,
             ha='right', va='bottom', fontsize=12)
    ax1.set_ylabel('')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.set_ylim(bottom=0, top=max(v_max, np.nanmax(ref_speed)) * 1.15)
    ax1.grid(True, alpha=0.3)

    # --- Curvature ---
    ax2 = axes[1]
    ax2.plot(t, curv_plot, 'b-', linewidth=1.0, label='$\\kappa(t)$')
    ax2.axhline(y=k_max, color='r', linestyle='--', linewidth=1.0,
                label=f'$\\kappa_{{max}}={k_max}$ m$^{{-1}}$')
    ax2.axhline(y=-k_max, color='r', linestyle='--', linewidth=1.0)
    ax2.text(-0.01, 1.02, '$\\kappa$ (m$^{-1}$)', transform=ax2.transAxes,
             ha='right', va='bottom', fontsize=12)
    ax2.set_ylabel('')
    ax2.set_xlabel('$t$ (s)', fontsize=12)
    ax2.legend(loc='upper right', fontsize=10)
    ax2.set_ylim(-k_max * 1.8, k_max * 1.8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = args.output
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"Saved figure to {out_path}")

    # Statistics
    duration = t[-1] - t[0]
    print(f"\n===== Profile Statistics =====")
    print(f"Duration: {duration:.2f} s  ({len(t)} samples)")
    print(f"Max ref speed: {np.max(ref_speed):.4f} m/s  (limit: {v_max})")
    print(f"Curvature valid only when speed > {curv_min_speed:.2f} m/s")
    print(f"Curvature display mode: {curv_gap_mode}")
    print(f"Mean ref speed: {np.mean(ref_speed[valid]):.4f} m/s")
    if np.any(valid):
        print(f"Max |ref curvature| (valid segment): {np.max(np.abs(ref_curv[valid])):.4f} m^-1  (limit: {k_max})")
        n_spd_viol = np.sum(ref_speed > v_max + 1e-4)
        n_curv_viol = np.sum(np.abs(ref_curv[valid]) > k_max + 1e-4)
        n_curv_masked = np.sum(~valid)
        print(f"Speed violations: {n_spd_viol} ({100*n_spd_viol/len(t):.1f}%)")
        print(f"Curvature violations: {n_curv_viol} ({100*n_curv_viol/np.sum(valid):.1f}%)")
        print(f"Curvature masked samples: {n_curv_masked} ({100*n_curv_masked/len(t):.1f}%)")


if __name__ == '__main__':
    main()
