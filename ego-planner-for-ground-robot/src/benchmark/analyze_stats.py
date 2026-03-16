#!/usr/bin/env python3
"""
Analyze planning statistics for B-spline or MINCO experiments.

Examples:
  python3 analyze_stats.py
  python3 analyze_stats.py --planner-type minco --format chapter4
  python3 analyze_stats.py --stats-file planning_stats_minco.csv --trial-file trial_results_minco.csv --format chapter4
"""

import argparse
import csv
import os
import sys

import numpy as np


LATEX_NL = r"\\"


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze planning statistics CSV")
    parser.add_argument("--stats-file", type=str, default="planning_stats.csv", help="Planning stats CSV path")
    parser.add_argument("--trial-file", type=str, default="trial_results.csv", help="Trial results CSV path")
    parser.add_argument("--planner-type", type=str, default="", help="Filter rows by planner_type, e.g. bspline or minco")
    parser.add_argument("--curv-cap", type=float, default=10.0, help="Cap curvature outliers above this threshold")
    parser.add_argument("--format", type=str, default="chapter3", choices=["chapter3", "chapter4"], help="Output format")
    return parser.parse_args()


def resolve_path(script_dir, path_arg):
    if os.path.isabs(path_arg):
        return path_arg
    return os.path.join(script_dir, path_arg)


def read_stats_rows(csv_path, planner_type):
    rows = []
    with open(csv_path, "r", newline="") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            row_planner = row.get("planner_type", "")
            if planner_type and row_planner and row_planner != planner_type:
                continue
            rows.append({
                "planner_type": row_planner,
                "traj_id": int(row["traj_id"]),
                "plan_time_ms": float(row["plan_time_ms"]),
                "traj_length_m": float(row["traj_length_m"]),
                "duration_s": float(row["duration_s"]),
                "max_speed": float(row["max_speed"]),
                "max_curvature": float(row["max_curvature"]),
                "iterations": float(row["iterations"]),
                "success": float(row.get("success", 1)),
            })
    return rows


def read_trial_rows(csv_path):
    if not os.path.exists(csv_path):
        return []
    with open(csv_path, "r", newline="") as file_obj:
        return list(csv.DictReader(file_obj))


def describe(name, values, digits=2):
    fmt = f"{{:>10.{digits}f}}"
    return (
        f"{name:<25} "
        f"{fmt.format(np.mean(values))} "
        f"{fmt.format(np.std(values))} "
        f"{fmt.format(np.min(values))} "
        f"{fmt.format(np.max(values))}"
    )


def main():
    args = parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stats_path = resolve_path(script_dir, args.stats_file)
    trial_path = resolve_path(script_dir, args.trial_file)

    if not os.path.exists(stats_path):
        print(f"Error: {stats_path} not found.")
        sys.exit(1)

    rows = read_stats_rows(stats_path, args.planner_type)
    if not rows:
        print("Error: no planning stats rows matched the requested filter.")
        sys.exit(1)

    plan_time = np.array([row["plan_time_ms"] for row in rows], dtype=float)
    traj_length = np.array([row["traj_length_m"] for row in rows], dtype=float)
    duration = np.array([row["duration_s"] for row in rows], dtype=float)
    max_speed = np.array([row["max_speed"] for row in rows], dtype=float)
    max_curvature = np.array([row["max_curvature"] for row in rows], dtype=float)
    iterations = np.array([row["iterations"] for row in rows], dtype=float)

    max_curvature_capped = np.clip(max_curvature, 0.0, args.curv_cap)
    n_outliers = int(np.sum(max_curvature > args.curv_cap))
    planner_label = args.planner_type if args.planner_type else (rows[0]["planner_type"] or "mixed")

    print("=" * 60)
    print(f"  Planning Statistics ({planner_label})")
    print(f"  Total planning calls: {len(rows)}")
    if n_outliers > 0:
        print(f"  Curvature outliers (>{args.curv_cap} m^-1, capped): {n_outliers}")
    print("=" * 60)
    print()
    print(f"{'Metric':<25} {'Mean':>10} {'Std':>10} {'Min':>10} {'Max':>10}")
    print("-" * 65)
    print(describe("Plan time (ms)", plan_time, 2))
    print(describe("Traj time (s)", duration, 2))
    print(describe("Traj length (m)", traj_length, 2))
    print(describe("Max speed (m/s)", max_speed, 4))
    print(describe("Max curvature (m^-1)", max_curvature_capped, 4))
    print(describe("Iterations", iterations, 1))

    trial_rows = read_trial_rows(trial_path)
    rate = None
    if trial_rows:
        total_trials = len(trial_rows)
        success_trials = sum(int(row["success"]) for row in trial_rows)
        rate = 100.0 * success_trials / total_trials if total_trials > 0 else 0.0
        print(f"{'Success rate (%)':<25} {rate:>10.1f}  ({success_trials}/{total_trials})")
    else:
        print("\n  trial_results.csv not found - success rate unavailable")

    print()
    print("=" * 60)
    if args.format == "chapter4":
        print("  LaTeX table values (chapter4):")
        print("=" * 60)
        success_text = f"{rate:.1f}" if rate is not None else "--"
        print(f"  {planner_label} & {np.mean(plan_time):.2f} & {np.mean(duration):.2f} & {np.mean(traj_length):.2f} & {np.mean(max_curvature_capped):.4f} & {success_text} {LATEX_NL}")
    else:
        print("  LaTeX table values (chapter3):")
        print("=" * 60)
        print(f"  规划耗时 (ms)          & {np.mean(plan_time):.2f} & {np.std(plan_time):.2f} {LATEX_NL}")
        print(f"  轨迹时间 (s)           & {np.mean(duration):.2f} & {np.std(duration):.2f} {LATEX_NL}")
        print(f"  轨迹长度 (m)           & {np.mean(traj_length):.2f} & {np.std(traj_length):.2f} {LATEX_NL}")
        print(f"  最大曲率 (m^{{-1}})      & {np.mean(max_curvature_capped):.4f} & {np.std(max_curvature_capped):.4f} {LATEX_NL}")
        print(f"  最大速度 (m/s)         & {np.mean(max_speed):.4f} & {np.std(max_speed):.4f} {LATEX_NL}")
        print(f"  优化迭代次数           & {np.mean(iterations):.1f} & {np.std(iterations):.1f} {LATEX_NL}")
        if rate is not None:
            print(f"  规划成功率 (\\%)       & \\multicolumn{{2}}{{c}}{{{rate:.1f}}} {LATEX_NL}")


if __name__ == "__main__":
    main()
