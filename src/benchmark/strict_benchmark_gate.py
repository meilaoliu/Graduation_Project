#!/usr/bin/env python3
"""Summarize benchmark outputs against strict MINCO promotion gates."""

import argparse
import csv
import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class GateResult:
    passed: bool
    failures: list
    final_plan_successes: int
    final_plan_failures: int
    final_plan_success_rate: float
    arrival_successes: int
    arrival_total: int
    arrival_rate: float
    traj_reject_count: int
    obstacle_count: int
    emergency_stop_count: int
    curv_log_count: int
    max_restarts_count: int
    max_speed_mean: float
    max_speed_p95: float
    max_speed_max: float
    max_curvature_mean: float
    max_curvature_p95: float
    max_curvature_max: float
    plan_time_ms_mean: float
    plan_time_ms_p95: float
    plan_time_ms_max: float
    stop_go_ratio: float
    stop_go_samples: int
    cmd_samples: int


def _read_text(path):
    return Path(path).read_text(errors="ignore") if path else ""


def _count(pattern, text):
    return len(re.findall(pattern, text))


def _percent(numerator, denominator):
    return 100.0 * numerator / denominator if denominator else 0.0


def _float_values(rows, field):
    values = []
    for row in rows:
        try:
            values.append(float(row[field]))
        except (KeyError, TypeError, ValueError):
            continue
    return values


def _summary(values):
    if not values:
        return 0.0, 0.0, 0.0
    values = sorted(values)
    p95_index = int((len(values) - 1) * 0.95)
    return sum(values) / len(values), values[p95_index], values[-1]


def _read_csv_rows(path):
    if not path:
        return []
    csv_path = Path(path)
    if not csv_path.exists():
        return []
    with csv_path.open("r", newline="") as file_obj:
        return list(csv.DictReader(file_obj))


def _arrival_summary(trial_csv):
    rows = _read_csv_rows(trial_csv)
    successes = 0
    for row in rows:
        try:
            successes += int(row.get("success", "0"))
        except ValueError:
            continue
    return successes, len(rows), _percent(successes, len(rows))


def _stats_summary(stats_csv):
    rows = _read_csv_rows(stats_csv)
    speed_mean, speed_p95, speed_max = _summary(_float_values(rows, "max_speed"))
    curv_mean, curv_p95, curv_max = _summary(_float_values(rows, "max_curvature"))
    time_mean, time_p95, time_max = _summary(_float_values(rows, "plan_time_ms"))
    return speed_mean, speed_p95, speed_max, curv_mean, curv_p95, curv_max, time_mean, time_p95, time_max


def _stop_go_summary(cmd_csv, active_ref_speed=0.2, stopped_cmd_speed=0.05):
    rows = _read_csv_rows(cmd_csv)
    active_samples = 0
    stopped_samples = 0
    for row in rows:
        try:
            ref_speed = abs(float(row.get("ref_speed", "0")))
            cmd_v = abs(float(row.get("cmd_v", "0")))
        except ValueError:
            continue
        if ref_speed < active_ref_speed:
            continue
        active_samples += 1
        if cmd_v < stopped_cmd_speed:
            stopped_samples += 1
    return _percent(stopped_samples, active_samples) / 100.0, stopped_samples, active_samples


def evaluate(planner_log,
             trial_csv,
             stats_csv=None,
             cmd_csv=None,
             min_final_plan_success=95.0,
             require_full_arrival=True):
    planner_text = _read_text(planner_log)
    final_successes = _count(r"final_plan_success=1", planner_text)
    final_failures = _count(r"final_plan_success=0", planner_text)
    final_total = final_successes + final_failures
    final_rate = _percent(final_successes, final_total)

    arrival_successes, arrival_total, arrival_rate = _arrival_summary(trial_csv)
    traj_reject_count = _count(r"TRAJ_REJECT", planner_text)
    obstacle_count = _count(r"The drone is in obstacle", planner_text)
    emergency_stop_count = _count(r"Emergency stop", planner_text)
    curv_log_count = _count(r"\[CURV\]", planner_text)
    max_restarts_count = _count(r"Optimize failed: reason=max_restarts", planner_text)

    stats = _stats_summary(stats_csv)
    stop_go_ratio, stop_go_samples, cmd_samples = _stop_go_summary(cmd_csv)

    failures = []
    if final_rate < min_final_plan_success:
        failures.append(f"final_plan_success {final_rate:.2f}% < {min_final_plan_success:.2f}%")
    if require_full_arrival and arrival_successes != arrival_total:
        failures.append(f"arrival {arrival_successes}/{arrival_total} is not 100%")
    if traj_reject_count:
        failures.append(f"TRAJ_REJECT count {traj_reject_count} > 0")
    if obstacle_count:
        failures.append(f"obstacle penetration count {obstacle_count} > 0")

    return GateResult(
        passed=not failures,
        failures=failures,
        final_plan_successes=final_successes,
        final_plan_failures=final_failures,
        final_plan_success_rate=final_rate,
        arrival_successes=arrival_successes,
        arrival_total=arrival_total,
        arrival_rate=arrival_rate,
        traj_reject_count=traj_reject_count,
        obstacle_count=obstacle_count,
        emergency_stop_count=emergency_stop_count,
        curv_log_count=curv_log_count,
        max_restarts_count=max_restarts_count,
        max_speed_mean=stats[0],
        max_speed_p95=stats[1],
        max_speed_max=stats[2],
        max_curvature_mean=stats[3],
        max_curvature_p95=stats[4],
        max_curvature_max=stats[5],
        plan_time_ms_mean=stats[6],
        plan_time_ms_p95=stats[7],
        plan_time_ms_max=stats[8],
        stop_go_ratio=stop_go_ratio,
        stop_go_samples=stop_go_samples,
        cmd_samples=cmd_samples,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate strict MINCO benchmark promotion gates")
    parser.add_argument("--planner-log", required=True)
    parser.add_argument("--trial-csv", required=True)
    parser.add_argument("--stats-csv")
    parser.add_argument("--cmd-csv")
    parser.add_argument("--min-final-plan-success", type=float, default=95.0)
    parser.add_argument("--allow-partial-arrival", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    result = evaluate(
        planner_log=args.planner_log,
        trial_csv=args.trial_csv,
        stats_csv=args.stats_csv,
        cmd_csv=args.cmd_csv,
        min_final_plan_success=args.min_final_plan_success,
        require_full_arrival=not args.allow_partial_arrival,
    )
    if args.json:
        print(json.dumps(asdict(result), indent=2, sort_keys=True))
    else:
        status = "PASS" if result.passed else "FAIL"
        print(f"[Gate] {status}")
        print(f"final_plan_success={result.final_plan_successes}/"
              f"{result.final_plan_successes + result.final_plan_failures} "
              f"({result.final_plan_success_rate:.2f}%)")
        print(f"arrival={result.arrival_successes}/{result.arrival_total} ({result.arrival_rate:.2f}%)")
        print(f"TRAJ_REJECT={result.traj_reject_count}, obstacle={result.obstacle_count}, "
              f"emergency_stop={result.emergency_stop_count}, max_restarts={result.max_restarts_count}, "
              f"CURV_logs={result.curv_log_count}")
        print(f"max_speed mean/p95/max={result.max_speed_mean:.3f}/"
              f"{result.max_speed_p95:.3f}/{result.max_speed_max:.3f}")
        print(f"max_curvature mean/p95/max={result.max_curvature_mean:.3f}/"
              f"{result.max_curvature_p95:.3f}/{result.max_curvature_max:.3f}")
        print(f"plan_time_ms mean/p95/max={result.plan_time_ms_mean:.3f}/"
              f"{result.plan_time_ms_p95:.3f}/{result.plan_time_ms_max:.3f}")
        print(f"stop_go_ratio={result.stop_go_ratio:.3f} "
              f"({result.stop_go_samples}/{result.cmd_samples} active cmd samples)")
        for failure in result.failures:
            print(f"- {failure}")
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
