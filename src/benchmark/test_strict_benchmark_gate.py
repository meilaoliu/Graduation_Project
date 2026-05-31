#!/usr/bin/env python3

import csv
import tempfile
import unittest
from pathlib import Path

import strict_benchmark_gate


class StrictBenchmarkGateTest(unittest.TestCase):
    def write_text(self, directory, name, content):
        path = directory / name
        path.write_text(content)
        return path

    def write_csv(self, directory, name, fieldnames, rows):
        path = directory / name
        with path.open("w", newline="") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_passes_only_when_strict_goal_metrics_are_met(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            planner_log = self.write_text(
                directory,
                "planner.log",
                "\n".join([
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=1",
                    "final_plan_success=0",
                    "[WARN] [CURV] ki~=3.1",
                    "[WARN] [MINCO] Optimize failed: reason=max_restarts",
                ]),
            )
            trials = self.write_csv(
                directory,
                "trial_results.csv",
                ["trial", "goal_id", "zone", "goal_x", "goal_y", "success"],
                [
                    {"trial": "1", "goal_id": "1", "zone": "NW", "goal_x": "1", "goal_y": "2", "success": "1"},
                    {"trial": "2", "goal_id": "2", "zone": "SE", "goal_x": "3", "goal_y": "4", "success": "1"},
                ],
            )
            stats = self.write_csv(
                directory,
                "planning_stats.csv",
                ["planner_type", "traj_id", "plan_time_ms", "max_speed", "max_curvature", "success"],
                [
                    {"planner_type": "minco", "traj_id": "1", "plan_time_ms": "2.0", "max_speed": "1.2", "max_curvature": "0.5", "success": "1"},
                    {"planner_type": "minco", "traj_id": "2", "plan_time_ms": "4.0", "max_speed": "1.3", "max_curvature": "1.5", "success": "1"},
                ],
            )
            cmd = self.write_csv(
                directory,
                "cmd_profile.csv",
                ["time", "ref_speed", "cmd_v", "cmd_w"],
                [
                    {"time": "0.0", "ref_speed": "0.8", "cmd_v": "0.7", "cmd_w": "0.1"},
                    {"time": "0.1", "ref_speed": "0.8", "cmd_v": "0.02", "cmd_w": "0.1"},
                    {"time": "0.2", "ref_speed": "0.8", "cmd_v": "0.01", "cmd_w": "0.1"},
                ],
            )

            result = strict_benchmark_gate.evaluate(
                planner_log=planner_log,
                trial_csv=trials,
                stats_csv=stats,
                cmd_csv=cmd,
                min_final_plan_success=95.0,
                require_full_arrival=True,
            )

        self.assertFalse(result.passed)
        self.assertAlmostEqual(result.final_plan_success_rate, 90.0)
        self.assertEqual(result.arrival_successes, 2)
        self.assertEqual(result.arrival_total, 2)
        self.assertAlmostEqual(result.arrival_rate, 100.0)
        self.assertEqual(result.traj_reject_count, 0)
        self.assertEqual(result.obstacle_count, 0)
        self.assertEqual(result.curv_log_count, 1)
        self.assertEqual(result.max_restarts_count, 1)
        self.assertAlmostEqual(result.max_curvature_p95, 0.5)
        self.assertAlmostEqual(result.max_curvature_max, 1.5)
        self.assertAlmostEqual(result.stop_go_ratio, 2.0 / 3.0)
        self.assertIn("final_plan_success 90.00% < 95.00%", result.failures)


if __name__ == "__main__":
    unittest.main()
