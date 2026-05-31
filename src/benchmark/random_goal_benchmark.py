#!/usr/bin/env python3
"""
Benchmark runner for fixed-goal planning experiments.

Usage examples:
  python3 random_goal_benchmark.py --generate-only --num-goals 50
  python3 random_goal_benchmark.py --num-goals 50 --tag bspline
  python3 random_goal_benchmark.py --num-goals 50 --goal-file fixed_goals.csv --tag minco
"""

import argparse
import csv
import math
import os
import random
import shutil
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


# Zones spread around the perimeter of the 40m×40m map.
# Obstacles fill [-16.5, 16.5], walls at ±20.
# Each zone is tagged with a region (NW/N/NE/W/E/SW/S/SE).
# Consecutive goals must be in "opposite" regions so the path always
# crosses through the obstacle-dense centre — never along the same edge.
#
#   MAP_MIN/MAX: hard coordinate clamp (stay inside walls)
MAP_MIN, MAP_MAX = -19.5, 19.5

ZONES = [
    # Corners
    {"name": "NW", "region": "NW", "cx": -17.0, "cy":  17.0, "radius": 1.8},
    {"name": "NE", "region": "NE", "cx":  17.0, "cy":  17.0, "radius": 1.8},
    {"name": "SW", "region": "SW", "cx": -17.0, "cy": -17.0, "radius": 1.8},
    {"name": "SE", "region": "SE", "cx":  17.0, "cy": -17.0, "radius": 1.8},
    # Edge midpoints
    {"name": "N",  "region": "N",  "cx":   0.0, "cy":  18.0, "radius": 1.8},
    {"name": "S",  "region": "S",  "cx":   0.0, "cy": -18.0, "radius": 1.8},
    {"name": "W",  "region": "W",  "cx": -18.0, "cy":   0.0, "radius": 1.8},
    {"name": "E",  "region": "E",  "cx":  18.0, "cy":   0.0, "radius": 1.8},
]

# For each region, list which regions are valid NEXT targets.
# Rule: the path between them must cross the obstacle-dense centre.
# Forbidden: same region, or adjacent regions that share an edge
#   (e.g. NW→N, NW→W, N→NE, etc. — robot can hug the perimeter).
VALID_NEXT = {
    "NW": ["SE", "S", "E"],         # diagonal or far opposite
    "NE": ["SW", "S", "W"],
    "SW": ["NE", "N", "E"],
    "SE": ["NW", "N", "W"],
    "N":  ["S", "SW", "SE"],         # cross-map vertical + diagonal
    "S":  ["N", "NW", "NE"],
    "W":  ["E", "NE", "SE"],         # cross-map horizontal + diagonal
    "E":  ["W", "NW", "SW"],
}

# Build a lookup: region -> list of zones in that region
_REGION_TO_ZONES = {}
for _z in ZONES:
    _REGION_TO_ZONES.setdefault(_z["region"], []).append(_z)


def parse_args():
    parser = argparse.ArgumentParser(description="Run fixed-goal planning benchmark")
    parser.add_argument("--num-goals", type=int, default=50, help="Number of fixed goals to use")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for fixed-goal generation")
    parser.add_argument("--timeout", type=float, default=120.0, help="Per-goal timeout in seconds")
    parser.add_argument("--settle-time", type=float, default=4.0, help="Wait time after each trial")
    parser.add_argument("--arrival-threshold", type=float, default=1.5, help="Arrival distance threshold in meters")
    parser.add_argument("--goal-file", type=str, default="fixed_goals.csv", help="CSV file storing reusable fixed goals")
    parser.add_argument("--generate-only", action="store_true", help="Only generate the goal file, do not run benchmark")
    parser.add_argument("--regenerate", action="store_true", help="Regenerate goal file even if it already exists")
    parser.add_argument("--tag", type=str, default="", help="Optional suffix for copied output CSV files")
    return parser.parse_args()


def sample_goal_in_zone(zone, rng):
    """Sample a random point inside a circular zone, clamped to map bounds."""
    radius = zone["radius"] * math.sqrt(rng.uniform(0.05, 0.85))
    theta = rng.uniform(0.0, 2.0 * math.pi)
    x = max(MAP_MIN, min(MAP_MAX, zone["cx"] + radius * math.cos(theta)))
    y = max(MAP_MIN, min(MAP_MAX, zone["cy"] + radius * math.sin(theta)))
    return x, y


def generate_fixed_goals(num_goals, seed):
    """Generate goals that always cross through the obstacle-dense centre.

    Each consecutive goal is placed in a region that is far from the
    previous one (diagonal, opposite side, etc.), so the robot cannot
    travel along the empty perimeter.  Valid transitions are defined
    by the VALID_NEXT table.

    Examples of generated paths:
      NW → SE (diagonal), SE → N (diagonal), N → SW (diagonal),
      W → NE (diagonal),  S → NW (diagonal), E → SW (diagonal), ...
    """
    rng = random.Random(seed)
    goals = []

    # Start from a random region
    current_region = rng.choice(list(VALID_NEXT.keys()))

    for index in range(num_goals):
        zone = rng.choice(_REGION_TO_ZONES[current_region])
        goal_x, goal_y = sample_goal_in_zone(zone, rng)
        goals.append({
            "goal_id": index + 1,
            "zone": zone["name"],
            "goal_x": round(goal_x, 2),
            "goal_y": round(goal_y, 2),
        })
        # Pick next region from valid opposites
        current_region = rng.choice(VALID_NEXT[current_region])

    return goals


def save_goals(goal_file, goals):
    with open(goal_file, "w", newline="") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=["goal_id", "zone", "goal_x", "goal_y"])
        writer.writeheader()
        writer.writerows(goals)


def load_goals(goal_file, expected_num_goals):
    with open(goal_file, "r", newline="") as file_obj:
        rows = list(csv.DictReader(file_obj))

    if expected_num_goals > 0 and len(rows) != expected_num_goals:
        raise ValueError(f"goal file contains {len(rows)} goals, expected {expected_num_goals}")

    goals = []
    for row in rows:
        goals.append({
            "goal_id": int(row["goal_id"]),
            "zone": row["zone"],
            "goal_x": float(row["goal_x"]),
            "goal_y": float(row["goal_y"]),
        })
    return goals


class GoalBenchmark:
    def __init__(self, args, goals, script_dir):
        self.args = args
        self.goals = goals
        self.script_dir = script_dir
        self.robot_pos = None

        rospy.init_node("goal_benchmark", anonymous=True)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber("/state_estimation", Odometry, self.odom_cb)

        rospy.loginfo("[Benchmark] Waiting for odometry...")
        while self.robot_pos is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("[Benchmark] Waiting for goal subscriber...")
        while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.stats_file = os.path.join(script_dir, "planning_stats.csv")
        self.trial_file = os.path.join(script_dir, "trial_results.csv")
        for path in (self.stats_file, self.trial_file):
            if os.path.exists(path):
                os.remove(path)
                rospy.loginfo(f"[Benchmark] Removed old {path}")

    def odom_cb(self, msg):
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])

    def publish_goal(self, goal_x, goal_y):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def wait_for_arrival(self, goal_x, goal_y):
        start_time = time.time()
        while not rospy.is_shutdown():
            if self.robot_pos is not None:
                dist = math.hypot(self.robot_pos[0] - goal_x, self.robot_pos[1] - goal_y)
                if dist < self.args.arrival_threshold:
                    return True
            if time.time() - start_time > self.args.timeout:
                return False
            rospy.sleep(0.1)
        return False

    def copy_outputs_with_tag(self):
        if not self.args.tag:
            return

        tagged_stats = os.path.join(self.script_dir, f"planning_stats_{self.args.tag}.csv")
        tagged_trials = os.path.join(self.script_dir, f"trial_results_{self.args.tag}.csv")
        if os.path.exists(self.stats_file):
            shutil.copyfile(self.stats_file, tagged_stats)
        if os.path.exists(self.trial_file):
            shutil.copyfile(self.trial_file, tagged_trials)
        rospy.loginfo(f"[Benchmark] Tagged outputs saved with suffix {self.args.tag}")

    def run(self):
        rospy.loginfo(f"[Benchmark] Starting {len(self.goals)} fixed-goal trials")
        results = []
        success_count = 0

        for trial_index, goal in enumerate(self.goals, start=1):
            if rospy.is_shutdown():
                break

            goal_x = goal["goal_x"]
            goal_y = goal["goal_y"]
            rospy.loginfo(
                f"[Trial {trial_index}/{len(self.goals)}] goal_id={goal['goal_id']} zone={goal['zone']} "
                f"target=({goal_x:.2f}, {goal_y:.2f})"
            )

            self.publish_goal(goal_x, goal_y)
            rospy.sleep(0.5)
            self.publish_goal(goal_x, goal_y)

            arrived = self.wait_for_arrival(goal_x, goal_y)
            success_count += int(arrived)
            results.append({
                "trial": trial_index,
                "goal_id": goal["goal_id"],
                "zone": goal["zone"],
                "goal_x": goal_x,
                "goal_y": goal_y,
                "success": int(arrived),
            })

            if arrived:
                rospy.loginfo("[Benchmark] SUCCESS")
            else:
                rospy.logwarn("[Benchmark] TIMEOUT / FAILED")

            rospy.sleep(self.args.settle_time)

        with open(self.trial_file, "w", newline="") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=["trial", "goal_id", "zone", "goal_x", "goal_y", "success"])
            writer.writeheader()
            writer.writerows(results)

        total = len(results)
        rate = 100.0 * success_count / total if total else 0.0
        rospy.loginfo(f"[Benchmark] Complete: {success_count}/{total} succeeded ({rate:.1f}%)")
        rospy.loginfo(f"[Benchmark] Trial results saved to {self.trial_file}")
        rospy.loginfo(f"[Benchmark] Planning stats saved to {self.stats_file}")
        self.copy_outputs_with_tag()


def main():
    args = parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    goal_file = args.goal_file if os.path.isabs(args.goal_file) else os.path.join(script_dir, args.goal_file)

    if args.regenerate or not os.path.exists(goal_file):
        goals = generate_fixed_goals(args.num_goals, args.seed)
        save_goals(goal_file, goals)
        print(f"Saved {len(goals)} fixed goals to {goal_file}")
    else:
        goals = load_goals(goal_file, args.num_goals)
        print(f"Loaded {len(goals)} fixed goals from {goal_file}")

    if args.generate_only:
        return

    benchmark = GoalBenchmark(args, goals, script_dir)
    benchmark.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
