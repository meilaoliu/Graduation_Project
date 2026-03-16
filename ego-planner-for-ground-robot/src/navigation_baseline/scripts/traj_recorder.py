#!/usr/bin/env python3
"""
Trajectory recorder for move_base (DWA / TEB) experiments.

Records the actual executed trajectory for each navigation goal and computes
metrics compatible with the existing analyze_stats.py pipeline:
  - plan_time_ms  (average control loop period, converted to ms)
  - traj_length_m (integrated odometry path length)
  - duration_s    (wall-clock time from goal received to arrival/abort)
  - max_speed     (peak linear speed observed)
  - max_curvature (peak curvature observed)
  - iterations    (number of cmd_vel messages issued for this goal)
  - success       (1 if reached goal, 0 otherwise)

It also records the curvature-derivative RMS (smoothness metric used in
Chapter 4 table) in an extra column: curv_smoothness.

Usage:
  rosrun navigation_baseline traj_recorder.py _planner_type:=dwa _output_dir:=/path/to/benchmark
"""

import csv
import math
import os
import threading
import time

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry


class TrajectoryRecorder:
    def __init__(self):
        rospy.init_node("traj_recorder", anonymous=True)

        self.planner_type = rospy.get_param("~planner_type", "dwa")
        output_dir = rospy.get_param("~output_dir", "")
        if not output_dir:
            output_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "..", "benchmark"
            )
        self.output_dir = os.path.normpath(output_dir)

        self.stats_path = os.path.join(self.output_dir, "planning_stats.csv")
        self.lock = threading.Lock()
        self.reset_trial()

        self.traj_id = 0
        self.stats_rows = []

        rospy.Subscriber("/state_estimation", Odometry, self.odom_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)

        rospy.loginfo(f"[TrajRecorder] planner_type={self.planner_type}, output_dir={self.output_dir}")

    def reset_trial(self):
        self.positions = []       # [(x, y, yaw, stamp)]
        self.speeds = []          # [linear_speed]
        self.cmd_count = 0
        self.trial_start = None
        self.goal_active = False
        self.goal_x = None
        self.goal_y = None

    def odom_cb(self, msg):
        with self.lock:
            if not self.goal_active:
                return
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            stamp = msg.header.stamp.to_sec()
            self.positions.append((x, y, yaw, stamp))

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.speeds.append(math.sqrt(vx * vx + vy * vy))

    def goal_cb(self, msg):
        with self.lock:
            self.reset_trial()
            self.goal_active = True
            self.trial_start = time.time()
            self.goal_x = msg.pose.position.x
            self.goal_y = msg.pose.position.y
            self.traj_id += 1
            rospy.loginfo(f"[TrajRecorder] Goal #{self.traj_id}: "
                          f"({self.goal_x:.2f}, {self.goal_y:.2f})")

    def cmd_vel_cb(self, msg):
        with self.lock:
            if self.goal_active:
                self.cmd_count += 1

    def result_cb(self, msg):
        with self.lock:
            if not self.goal_active:
                return
            success = 1 if msg.status.status == 3 else 0  # SUCCEEDED=3
            self.finalize_trial(success)

    def finalize_trial(self, success):
        self.goal_active = False
        duration = time.time() - self.trial_start if self.trial_start else 0.0

        pts = self.positions
        if len(pts) < 3:
            rospy.logwarn("[TrajRecorder] Too few odometry samples, skipping metrics.")
            row = self._empty_row(success, duration)
            self.stats_rows.append(row)
            self._write_csv()
            return

        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])
        yaws = np.array([p[2] for p in pts])
        stamps = np.array([p[3] for p in pts])

        # Path length
        dx = np.diff(xs)
        dy = np.diff(ys)
        seg_len = np.sqrt(dx * dx + dy * dy)
        traj_length = float(np.sum(seg_len))

        # Max speed
        max_speed = float(np.max(self.speeds)) if self.speeds else 0.0

        # Curvature via finite differences of yaw and arc-length
        dt = np.diff(stamps)
        dt = np.where(dt < 1e-6, 1e-6, dt)
        dyaw = np.diff(yaws)
        dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))  # normalize to [-pi, pi]
        ds = seg_len.copy()
        ds = np.where(ds < 1e-6, 1e-6, ds)
        curvature = np.abs(dyaw / ds)
        max_curvature = float(np.min([np.max(curvature), 50.0]))

        # Curvature smoothness: RMS of curvature change rate dkappa/ds
        if len(curvature) >= 2:
            dk = np.diff(curvature)
            ds2 = ds[:-1]
            ds2 = np.where(ds2 < 1e-6, 1e-6, ds2)
            dkds = dk / ds2
            curv_smoothness = float(np.sqrt(np.mean(dkds * dkds)))
        else:
            curv_smoothness = 0.0

        # Planning frequency (cmd_vel count / duration)
        plan_freq = self.cmd_count / duration if duration > 0 else 0.0
        plan_time_ms = (1000.0 / plan_freq) if plan_freq > 0 else 0.0

        row = {
            "planner_type": self.planner_type,
            "traj_id": self.traj_id,
            "plan_time_ms": round(plan_time_ms, 2),
            "traj_length_m": round(traj_length, 2),
            "duration_s": round(duration, 2),
            "max_speed": round(max_speed, 4),
            "max_curvature": round(max_curvature, 4),
            "iterations": self.cmd_count,
            "success": success,
            "curv_smoothness": round(curv_smoothness, 4),
        }
        self.stats_rows.append(row)
        self._write_csv()

        status_str = "SUCCESS" if success else "FAILED"
        rospy.loginfo(
            f"[TrajRecorder] Goal #{self.traj_id} {status_str}: "
            f"len={traj_length:.2f}m, dur={duration:.2f}s, "
            f"max_v={max_speed:.2f}m/s, max_k={max_curvature:.4f}, "
            f"smooth={curv_smoothness:.4f}"
        )

    def _empty_row(self, success, duration):
        return {
            "planner_type": self.planner_type,
            "traj_id": self.traj_id,
            "plan_time_ms": 0.0,
            "traj_length_m": 0.0,
            "duration_s": round(duration, 2),
            "max_speed": 0.0,
            "max_curvature": 0.0,
            "iterations": self.cmd_count,
            "success": success,
            "curv_smoothness": 0.0,
        }

    def _write_csv(self):
        fieldnames = [
            "planner_type", "traj_id", "plan_time_ms", "traj_length_m",
            "duration_s", "max_speed", "max_curvature", "iterations",
            "success", "curv_smoothness",
        ]
        with open(self.stats_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.stats_rows)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        recorder = TrajectoryRecorder()
        recorder.spin()
    except rospy.ROSInterruptException:
        pass
