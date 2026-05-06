# -*- coding: utf-8 -*-
"""Deterministic segment progress monitor for runtime anomaly events."""

import math
from typing import Dict, Iterable, List, Optional, Tuple


Point = Tuple[float, float]


class SegmentProgressMonitor:
    """Detects suspicious segment execution without making task decisions."""

    def __init__(
        self,
        enabled: bool = True,
        sample_interval_s: float = 1.0,
        warmup_s: float = 8.0,
        min_progress_m: float = 0.25,
        min_motion_m: float = 0.08,
        no_progress_timeout_s: float = 25.0,
        stuck_timeout_s: float = 18.0,
        moving_away_timeout_s: float = 15.0,
        moving_away_margin_m: float = 1.0,
        path_deviation_m: float = 3.0,
        path_deviation_timeout_s: float = 8.0,
        arrive_tolerance_m: float = 0.5,
    ):
        self.enabled = bool(enabled)
        self.sample_interval_s = max(float(sample_interval_s), 0.05)
        self.warmup_s = max(float(warmup_s), 0.0)
        self.min_progress_m = max(float(min_progress_m), 0.01)
        self.min_motion_m = max(float(min_motion_m), 0.01)
        self.no_progress_timeout_s = max(float(no_progress_timeout_s), 1.0)
        self.stuck_timeout_s = max(float(stuck_timeout_s), 1.0)
        self.moving_away_timeout_s = max(float(moving_away_timeout_s), 1.0)
        self.moving_away_margin_m = max(float(moving_away_margin_m), 0.1)
        self.path_deviation_m = max(float(path_deviation_m), 0.1)
        self.path_deviation_timeout_s = max(float(path_deviation_timeout_s), 1.0)
        self.arrive_tolerance_m = max(float(arrive_tolerance_m), 0.05)
        self.reset()

    def reset(self) -> None:
        self.segment_id: Optional[int] = None
        self.target: str = ""
        self.route_points: List[Point] = []
        self.end_xy: Optional[Point] = None
        self.start_time_s: Optional[float] = None
        self.last_sample_time_s: Optional[float] = None
        self.last_xy: Optional[Point] = None
        self.last_motion_time_s: Optional[float] = None
        self.last_progress_time_s: Optional[float] = None
        self.best_progress_m = 0.0
        self.best_distance_to_goal_m: Optional[float] = None
        self.total_route_m = 0.0
        self.path_deviation_since_s: Optional[float] = None
        self.moving_away_since_s: Optional[float] = None
        self.emitted_reasons = set()

    def start(
        self,
        segment_id: int,
        target: str,
        route_points: Iterable[Point],
        end_xy: Point,
        now_s: float,
    ) -> None:
        self.reset()
        self.segment_id = int(segment_id)
        self.target = str(target or "")
        self.route_points = [self._point(point) for point in route_points]
        if not self.route_points:
            self.route_points = [self._point(end_xy)]
        self.end_xy = self._point(end_xy)
        self.start_time_s = float(now_s)
        self.last_sample_time_s = None
        self.last_motion_time_s = float(now_s)
        self.last_progress_time_s = float(now_s)
        self.total_route_m = self._polyline_length(self.route_points)

    def update(self, current_xy: Point, now_s: float) -> Optional[Dict[str, object]]:
        if not self.enabled or self.start_time_s is None or self.end_xy is None:
            return None

        now_s = float(now_s)
        if self.last_sample_time_s is not None and now_s - self.last_sample_time_s < self.sample_interval_s:
            return None
        self.last_sample_time_s = now_s

        current = self._point(current_xy)
        progress_m, deviation_m = self._project_to_route(current)
        distance_to_goal_m = self._distance(current, self.end_xy)
        elapsed_s = now_s - self.start_time_s

        if self.last_xy is None:
            self.last_xy = current
            self.best_progress_m = progress_m
            self.best_distance_to_goal_m = distance_to_goal_m
            return None

        if self._distance(current, self.last_xy) >= self.min_motion_m:
            self.last_xy = current
            self.last_motion_time_s = now_s

        if progress_m > self.best_progress_m + self.min_progress_m:
            self.best_progress_m = progress_m
            self.last_progress_time_s = now_s

        if self.best_distance_to_goal_m is None or distance_to_goal_m < self.best_distance_to_goal_m - self.min_progress_m:
            self.best_distance_to_goal_m = distance_to_goal_m
            self.moving_away_since_s = None

        if elapsed_s < self.warmup_s or distance_to_goal_m <= self.arrive_tolerance_m:
            return None

        no_progress_s = now_s - (self.last_progress_time_s or self.start_time_s)
        if deviation_m > self.path_deviation_m and no_progress_s >= self.path_deviation_timeout_s:
            return self._event("path_deviation", now_s, current, progress_m, deviation_m, distance_to_goal_m)
        else:
            self.path_deviation_since_s = None

        best_goal = self.best_distance_to_goal_m if self.best_distance_to_goal_m is not None else distance_to_goal_m
        if distance_to_goal_m > best_goal + self.moving_away_margin_m:
            if self.moving_away_since_s is None:
                self.moving_away_since_s = now_s
            elif now_s - self.moving_away_since_s >= self.moving_away_timeout_s:
                return self._event("moving_away", now_s, current, progress_m, deviation_m, distance_to_goal_m)
        else:
            self.moving_away_since_s = None

        if now_s - (self.last_motion_time_s or self.start_time_s) >= self.stuck_timeout_s:
            return self._event("stuck", now_s, current, progress_m, deviation_m, distance_to_goal_m)

        if now_s - (self.last_progress_time_s or self.start_time_s) >= self.no_progress_timeout_s:
            return self._event("no_progress", now_s, current, progress_m, deviation_m, distance_to_goal_m)

        return None

    def should_wall_clock_timeout_fail(self, now_s: float) -> bool:
        if not self.enabled or self.start_time_s is None:
            return True
        no_motion_s = float(now_s) - (self.last_motion_time_s or self.start_time_s)
        no_progress_s = float(now_s) - (self.last_progress_time_s or self.start_time_s)
        return no_motion_s >= self.stuck_timeout_s or no_progress_s >= self.no_progress_timeout_s

    def _event(
        self,
        reason: str,
        now_s: float,
        current: Point,
        progress_m: float,
        deviation_m: float,
        distance_to_goal_m: float,
    ) -> Optional[Dict[str, object]]:
        if reason in self.emitted_reasons:
            return None
        self.emitted_reasons.add(reason)
        elapsed_s = now_s - (self.start_time_s or now_s)
        no_progress_s = now_s - (self.last_progress_time_s or self.start_time_s or now_s)
        no_motion_s = now_s - (self.last_motion_time_s or self.start_time_s or now_s)
        return {
            "reason": reason,
            "segment_elapsed_s": round(elapsed_s, 1),
            "distance_to_goal_m": round(distance_to_goal_m, 2),
            "best_distance_to_goal_m": round(self.best_distance_to_goal_m or distance_to_goal_m, 2),
            "route_progress_m": round(progress_m, 2),
            "best_route_progress_m": round(self.best_progress_m, 2),
            "total_route_m": round(self.total_route_m, 2),
            "path_deviation_m": round(deviation_m, 2),
            "no_progress_s": round(max(no_progress_s, 0.0), 1),
            "no_motion_s": round(max(no_motion_s, 0.0), 1),
            "current_xy": [round(current[0], 3), round(current[1], 3)],
        }

    def _project_to_route(self, point: Point) -> Tuple[float, float]:
        if len(self.route_points) == 1:
            return 0.0, self._distance(point, self.route_points[0])

        best_progress = 0.0
        best_distance = float("inf")
        accumulated = 0.0
        for start, end in zip(self.route_points, self.route_points[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-9:
                distance = self._distance(point, start)
                progress = accumulated
            else:
                t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
                t = min(max(t, 0.0), 1.0)
                projection = (start[0] + t * dx, start[1] + t * dy)
                segment_len = math.sqrt(length_sq)
                distance = self._distance(point, projection)
                progress = accumulated + t * segment_len
            if distance < best_distance:
                best_distance = distance
                best_progress = progress
            accumulated += math.sqrt(length_sq) if length_sq > 0.0 else 0.0
        return best_progress, best_distance

    @staticmethod
    def _polyline_length(points: List[Point]) -> float:
        return sum(SegmentProgressMonitor._distance(a, b) for a, b in zip(points, points[1:]))

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _point(value: Point) -> Point:
        return float(value[0]), float(value[1])