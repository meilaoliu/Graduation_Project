# -*- coding: utf-8 -*-
"""Local world model for task-level agent decisions."""

from collections import deque
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple

from .schemas import RobotState, RuntimeEvent, TaskStateSnapshot


class WorldModel:
    """Keeps compact robot, task, and runtime facts for the agent loop."""

    def __init__(self, max_events: int = 40):
        self.max_events = int(max(1, max_events))
        self.robot_state = RobotState()
        self.task_state = TaskStateSnapshot()
        self.events: Deque[RuntimeEvent] = deque(maxlen=self.max_events)

    def start_task(
        self,
        task_id: str,
        targets: Iterable[str],
        current_target: Optional[str] = None,
        time_budget: Optional[Dict[str, Any]] = None,
    ) -> None:
        target_list = [target for target in targets if target]
        self.task_state = TaskStateSnapshot(
            task_id=task_id,
            active=True,
            phase="RUNNING",
            current_target=current_target or (target_list[0] if target_list else None),
            remaining_targets=target_list,
            time_budget=dict(time_budget or {}),
        )

    def finish_task(self) -> None:
        self.task_state.active = False
        self.task_state.phase = "FINISHED"
        self.task_state.current_target = None
        self.task_state.failed_target = None
        self.task_state.remaining_targets = []

    def stop_task(self, phase: str = "STOPPED") -> None:
        self.task_state.active = False
        self.task_state.phase = phase
        self.task_state.current_target = None

    def update_robot_state(
        self,
        current_node: Optional[str] = None,
        current_xy: Optional[Tuple[float, float]] = None,
        battery_pct: Optional[float] = None,
        charging: Optional[bool] = None,
        low_battery: Optional[bool] = None,
    ) -> None:
        if current_node:
            self.robot_state.current_node = current_node
            self.robot_state.last_reachable_node = current_node
        if current_xy is not None:
            self.robot_state.current_xy = current_xy
        if battery_pct is not None:
            self.robot_state.battery_pct = float(battery_pct)
        if charging is not None:
            self.robot_state.charging = bool(charging)
        if low_battery is not None:
            self.robot_state.low_battery = bool(low_battery)

    def update_from_event(self, event: Dict[str, Any]) -> RuntimeEvent:
        runtime_event = RuntimeEvent.from_dict(event)
        self.events.append(runtime_event)
        event_type = runtime_event.event_type
        data = runtime_event.data

        battery_pct = data.get("battery_pct", data.get("battery_percent"))
        if battery_pct is not None:
            try:
                self.robot_state.battery_pct = float(battery_pct)
            except (TypeError, ValueError):
                pass

        current_node = data.get("current_node") or data.get("last_reachable_node")
        if current_node:
            self.robot_state.current_node = str(current_node)
            self.robot_state.last_reachable_node = str(current_node)

        current_xy = data.get("current_xy")
        if isinstance(current_xy, (list, tuple)) and len(current_xy) >= 2:
            try:
                self.robot_state.current_xy = (float(current_xy[0]), float(current_xy[1]))
            except (TypeError, ValueError):
                pass

        estimated_range = data.get("estimated_remaining_distance_m")
        if estimated_range is not None:
            try:
                self.robot_state.estimated_remaining_distance_m = float(estimated_range)
            except (TypeError, ValueError):
                pass

        current_target = data.get("current_target") or data.get("target") or data.get("failed_target")
        if current_target:
            self.task_state.current_target = str(current_target)

        remaining_targets = data.get("remaining_targets")
        if isinstance(remaining_targets, list):
            self.task_state.remaining_targets = [str(target) for target in remaining_targets if target]

        event_time_budget = data.get("time_budget")
        if isinstance(event_time_budget, dict):
            self.task_state.time_budget.update(event_time_budget)
        for key in ("deadline_epoch", "remaining_time_s", "time_budget_cycle", "time_budget_target_cursor", "elapsed_s"):
            if key in data:
                self.task_state.time_budget[key] = data[key]

        if event_type in {"segment_failed", "planning_failed"}:
            failed_target = data.get("failed_target") or data.get("target") or data.get("current_target")
            if failed_target:
                self.record_failure(str(failed_target))
            self.task_state.phase = "NEEDS_DECISION"
        elif event_type == "progress_anomaly":
            suspect_target = data.get("failed_target") or data.get("target") or data.get("current_target")
            if suspect_target:
                self.task_state.failed_target = str(suspect_target)
            self.task_state.phase = "NEEDS_DECISION"
        elif event_type in {"task_finished", "task_completed"}:
            self.finish_task()
        elif event_type in {"task_stopped", "task_aborted"}:
            self.stop_task("STOPPED")
        elif event_type in {"battery_low", "low_battery_alert", "energy_guard"}:
            self.robot_state.low_battery = True
            self.task_state.phase = "ENERGY_GUARD"
        elif event_type == "charge_completed":
            self.robot_state.charging = False
            self.robot_state.low_battery = False

        return runtime_event

    def record_failure(self, target: str) -> None:
        self.task_state.failed_target = target
        self.task_state.failure_counts[target] = self.task_state.failure_counts.get(target, 0) + 1

    def snapshot(self) -> Dict[str, Any]:
        return {
            "robot": self.robot_state.to_dict(),
            "task": self.task_state.to_dict(),
            "events": [event.to_dict() for event in self.events],
        }

    def recent_events(self, limit: int = 10) -> List[Dict[str, Any]]:
        limit = int(max(0, limit))
        if limit == 0:
            return []
        return [event.to_dict() for event in list(self.events)[-limit:]]

    def to_prompt_context(self, max_events: int = 10) -> str:
        robot = self.robot_state
        task = self.task_state
        lines = [
            f"机器人: current_node={robot.current_node}, last_reachable={robot.last_reachable_node}, battery={robot.battery_pct:.1f}%, low_battery={robot.low_battery}",
            f"估计剩余可走: {robot.estimated_remaining_distance_m if robot.estimated_remaining_distance_m is not None else '未知'}m",
            f"任务: active={task.active}, phase={task.phase}, current_target={task.current_target}, failed_target={task.failed_target}",
            f"剩余目标: {task.remaining_targets}",
            f"失败计数: {task.failure_counts}",
        ]
        if task.time_budget:
            lines.append(f"时间预算: {task.time_budget}")
        events = self.recent_events(max_events)
        if events:
            lines.append("近期事件:")
            for event in events:
                lines.append(f"- {event.get('event_type')}: {event.get('message')} {event.get('data')}")
        return "\n".join(lines)
