# -*- coding: utf-8 -*-
"""Aggregate entry point for the local task agent runtime."""

from typing import Any, Callable, Dict, List, Optional

from ..runtime_policy import BatteryPolicy
from .action_executor import ActionExecutor
from .autonomous_supervisor import AutonomousSupervisor
from .safety_validator import SafetyValidator
from .schemas import ActionResult, SupervisorRunResult
from .tool_registry import ToolRegistry
from .world_model import WorldModel


class TaskAgentRuntime:
    """Coordinates world state, tools, supervision, validation, and execution."""

    DEFAULT_TRIGGER_EVENTS = {
        "segment_failed",
        "time_budget_extend_failed",
        "progress_anomaly",
    }

    def __init__(
        self,
        path_planner: Any,
        handle_navigation_request: Callable[..., str],
        segment_scheduler: Any = None,
        waypoint_manager: Any = None,
        llm_callable: Optional[Callable[[List[Dict[str, Any]], List[Dict[str, Any]]], Any]] = None,
        say: Optional[Callable[[str], None]] = None,
        enabled: bool = True,
        max_tool_iterations: int = 6,
        min_tool_calls_before_final: int = 2,
        min_confidence: float = 0.35,
        max_retry_per_target: int = 1,
        max_consecutive_failures: int = 3,
        nominal_speed_mps: float = 0.8,
        battery_full_range_m: float = 300.0,
        battery_reserve_m: float = 15.0,
        trigger_events: Optional[List[str]] = None,
    ):
        self.enabled = bool(enabled)
        self.path_planner = path_planner
        self.world_model = WorldModel()
        self.say = say or (lambda message: None)
        self.trigger_events = set(trigger_events or self.DEFAULT_TRIGGER_EVENTS)
        self.battery_policy = BatteryPolicy(
            full_range_m=battery_full_range_m,
            reserve_m=battery_reserve_m,
        )
        self.tool_registry = ToolRegistry(
            path_planner=path_planner,
            world_model=self.world_model,
            battery_policy=self.battery_policy,
            nominal_speed_mps=nominal_speed_mps,
        )
        self.safety_validator = SafetyValidator(
            path_planner=path_planner,
            world_model=self.world_model,
            battery_policy=self.battery_policy,
            min_confidence=min_confidence,
            max_retry_per_target=max_retry_per_target,
            max_consecutive_failures=max_consecutive_failures,
        )
        charge_point_name, _, _ = path_planner.graph.get_charge_point()
        self.action_executor = ActionExecutor(
            world_model=self.world_model,
            handle_navigation_request=handle_navigation_request,
            segment_scheduler=segment_scheduler,
            waypoint_manager=waypoint_manager,
            say=say,
            charge_point_name=charge_point_name,
        )
        self.supervisor = AutonomousSupervisor(
            world_model=self.world_model,
            tool_registry=self.tool_registry,
            safety_validator=self.safety_validator,
            llm_callable=llm_callable,
            max_tool_iterations=max_tool_iterations,
            min_tool_calls_before_final=min_tool_calls_before_final,
        )
        self.last_run_result: Optional[SupervisorRunResult] = None
        self.last_action_result: Optional[ActionResult] = None
        self.decision_history: List[Dict[str, Any]] = []

    def on_user_command(self, command: str, normalized_plan: Dict[str, Any]) -> None:
        if not normalized_plan:
            return
        targets = [target.get("name") for target in normalized_plan.get("targets", []) if isinstance(target, dict)]
        if not targets:
            targets = list(normalized_plan.get("target_names", []))
        self.world_model.start_task(
            task_id=f"user:{len(self.decision_history) + 1}",
            targets=[target for target in targets if target],
            current_target=targets[0] if targets else None,
            time_budget=normalized_plan.get("execution") or {},
        )
        self.world_model.update_from_event(
            {
                "event_type": "user_command_planned",
                "message": command,
                "data": {
                    "task_type": normalized_plan.get("task_type"),
                    "remaining_targets": targets,
                    "execution": normalized_plan.get("execution") or {},
                },
            }
        )

    def should_handle_event(self, event: Dict[str, Any]) -> bool:
        if not self.enabled:
            return False
        data = event.get("data") or {}
        if event.get("event_type") == "progress_anomaly" and data.get("kind") == "return_charge":
            return False
        return str(event.get("event_type", "")) in self.trigger_events

    def observe_event(self, event: Dict[str, Any]) -> None:
        self.world_model.update_from_event(event)

    def on_runtime_event(self, event: Dict[str, Any], execute: bool = True) -> Optional[Dict[str, Any]]:
        if not self.enabled:
            self.observe_event(event)
            return None
        if not self.should_handle_event(event):
            self.observe_event(event)
            return None

        run_result = self.supervisor.run(event, update_world=True)
        self.last_run_result = run_result
        action_result = self.action_executor.execute_supervisor_result(run_result) if execute else None
        self.last_action_result = action_result
        record = {
            "event": event,
            "run_result": run_result.to_dict(),
            "action_result": action_result.to_dict() if action_result else None,
        }
        self.decision_history.append(record)
        self.decision_history = self.decision_history[-20:]
        return record

    def status(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "world": self.world_model.snapshot(),
            "last_run_result": self.last_run_result.to_dict() if self.last_run_result else None,
            "last_action_result": self.last_action_result.to_dict() if self.last_action_result else None,
            "decision_history_size": len(self.decision_history),
        }
