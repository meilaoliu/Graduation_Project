# -*- coding: utf-8 -*-
"""Safety checks for task-level agent decisions."""

from typing import Any, Dict, List, Optional, Tuple, Union

from ..runtime_policy import BatteryPolicy
from .schemas import SupervisorDecision, ValidationResult
from .world_model import WorldModel


DecisionLike = Union[SupervisorDecision, Dict[str, Any]]


class SafetyValidator:
    """Validates whether an agent action is allowed before execution."""

    ACTIONS = {
        "continue",
        "retry_failed_target",
        "skip_target",
        "reorder_remaining_targets",
        "return_home",
        "go_charge",
        "resume_remaining_task",
        "finish_task",
        "abort",
    }
    LOW_BATTERY_ACTIONS = {"return_home", "go_charge", "finish_task", "abort", "continue"}
    CHARGING_ACTIONS = {"resume_remaining_task", "finish_task", "abort", "continue"}

    def __init__(
        self,
        path_planner: Any,
        world_model: WorldModel,
        battery_policy: Optional[BatteryPolicy] = None,
        min_confidence: float = 0.35,
        max_retry_per_target: int = 1,
        max_consecutive_failures: int = 3,
    ):
        self.path_planner = path_planner
        self.world_model = world_model
        self.battery_policy = battery_policy or BatteryPolicy()
        self.min_confidence = float(min_confidence)
        self.max_retry_per_target = max(0, int(max_retry_per_target))
        self.max_consecutive_failures = max(1, int(max_consecutive_failures))

    def validate_decision(self, decision: DecisionLike) -> ValidationResult:
        decision_obj = self._to_decision(decision)
        action = decision_obj.action.strip()
        if not action:
            return self._deny("missing_action")
        if action not in self.ACTIONS:
            return self._deny("unknown_action", action=action)
        if decision_obj.confidence < self.min_confidence and action not in {"return_home", "go_charge", "abort"}:
            return self._deny(
                "confidence_too_low",
                action=action,
                confidence=decision_obj.confidence,
                min_confidence=self.min_confidence,
            )

        robot = self.world_model.robot_state
        if robot.low_battery and action not in self.LOW_BATTERY_ACTIONS:
            return self._deny("low_battery_requires_return_or_charge", action=action)
        if robot.charging and action not in self.CHARGING_ACTIONS:
            return self._deny("charging_state_rejects_patrol_action", action=action)

        if action in {"continue", "finish_task", "abort"}:
            return self._allow(action)
        if action in {"return_home", "go_charge"}:
            return self._validate_return_or_charge(action)
        if action == "retry_failed_target":
            return self._validate_retry(decision_obj)
        if action == "skip_target":
            return self._validate_skip(decision_obj)
        if action in {"reorder_remaining_targets", "resume_remaining_task"}:
            return self._validate_remaining_route(action, decision_obj.remaining_targets)
        return self._deny("unhandled_action", action=action)

    def _validate_retry(self, decision: SupervisorDecision) -> ValidationResult:
        target = decision.target or self.world_model.task_state.failed_target
        if not target:
            return self._deny("missing_retry_target")
        if target not in self.path_planner.graph.locations:
            return self._deny("unknown_target", target=target)

        failure_count = self.world_model.task_state.failure_counts.get(target, 0)
        total_failures = sum(self.world_model.task_state.failure_counts.values())
        if failure_count > self.max_retry_per_target:
            return self._deny(
                "retry_limit_exceeded",
                target=target,
                failure_count=failure_count,
                max_retry_per_target=self.max_retry_per_target,
            )
        if total_failures >= self.max_consecutive_failures:
            return self._deny(
                "consecutive_failure_limit_reached",
                total_failures=total_failures,
                max_consecutive_failures=self.max_consecutive_failures,
            )

        start = self._start_node()
        path = self.path_planner.plan_path_to_single_target(start, target)
        route_result = self._validate_path(path, start=start, target=target)
        if not route_result.allowed:
            return route_result
        return self._validate_energy_margin(path, action="retry_failed_target", target=target)

    def _validate_skip(self, decision: SupervisorDecision) -> ValidationResult:
        target = decision.target or self.world_model.task_state.failed_target
        if target and target not in self.path_planner.graph.locations:
            return self._deny("unknown_target", target=target)
        if target and self.world_model.task_state.failure_counts.get(target, 0) == 0:
            return self._deny("skip_requires_recorded_failure", target=target)

        remaining = decision.remaining_targets or [
            item for item in self.world_model.task_state.remaining_targets if item != target
        ]
        unknown = [item for item in remaining if item not in self.path_planner.graph.locations]
        if unknown:
            return self._deny("unknown_remaining_target", targets=unknown)
        if target and target in remaining:
            return self._deny("skipped_target_still_in_remaining", target=target)
        if not remaining:
            return self._allow("skip_target", target=target, remaining_targets=[])
        return self._validate_remaining_route("skip_target", remaining)

    def _validate_remaining_route(self, action: str, remaining_targets: List[str]) -> ValidationResult:
        targets = [target for target in remaining_targets if target]
        if not targets:
            return self._deny("missing_remaining_targets", action=action)
        unknown = [target for target in targets if target not in self.path_planner.graph.locations]
        if unknown:
            return self._deny("unknown_remaining_target", action=action, targets=unknown)

        start = self._start_node()
        path = self.path_planner.plan_ordered_targets_path(start, targets, deduplicate=False)
        route_result = self._validate_path(path, start=start, targets=targets)
        if not route_result.allowed:
            return route_result
        return self._validate_energy_margin(path, action=action, targets=targets)

    def _validate_return_or_charge(self, action: str) -> ValidationResult:
        target = self._charge_point_name()
        start = self._start_node()
        path = self.path_planner.plan_path_to_single_target(start, target)
        route_result = self._validate_path(path, start=start, target=target)
        if not route_result.allowed:
            return route_result
        distance_m = self.path_planner.get_path_distance(path)
        return self._allow(action, start=start, target=target, path=path, distance_m=distance_m)

    def _validate_path(self, path: List[str], **metadata: Any) -> ValidationResult:
        if not path:
            return self._deny("route_unreachable", **metadata)
        if not self.path_planner.validate_path(path):
            return self._deny("route_adjacency_invalid", path=path, **metadata)
        return self._allow("route_valid", path=path, distance_m=self.path_planner.get_path_distance(path), **metadata)

    def _validate_energy_margin(self, path: List[str], **metadata: Any) -> ValidationResult:
        if self.world_model.robot_state.low_battery:
            return self._deny("low_battery_rejects_non_charge_action", **metadata)
        distance_to_next = self.path_planner.get_path_distance(path)
        last_node = path[-1]
        charge_point = self._charge_point_name()
        charge_route = self.path_planner.plan_path_to_single_target(last_node, charge_point)
        if not charge_route:
            return self._deny("charge_route_unreachable_after_action", last_node=last_node, charge_point=charge_point)
        distance_to_charge = self.path_planner.get_path_distance(charge_route)
        decision = self.battery_policy.evaluate(
            self.world_model.robot_state.battery_pct,
            distance_to_next,
            distance_to_charge,
        )
        if decision.should_charge:
            return self._deny(
                "battery_margin_insufficient",
                available_range_m=decision.available_range_m,
                required_range_m=decision.required_range_m,
                margin_m=decision.margin_m,
                **metadata,
            )
        return self._allow(
            "energy_margin_sufficient",
            path=path,
            distance_to_next_m=distance_to_next,
            distance_to_charge_m=distance_to_charge,
            available_range_m=decision.available_range_m,
            required_range_m=decision.required_range_m,
            margin_m=decision.margin_m,
            **metadata,
        )

    def _to_decision(self, value: DecisionLike) -> SupervisorDecision:
        if isinstance(value, SupervisorDecision):
            return value
        remaining = value.get("remaining_targets") or []
        if isinstance(remaining, str):
            remaining = [remaining]
        return SupervisorDecision(
            action=str(value.get("action", "")),
            target=value.get("target"),
            remaining_targets=[str(item) for item in remaining if item],
            route_policy=dict(value.get("route_policy") or {}),
            confidence=float(value.get("confidence", 0.0)),
            reasoning=str(value.get("reasoning", "")),
            user_message=str(value.get("user_message", "")),
        )

    def _start_node(self) -> str:
        robot = self.world_model.robot_state
        return robot.last_reachable_node or robot.current_node or "入口点"

    def _charge_point_name(self) -> str:
        name, _, _ = self.path_planner.graph.get_charge_point()
        return name

    def _allow(self, reason: str, **details: Any) -> ValidationResult:
        return ValidationResult(True, reason=reason, details=details)

    def _deny(self, reason: str, **details: Any) -> ValidationResult:
        return ValidationResult(False, reason=reason, details=details)
