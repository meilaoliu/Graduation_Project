# -*- coding: utf-8 -*-
"""Execute validated task-level agent actions through existing navigation APIs."""

import threading
from typing import Any, Callable, Dict, List, Optional

from .schemas import ActionResult, SupervisorDecision, SupervisorRunResult
from .world_model import WorldModel


class ActionExecutor:
    """Maps validated task actions to the current ROS navigation facade."""

    def __init__(
        self,
        world_model: WorldModel,
        handle_navigation_request: Callable[..., str],
        segment_scheduler: Any = None,
        waypoint_manager: Any = None,
        say: Optional[Callable[[str], None]] = None,
        charge_point_name: str = "入口点",
        scheduler_join_timeout_s: float = 1.5,
    ):
        self.world_model = world_model
        self.handle_navigation_request = handle_navigation_request
        self.segment_scheduler = segment_scheduler
        self.waypoint_manager = waypoint_manager
        self.say = say or (lambda message: None)
        self.charge_point_name = charge_point_name or "入口点"
        self.scheduler_join_timeout_s = max(float(scheduler_join_timeout_s), 0.0)

    def execute_supervisor_result(self, run_result: SupervisorRunResult) -> ActionResult:
        if not run_result.success or run_result.decision is None:
            return ActionResult(
                success=False,
                action="none",
                error=run_result.error or "supervisor_failed_without_decision",
                data=run_result.to_dict(),
            )
        return self.execute(run_result.decision)

    def execute(self, decision: SupervisorDecision) -> ActionResult:
        action = decision.action
        if action == "continue":
            return ActionResult(True, action, decision.user_message or "继续执行当前任务")
        if action == "retry_failed_target":
            return self._retry_failed_target(decision)
        if action == "skip_target":
            return self._skip_target(decision)
        if action in {"reorder_remaining_targets", "resume_remaining_task"}:
            return self._resume_remaining(decision, action)
        if action == "return_home":
            return self._return_home(decision, action)
        if action == "go_charge":
            return self._go_charge(decision)
        if action in {"finish_task", "abort"}:
            return self._finish_or_abort(decision)
        return ActionResult(False, action, error=f"unsupported_action: {action}")

    def _retry_failed_target(self, decision: SupervisorDecision) -> ActionResult:
        target = decision.target or self.world_model.task_state.failed_target
        if not target:
            return ActionResult(False, "retry_failed_target", error="missing_failed_target")
        remaining = [item for item in self.world_model.task_state.remaining_targets if item != target]
        targets = [target] + remaining
        return self._start_targets(
            action="retry_failed_target",
            targets=targets,
            description=decision.user_message or f"自动重试 {target} 并继续剩余任务",
            preserve_order=True,
            photo_required=True,
        )

    def _skip_target(self, decision: SupervisorDecision) -> ActionResult:
        target = decision.target or self.world_model.task_state.failed_target
        remaining = decision.remaining_targets or [
            item for item in self.world_model.task_state.remaining_targets if item != target
        ]
        if not remaining:
            return self._finish_or_abort(
                SupervisorDecision(
                    action="finish_task",
                    confidence=1.0,
                    reasoning="跳过失败目标后没有剩余任务。",
                    user_message=f"{target or '失败目标'} 已跳过；当前没有剩余巡检目标，任务结束。",
                )
            )
        return self._start_targets(
            action="skip_target",
            targets=remaining,
            description=decision.user_message or f"跳过 {target} 并继续剩余任务",
            preserve_order=True,
            photo_required=True,
        )

    def _resume_remaining(self, decision: SupervisorDecision, action: str) -> ActionResult:
        remaining = decision.remaining_targets or self.world_model.task_state.remaining_targets
        if not remaining:
            return ActionResult(False, action, error="missing_remaining_targets")
        return self._start_targets(
            action=action,
            targets=remaining,
            description=decision.user_message or "继续执行剩余巡检任务",
            preserve_order=True,
            photo_required=True,
        )

    def _return_home(self, decision: SupervisorDecision, action: str) -> ActionResult:
        return self._start_targets(
            action=action,
            targets=[self.charge_point_name],
            description=decision.user_message or "自动返航到入口点",
            preserve_order=True,
            photo_required=False,
            task_type="return_home",
            inherit_time_budget=False,
        )

    def _go_charge(self, decision: SupervisorDecision) -> ActionResult:
        if self.segment_scheduler is not None and hasattr(self.segment_scheduler, "start_charge_task"):
            stop_message = self._stop_active_task(
                preserve_pending_mission=True,
                reason="agent_go_charge",
            )
            start_name = self.world_model.robot_state.last_reachable_node or self.world_model.robot_state.current_node
            result = self.segment_scheduler.start_charge_task(start_name=start_name, reason="agent_go_charge")
            success = not str(result).startswith("❌")
            if success:
                self.world_model.stop_task("CHARGING")
            return ActionResult(
                success=success,
                action="go_charge",
                message=result,
                error="" if success else result,
                data={"stop_message": stop_message, "start_name": start_name},
            )
        return self._return_home(decision, "go_charge")

    def _finish_or_abort(self, decision: SupervisorDecision) -> ActionResult:
        stop_message = self._stop_active_task(reason=f"agent_{decision.action}")
        self.world_model.stop_task("FINISHED" if decision.action == "finish_task" else "ABORTED")
        message = decision.user_message or stop_message or "任务已结束"
        return ActionResult(True, decision.action, message=message, data={"stop_message": stop_message})

    def _start_targets(
        self,
        action: str,
        targets: List[str],
        description: str,
        preserve_order: bool,
        photo_required: bool,
        task_type: str = "custom_path",
        inherit_time_budget: bool = True,
    ) -> ActionResult:
        clean_targets = [target for target in targets if target]
        if not clean_targets:
            return ActionResult(False, action, error="empty_target_list")

        previous_failure_counts = dict(self.world_model.task_state.failure_counts)
        stop_message = self._stop_active_task(reason=f"agent_{action}")
        target_specs = [
            {
                "name": target,
                "stop_required": True,
                "photo_required": bool(photo_required),
            }
            for target in clean_targets
        ]
        result = self.handle_navigation_request(
            waypoint_sequence=clean_targets,
            task_description=description,
            task_type=task_type,
            execution_options=self._execution_options(preserve_order, inherit_time_budget),
            target_specs=target_specs,
            route_policy={"use_topology": True, "optimize_order": not preserve_order},
        )
        success = not str(result).startswith("❌")
        if success:
            self.world_model.start_task(
                task_id=f"agent:{action}",
                targets=clean_targets,
                current_target=clean_targets[0],
            )
            self.world_model.task_state.failure_counts.update(previous_failure_counts)
        return ActionResult(
            success=success,
            action=action,
            message=result,
            error="" if success else result,
            data={"targets": clean_targets, "stop_message": stop_message},
        )

    def _execution_options(self, preserve_order: bool, inherit_time_budget: bool = True) -> Dict[str, Any]:
        options = {"repeat_count": 1, "preserve_order": preserve_order}
        if not inherit_time_budget:
            return options
        time_budget = self.world_model.task_state.time_budget or {}
        deadline_epoch = time_budget.get("deadline_epoch") or time_budget.get("until_epoch")
        if deadline_epoch:
            options["until_epoch"] = deadline_epoch
        elif time_budget.get("duration_minutes"):
            options["duration_minutes"] = time_budget.get("duration_minutes")
        return options

    def _stop_active_task(self, preserve_pending_mission: bool = False, reason: str = "external_stop") -> str:
        if self.segment_scheduler is not None and self._scheduler_is_active():
            result = self.segment_scheduler.stop_task(
                preserve_pending_mission=preserve_pending_mission,
                reason=reason,
            )
            self._wait_scheduler_idle()
            return result
        if self.waypoint_manager is not None:
            status = self.waypoint_manager.get_current_status()
            if status.get("task_active") or status.get("remaining_waypoints", 0) > 0:
                return self.waypoint_manager.stop_current_task()
        return ""

    def _scheduler_is_active(self) -> bool:
        try:
            return bool(self.segment_scheduler.is_active())
        except Exception:
            return False

    def _wait_scheduler_idle(self) -> None:
        worker = getattr(self.segment_scheduler, "_worker", None)
        if worker is None or not worker.is_alive():
            return
        if threading.current_thread() is worker:
            return
        worker.join(timeout=self.scheduler_join_timeout_s)
