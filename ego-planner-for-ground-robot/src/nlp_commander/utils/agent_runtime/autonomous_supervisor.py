# -*- coding: utf-8 -*-
"""ReAct-style autonomous supervisor for task-level decisions."""

import json
from typing import Any, Callable, Dict, List, Optional

from .safety_validator import SafetyValidator
from .schemas import SupervisorDecision, SupervisorRunResult, ToolResult, ValidationResult
from .tool_registry import ToolRegistry
from .world_model import WorldModel


LlmCallable = Callable[[List[Dict[str, Any]], List[Dict[str, Any]]], Any]


class AutonomousSupervisor:
    """Runs a bounded JSON tool loop and validates the final task action."""

    def __init__(
        self,
        world_model: WorldModel,
        tool_registry: ToolRegistry,
        safety_validator: SafetyValidator,
        llm_callable: Optional[LlmCallable] = None,
        max_tool_iterations: int = 6,
        min_tool_calls_before_final: int = 2,
    ):
        self.world_model = world_model
        self.tool_registry = tool_registry
        self.safety_validator = safety_validator
        self.llm_callable = llm_callable
        self.max_tool_iterations = max(1, int(max_tool_iterations))
        self.min_tool_calls_before_final = max(0, int(min_tool_calls_before_final))

    def run(self, trigger_event: Dict[str, Any], update_world: bool = True) -> SupervisorRunResult:
        if update_world:
            runtime_event = self.world_model.update_from_event(trigger_event)
            event_dict = runtime_event.to_dict()
        else:
            event_dict = dict(trigger_event or {})

        if self.llm_callable is None:
            return self._fallback_result([], 0, "llm_callable_not_configured")

        messages = self._build_messages(event_dict)
        tool_trace: List[Dict[str, Any]] = []
        last_validation: Optional[ValidationResult] = None

        for iteration in range(1, self.max_tool_iterations + 1):
            try:
                response = self._call_llm(messages)
                payload = self._parse_payload(response)
            except Exception as exc:
                return self._fallback_result(tool_trace, iteration - 1, f"llm_or_parse_error: {exc}", last_validation)
            response_type = str(payload.get("type", "")).strip()

            if response_type == "tool_call":
                tool_name = str(payload.get("tool") or payload.get("name") or "").strip()
                arguments = dict(payload.get("arguments") or {})
                result = self.tool_registry.execute(tool_name, arguments)
                tool_trace.append(
                    {
                        "tool": tool_name,
                        "arguments": arguments,
                        "result": result.to_dict(),
                    }
                )
                self._append_tool_result(messages, payload, result)
                continue

            if response_type in {"final_action", "final_decision", "decision"} or "action" in payload:
                if len(tool_trace) < self.min_tool_calls_before_final:
                    self._append_min_tool_error(messages, len(tool_trace))
                    continue
                decision = self._decision_from_payload(payload)
                validation = self.safety_validator.validate_decision(decision)
                last_validation = validation
                if validation.allowed:
                    return SupervisorRunResult(
                        success=True,
                        decision=decision,
                        validation=validation,
                        tool_trace=tool_trace,
                        iterations=iteration,
                    )
                self._append_validation_result(messages, decision, validation)
                continue

            self._append_protocol_error(messages, payload)

        return self._fallback_result(tool_trace, self.max_tool_iterations, "max_iterations_reached", last_validation)

    def _build_messages(self, event_dict: Dict[str, Any]) -> List[Dict[str, Any]]:
        system_prompt = (
            "你是变电站巡检机器人的任务级自主监督器。"
            "你必须先通过工具读取状态、规划路径或估算资源，再给出最终动作。"
            "不要输出速度、角速度、轨迹控制点或底盘控制命令。"
            "progress_anomaly 表示确定性进度监控发现卡住、远离目标或偏离路径；如果证据不足，可以选择 continue。"
            "默认自主决策，只有本地安全校验无法找到安全动作时才保留人工介入空间。"
            "输出必须是 JSON，且只能是 tool_call 或 final_action。"
        )
        protocol = {
            "tool_call": {
                "type": "tool_call",
                "tool": "plan_route",
                "arguments": {"start": "入口点", "target": "低压配电室1"},
            },
            "final_action": {
                "type": "final_action",
                "action": "retry_failed_target",
                "target": "低压配电室1",
                "remaining_targets": ["低压配电室2"],
                "confidence": 0.8,
                "reasoning": "说明决策依据",
                "user_message": "给用户看的简短说明",
            },
        }
        user_payload = {
            "trigger_event": event_dict,
            "world_snapshot": self.world_model.snapshot(),
            "tools": self.tool_registry.list_tools(),
            "protocol": protocol,
        }
        return [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(user_payload, ensure_ascii=False)},
        ]

    def _call_llm(self, messages: List[Dict[str, Any]]) -> Any:
        tools = self.tool_registry.list_tools()
        return self.llm_callable(messages, tools)

    def _parse_payload(self, response: Any) -> Dict[str, Any]:
        if isinstance(response, dict):
            if "parsed" in response and isinstance(response["parsed"], dict):
                return response["parsed"]
            return response
        if isinstance(response, str):
            return json.loads(response)
        raise ValueError(f"Unsupported supervisor response type: {type(response)}")

    def _decision_from_payload(self, payload: Dict[str, Any]) -> SupervisorDecision:
        remaining = payload.get("remaining_targets") or []
        if isinstance(remaining, str):
            remaining = [remaining]
        return SupervisorDecision(
            action=str(payload.get("action", "")),
            target=payload.get("target"),
            remaining_targets=[str(item) for item in remaining if item],
            route_policy=dict(payload.get("route_policy") or {}),
            confidence=float(payload.get("confidence", 0.0)),
            reasoning=str(payload.get("reasoning", "")),
            user_message=str(payload.get("user_message", "")),
        )

    def _append_tool_result(
        self,
        messages: List[Dict[str, Any]],
        payload: Dict[str, Any],
        result: ToolResult,
    ) -> None:
        messages.append({"role": "assistant", "content": json.dumps(payload, ensure_ascii=False)})
        messages.append(
            {
                "role": "tool",
                "name": result.tool_name,
                "content": json.dumps(result.to_dict(), ensure_ascii=False),
            }
        )

    def _append_validation_result(
        self,
        messages: List[Dict[str, Any]],
        decision: SupervisorDecision,
        validation: ValidationResult,
    ) -> None:
        messages.append({"role": "assistant", "content": json.dumps(decision.to_dict(), ensure_ascii=False)})
        messages.append(
            {
                "role": "tool",
                "name": "safety_validator",
                "content": json.dumps(validation.to_dict(), ensure_ascii=False),
            }
        )

    def _append_protocol_error(self, messages: List[Dict[str, Any]], payload: Dict[str, Any]) -> None:
        error_result = {
            "allowed": False,
            "reason": "invalid_supervisor_protocol",
            "details": {"payload": payload},
        }
        messages.append(
            {
                "role": "tool",
                "name": "protocol_checker",
                "content": json.dumps(error_result, ensure_ascii=False),
            }
        )

    def _append_min_tool_error(self, messages: List[Dict[str, Any]], tool_count: int) -> None:
        error_result = {
            "allowed": False,
            "reason": "insufficient_tool_calls_before_final_action",
            "details": {
                "tool_count": tool_count,
                "min_tool_calls_before_final": self.min_tool_calls_before_final,
            },
        }
        messages.append(
            {
                "role": "tool",
                "name": "protocol_checker",
                "content": json.dumps(error_result, ensure_ascii=False),
            }
        )

    def _fallback_result(
        self,
        tool_trace: List[Dict[str, Any]],
        iterations: int,
        error: str,
        last_validation: Optional[ValidationResult] = None,
    ) -> SupervisorRunResult:
        for decision in self._fallback_candidates():
            validation = self.safety_validator.validate_decision(decision)
            if validation.allowed:
                return SupervisorRunResult(
                    success=True,
                    decision=decision,
                    validation=validation,
                    tool_trace=tool_trace,
                    iterations=iterations,
                    fallback_used=True,
                    error=error,
                )
            last_validation = validation
        return SupervisorRunResult(
            success=False,
            validation=last_validation,
            tool_trace=tool_trace,
            iterations=iterations,
            fallback_used=True,
            error=error,
        )

    def _fallback_candidates(self) -> List[SupervisorDecision]:
        task = self.world_model.task_state
        robot = self.world_model.robot_state
        candidates: List[SupervisorDecision] = []

        if robot.low_battery:
            candidates.append(
                SupervisorDecision(
                    action="go_charge",
                    confidence=1.0,
                    reasoning="本地回退策略检测到低电量，优先返航充电。",
                    user_message="电量不足，系统自动返航充电。",
                )
            )

        if task.failed_target:
            failure_count = task.failure_counts.get(task.failed_target, 0)
            if failure_count <= self.safety_validator.max_retry_per_target:
                candidates.append(
                    SupervisorDecision(
                        action="retry_failed_target",
                        target=task.failed_target,
                        confidence=1.0,
                        reasoning="本地回退策略检测到目标首次失败，尝试重试一次。",
                        user_message=f"{task.failed_target} 首次执行失败，系统自动重试。",
                    )
                )
            remaining = [target for target in task.remaining_targets if target != task.failed_target]
            candidates.append(
                SupervisorDecision(
                    action="skip_target",
                    target=task.failed_target,
                    remaining_targets=remaining,
                    confidence=1.0,
                    reasoning="本地回退策略跳过失败目标，继续剩余任务。",
                    user_message=f"{task.failed_target} 多次失败，系统自动跳过并继续。",
                )
            )

        if task.remaining_targets:
            candidates.append(
                SupervisorDecision(
                    action="resume_remaining_task",
                    remaining_targets=task.remaining_targets,
                    confidence=1.0,
                    reasoning="本地回退策略继续执行剩余目标。",
                    user_message="系统将继续执行剩余巡检任务。",
                )
            )

        candidates.extend(
            [
                SupervisorDecision(
                    action="return_home",
                    confidence=1.0,
                    reasoning="本地回退策略返回入口点，保持任务处于安全状态。",
                    user_message="系统自动返回入口点。",
                ),
                SupervisorDecision(
                    action="finish_task",
                    confidence=1.0,
                    reasoning="没有可继续执行的安全动作，结束当前任务。",
                    user_message="当前任务已结束。",
                ),
            ]
        )
        return candidates
