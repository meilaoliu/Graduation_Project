# -*- coding: utf-8 -*-
"""Data schemas for the autonomous task agent runtime."""

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class RuntimeEvent:
    event_type: str
    message: str = ""
    time: str = ""
    data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, event: Dict[str, Any]) -> "RuntimeEvent":
        data = event.get("data") or {}
        return cls(
            event_type=str(event.get("event_type", "runtime_event")),
            message=str(event.get("message", "")),
            time=str(event.get("time", "")),
            data=dict(data) if isinstance(data, dict) else {},
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class RobotState:
    current_node: str = "入口点"
    current_xy: Optional[Tuple[float, float]] = None
    last_reachable_node: str = "入口点"
    battery_pct: float = 100.0
    estimated_remaining_distance_m: Optional[float] = None
    charging: bool = False
    low_battery: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class TaskStateSnapshot:
    task_id: str = ""
    active: bool = False
    phase: str = "IDLE"
    current_target: Optional[str] = None
    failed_target: Optional[str] = None
    remaining_targets: List[str] = field(default_factory=list)
    failure_counts: Dict[str, int] = field(default_factory=dict)
    time_budget: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ToolSpec:
    name: str
    description: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    read_only: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ToolCall:
    name: str
    arguments: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: Dict[str, Any]) -> "ToolCall":
        return cls(
            name=str(value.get("name", "")),
            arguments=dict(value.get("arguments") or {}),
        )


@dataclass
class ToolResult:
    success: bool
    tool_name: str
    data: Dict[str, Any] = field(default_factory=dict)
    error: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class SupervisorDecision:
    action: str
    target: Optional[str] = None
    remaining_targets: List[str] = field(default_factory=list)
    route_policy: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 0.0
    reasoning: str = ""
    user_message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ValidationResult:
    allowed: bool
    reason: str = ""
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class SupervisorRunResult:
    success: bool
    decision: Optional[SupervisorDecision] = None
    validation: Optional[ValidationResult] = None
    tool_trace: List[Dict[str, Any]] = field(default_factory=list)
    iterations: int = 0
    fallback_used: bool = False
    error: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "success": self.success,
            "decision": self.decision.to_dict() if self.decision else None,
            "validation": self.validation.to_dict() if self.validation else None,
            "tool_trace": self.tool_trace,
            "iterations": self.iterations,
            "fallback_used": self.fallback_used,
            "error": self.error,
        }


@dataclass
class ActionResult:
    success: bool
    action: str
    message: str = ""
    data: Dict[str, Any] = field(default_factory=dict)
    error: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
