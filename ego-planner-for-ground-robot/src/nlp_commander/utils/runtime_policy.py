# -*- coding: utf-8 -*-
"""Runtime feedback and deterministic safety policies for task execution."""

import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List


@dataclass
class BatteryDecision:
    should_charge: bool
    available_range_m: float
    required_range_m: float
    margin_m: float
    reason: str

    def as_dict(self) -> Dict[str, Any]:
        return {
            "should_charge": self.should_charge,
            "available_range_m": self.available_range_m,
            "required_range_m": self.required_range_m,
            "margin_m": self.margin_m,
            "reason": self.reason,
        }


class BatteryPolicy:
    """Decide charging from deterministic reachability, not from LLM text."""

    def __init__(self, full_range_m: float = 300.0, reserve_m: float = 15.0):
        self.full_range_m = max(float(full_range_m), 1.0)
        self.reserve_m = max(float(reserve_m), 0.0)

    def evaluate(self, battery_pct: float, distance_to_next_m: float, distance_next_to_charge_m: float) -> BatteryDecision:
        pct = min(max(float(battery_pct), 0.0), 100.0)
        available_range = pct / 100.0 * self.full_range_m
        required_range = max(float(distance_to_next_m), 0.0) + max(float(distance_next_to_charge_m), 0.0) + self.reserve_m
        margin = available_range - required_range
        should_charge = margin < 0.0
        reason = (
            "battery_range_insufficient_after_next_target"
            if should_charge else "battery_range_sufficient"
        )
        return BatteryDecision(should_charge, available_range, required_range, margin, reason)


class RuntimeEventLog:
    """Small rolling event log that can be injected into later LLM prompts."""

    def __init__(self, max_events: int = 40):
        self.events: Deque[Dict[str, Any]] = deque(maxlen=max(1, int(max_events)))

    def add(self, event_type: str, message: str, **data: Any) -> Dict[str, Any]:
        event = {
            "time": time.strftime("%H:%M:%S"),
            "event_type": event_type,
            "message": message,
            "data": data,
        }
        self.events.append(event)
        return event

    def clear(self):
        self.events.clear()

    def to_list(self) -> List[Dict[str, Any]]:
        return list(self.events)

    def to_prompt_text(self) -> str:
        if not self.events:
            return "无"
        lines = []
        for event in self.events:
            data = event.get("data") or {}
            detail = ", ".join(f"{key}={value}" for key, value in data.items())
            suffix = f" ({detail})" if detail else ""
            lines.append(f"[{event['time']}] {event['event_type']}: {event['message']}{suffix}")
        return "\n".join(lines)