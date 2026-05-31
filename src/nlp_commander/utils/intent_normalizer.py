# -*- coding: utf-8 -*-
"""Normalize LLM JSON intent into an executable inspection task."""

from typing import Any, Dict, Iterable, List, Optional


class IntentNormalizer:
    """Validate and normalize the structured intent returned by the LLM."""

    TASK_TYPES = {
        "single_target",
        "area_inspection",
        "full_inspection",
        "custom_path",
        "return_home",
        "go_charge",
    }
    HOME_TASK_TYPES = {"return_home", "go_home", "return_to_base"}
    CHARGE_TASK_TYPES = {"go_charge", "charge", "return_charge"}

    def __init__(self, available_locations: Iterable[str], home_name: str = "入口点"):
        self.available_locations = set(available_locations)
        self.home_name = home_name

    def normalize(self, parsed: Dict[str, Any], original_command: str = "") -> Dict[str, Any]:
        if not isinstance(parsed, dict):
            return {"success": False, "error": "LLM 输出不是 JSON 对象"}

        stages = self._extract_stage_items(parsed)
        if stages is not None:
            return self._normalize_task_plan(parsed, stages, original_command)

        return self._normalize_single(parsed, original_command)

    def _normalize_single(self, parsed: Dict[str, Any], original_command: str = "") -> Dict[str, Any]:
        if not isinstance(parsed, dict):
            return {"success": False, "error": "LLM 阶段输出不是 JSON 对象"}

        task_type = self._normalize_task_type(parsed.get("task_type"))
        targets = self._normalize_targets(parsed)
        execution = self._normalize_execution(parsed, original_command)
        route_policy = self._normalize_route_policy(parsed, task_type)

        if task_type in self.HOME_TASK_TYPES or task_type in self.CHARGE_TASK_TYPES:
            task_type = "go_charge" if task_type in self.CHARGE_TASK_TYPES else "single_target"
            targets = [self._target_spec(self.home_name, priority=1, stop_required=True, photo_required=False)]

        if task_type != "full_inspection" and not targets:
            return {
                "success": False,
                "error": "LLM 未返回可执行目标；请确认 JSON 中 targets/target_devices 使用地图里的设备名称",
            }

        return {
            "success": True,
            "task_type": task_type,
            "task_description": parsed.get("task_description") or original_command,
            "targets": targets,
            "execution": execution,
            "route_policy": route_policy,
            "reasoning": parsed.get("reasoning", ""),
            "raw": parsed,
        }

    def _extract_stage_items(self, parsed: Dict[str, Any]) -> Optional[List[Dict[str, Any]]]:
        task_plan = parsed.get("task_plan")
        if isinstance(task_plan, dict) and isinstance(task_plan.get("stages"), list):
            return task_plan.get("stages")
        if isinstance(parsed.get("stages"), list):
            return parsed.get("stages")
        return None

    def _normalize_task_plan(
        self,
        parsed: Dict[str, Any],
        stage_items: List[Dict[str, Any]],
        original_command: str,
    ) -> Dict[str, Any]:
        normalized_stages = []
        for index, stage in enumerate(stage_items):
            normalized = self._normalize_single(stage, original_command)
            if not normalized.get("success"):
                return {
                    "success": False,
                    "error": f"第{index + 1}阶段无效: {normalized.get('error', '未知错误')}",
                }
            normalized["stage_id"] = str(stage.get("stage_id") or f"stage_{index + 1}")
            normalized["stage_index"] = index + 1
            normalized_stages.append(normalized)

        if not normalized_stages:
            return {"success": False, "error": "多阶段任务没有可执行阶段"}

        return {
            "success": True,
            "is_multi_stage": True,
            "task_description": parsed.get("task_description") or original_command,
            "reasoning": parsed.get("reasoning", ""),
            "stages": normalized_stages,
            "raw": parsed,
        }

    def _normalize_task_type(self, value: Any) -> str:
        task_type = str(value or "custom_path").strip()
        if task_type in self.HOME_TASK_TYPES or task_type in self.CHARGE_TASK_TYPES:
            return task_type
        return task_type if task_type in self.TASK_TYPES else "custom_path"

    def _normalize_targets(self, parsed: Dict[str, Any]) -> List[Dict[str, Any]]:
        raw_targets = parsed.get("targets")
        if raw_targets is None:
            raw_targets = parsed.get("target_devices", [])
        if not isinstance(raw_targets, list):
            raw_targets = []

        targets = []
        for index, item in enumerate(raw_targets):
            spec = self._normalize_target_item(item, index)
            if spec is not None:
                targets.append(spec)

        targets.sort(key=lambda item: item["priority"])
        return targets

    def _normalize_target_item(self, item: Any, index: int) -> Optional[Dict[str, Any]]:
        if isinstance(item, dict):
            name = item.get("name")
            priority = self._safe_int(item.get("priority"), index + 1)
            stop_required = self._safe_bool(item.get("stop_required"), True)
            photo_required = self._safe_bool(item.get("photo_required"), True)
        else:
            name = str(item) if item is not None else ""
            priority = index + 1
            stop_required = True
            photo_required = True

        if name not in self.available_locations:
            return None

        return self._target_spec(name, priority, stop_required, photo_required)

    @staticmethod
    def _target_spec(name: str, priority: int, stop_required: bool, photo_required: bool) -> Dict[str, Any]:
        return {
            "name": name,
            "priority": priority,
            "stop_required": stop_required,
            "photo_required": photo_required,
        }

    def _normalize_execution(self, parsed: Dict[str, Any], original_command: str = "") -> Dict[str, Any]:
        raw_execution = parsed.get("execution")
        if not isinstance(raw_execution, dict):
            raw_execution = {}

        repeat_count = self._safe_int(raw_execution.get("repeat_count", parsed.get("repeat_count")), 1)
        duration_minutes = self._safe_float(
            raw_execution.get("duration_minutes", parsed.get("duration_minutes"))
        )
        until_time = raw_execution.get("until_time", parsed.get("until_time"))
        until_epoch = self._safe_float(raw_execution.get("until_epoch", parsed.get("until_epoch")))
        preserve_order = self._safe_bool(raw_execution.get("preserve_order"), False)

        if (duration_minutes or until_time or until_epoch) and not self._has_explicit_repeat(original_command):
            repeat_count = 1

        return {
            "repeat_count": max(1, repeat_count),
            "duration_minutes": duration_minutes,
            "until_time": until_time or None,
            "until_epoch": until_epoch,
            "preserve_order": preserve_order,
        }

    @staticmethod
    def _has_explicit_repeat(command: str) -> bool:
        command = (command or "").lower()
        repeat_markers = ("反复", "重复", "循环", "多遍", "几遍", "遍", "几次", "次", "repeat")
        return any(marker in command for marker in repeat_markers)

    def _normalize_route_policy(self, parsed: Dict[str, Any], task_type: str) -> Dict[str, Any]:
        raw_policy = parsed.get("route_policy")
        if not isinstance(raw_policy, dict):
            raw_policy = {}

        default_optimize = task_type in {"area_inspection", "full_inspection"}
        return {
            "use_topology": self._safe_bool(raw_policy.get("use_topology"), True),
            "optimize_order": self._safe_bool(raw_policy.get("optimize_order"), default_optimize),
            "return_home_on_low_battery": self._safe_bool(
                raw_policy.get("return_home_on_low_battery"), True
            ),
        }

    @staticmethod
    def _safe_int(value: Any, default: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _safe_float(value: Any) -> Optional[float]:
        if value in (None, ""):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _safe_bool(value: Any, default: bool) -> bool:
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}