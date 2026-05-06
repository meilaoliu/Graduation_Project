# -*- coding: utf-8 -*-
"""Read-only tool registry for the autonomous task agent runtime."""

from typing import Any, Callable, Dict, Iterable, List, Optional

from ..runtime_policy import BatteryPolicy
from .schemas import ToolResult, ToolSpec
from .world_model import WorldModel


class ToolRegistry:
    """Registers deterministic local tools that an LLM agent can call safely."""

    def __init__(
        self,
        path_planner: Any,
        world_model: WorldModel,
        battery_policy: Optional[BatteryPolicy] = None,
        nominal_speed_mps: float = 0.8,
    ):
        self.path_planner = path_planner
        self.world_model = world_model
        self.battery_policy = battery_policy or BatteryPolicy()
        self.nominal_speed_mps = max(float(nominal_speed_mps), 0.05)
        self._tools: Dict[str, Callable[[Dict[str, Any]], ToolResult]] = {
            "get_robot_state": self._get_robot_state,
            "get_current_task": self._get_current_task,
            "get_runtime_events": self._get_runtime_events,
            "get_known_locations": self._get_known_locations,
            "get_charge_point": self._get_charge_point,
            "get_map_neighbors": self._get_map_neighbors,
            "plan_route": self._plan_route,
            "plan_multi_target_route": self._plan_multi_target_route,
            "estimate_route_distance": self._estimate_route_distance,
            "estimate_energy": self._estimate_energy,
            "estimate_time": self._estimate_time,
        }
        self._specs: Dict[str, ToolSpec] = self._build_specs()

    def list_tools(self) -> List[Dict[str, Any]]:
        return [self._specs[name].to_dict() for name in sorted(self._specs)]

    def execute(self, name: str, arguments: Optional[Dict[str, Any]] = None) -> ToolResult:
        tool = self._tools.get(name)
        if tool is None:
            return ToolResult(False, name, error=f"未知工具: {name}")
        try:
            return tool(dict(arguments or {}))
        except Exception as exc:
            return ToolResult(False, name, error=str(exc))

    def _build_specs(self) -> Dict[str, ToolSpec]:
        return {
            "get_robot_state": ToolSpec(
                name="get_robot_state",
                description="读取机器人当前节点、最近可达节点、电量和低电量状态。",
            ),
            "get_current_task": ToolSpec(
                name="get_current_task",
                description="读取当前任务阶段、当前目标、失败目标、剩余目标和失败计数。",
            ),
            "get_runtime_events": ToolSpec(
                name="get_runtime_events",
                description="读取最近运行事件。参数 limit 控制返回数量。",
                parameters={"limit": "int, optional"},
            ),
            "get_known_locations": ToolSpec(
                name="get_known_locations",
                description="列出拓扑地图中已知位置名称。",
            ),
            "get_charge_point": ToolSpec(
                name="get_charge_point",
                description="读取默认充电点/返航点名称和坐标。",
            ),
            "get_map_neighbors": ToolSpec(
                name="get_map_neighbors",
                description="查询指定拓扑节点的相邻节点。",
                parameters={"node": "str"},
            ),
            "plan_route": ToolSpec(
                name="plan_route",
                description="规划从 start 到 target 的最短拓扑路径。start 省略时使用机器人当前节点。",
                parameters={"start": "str, optional", "target": "str"},
            ),
            "plan_multi_target_route": ToolSpec(
                name="plan_multi_target_route",
                description="规划多目标拓扑路径。preserve_order=true 时按给定顺序访问。",
                parameters={"start": "str, optional", "targets": "list[str]", "preserve_order": "bool, optional"},
            ),
            "estimate_route_distance": ToolSpec(
                name="estimate_route_distance",
                description="估算路径距离。可传 path，或传 target/targets 让工具先规划。",
                parameters={"path": "list[str], optional", "target": "str, optional", "targets": "list[str], optional"},
            ),
            "estimate_energy": ToolSpec(
                name="estimate_energy",
                description="按电量百分比和距离估算剩余里程裕量。",
                parameters={"distance_m": "float", "battery_pct": "float, optional", "extra_distance_to_charge_m": "float, optional"},
            ),
            "estimate_time": ToolSpec(
                name="estimate_time",
                description="按名义速度估算路径耗时。",
                parameters={"distance_m": "float", "speed_mps": "float, optional"},
            ),
        }

    def _get_robot_state(self, arguments: Dict[str, Any]) -> ToolResult:
        return ToolResult(True, "get_robot_state", self.world_model.robot_state.to_dict())

    def _get_current_task(self, arguments: Dict[str, Any]) -> ToolResult:
        return ToolResult(True, "get_current_task", self.world_model.task_state.to_dict())

    def _get_runtime_events(self, arguments: Dict[str, Any]) -> ToolResult:
        limit = int(arguments.get("limit", 10))
        return ToolResult(True, "get_runtime_events", {"events": self.world_model.recent_events(limit)})

    def _get_known_locations(self, arguments: Dict[str, Any]) -> ToolResult:
        locations = sorted(self.path_planner.graph.locations.keys())
        return ToolResult(True, "get_known_locations", {"locations": locations})

    def _get_charge_point(self, arguments: Dict[str, Any]) -> ToolResult:
        name, x, y = self.path_planner.graph.get_charge_point(arguments.get("name"))
        return ToolResult(True, "get_charge_point", {"name": name, "x": x, "y": y})

    def _get_map_neighbors(self, arguments: Dict[str, Any]) -> ToolResult:
        node = str(arguments.get("node") or "")
        if not node:
            return ToolResult(False, "get_map_neighbors", error="缺少 node 参数")
        if node not in self.path_planner.graph.locations:
            return ToolResult(False, "get_map_neighbors", error=f"未知节点: {node}")
        return ToolResult(True, "get_map_neighbors", {"node": node, "neighbors": self.path_planner.graph.get_neighbors(node)})

    def _plan_route(self, arguments: Dict[str, Any]) -> ToolResult:
        start = self._start_node(arguments)
        target = str(arguments.get("target") or "")
        if not target:
            return ToolResult(False, "plan_route", error="缺少 target 参数")
        path = self.path_planner.plan_path_to_single_target(start, target)
        return self._path_result("plan_route", path, start=start, target=target)

    def _plan_multi_target_route(self, arguments: Dict[str, Any]) -> ToolResult:
        start = self._start_node(arguments)
        targets = self._string_list(arguments.get("targets"))
        if not targets:
            return ToolResult(False, "plan_multi_target_route", error="缺少 targets 参数")
        preserve_order = bool(arguments.get("preserve_order", False))
        if preserve_order:
            path = self.path_planner.plan_ordered_targets_path(start, targets, deduplicate=False)
        else:
            path = self.path_planner.plan_multi_target_path(start, targets)
        return self._path_result(
            "plan_multi_target_route",
            path,
            start=start,
            targets=targets,
            preserve_order=preserve_order,
        )

    def _estimate_route_distance(self, arguments: Dict[str, Any]) -> ToolResult:
        path = self._string_list(arguments.get("path"))
        if not path:
            target = arguments.get("target")
            targets = self._string_list(arguments.get("targets"))
            if target:
                planned = self._plan_route({"start": arguments.get("start"), "target": target})
                path = planned.data.get("path", []) if planned.success else []
            elif targets:
                planned = self._plan_multi_target_route(
                    {
                        "start": arguments.get("start"),
                        "targets": targets,
                        "preserve_order": arguments.get("preserve_order", False),
                    }
                )
                path = planned.data.get("path", []) if planned.success else []
        if not path:
            return ToolResult(False, "estimate_route_distance", error="缺少有效 path、target 或 targets")
        distance_m = self.path_planner.get_path_distance(path)
        return ToolResult(True, "estimate_route_distance", {"path": path, "distance_m": distance_m})

    def _estimate_energy(self, arguments: Dict[str, Any]) -> ToolResult:
        distance_m = float(arguments.get("distance_m", 0.0))
        battery_pct = float(arguments.get("battery_pct", self.world_model.robot_state.battery_pct))
        extra_distance = float(arguments.get("extra_distance_to_charge_m", 0.0))
        available_range = arguments.get(
            "available_range_m",
            self.world_model.robot_state.estimated_remaining_distance_m,
        )
        decision = self.battery_policy.evaluate(
            battery_pct,
            distance_m,
            extra_distance,
            available_range_m=available_range,
        )
        return ToolResult(True, "estimate_energy", decision.as_dict())

    def _estimate_time(self, arguments: Dict[str, Any]) -> ToolResult:
        distance_m = max(float(arguments.get("distance_m", 0.0)), 0.0)
        speed_mps = max(float(arguments.get("speed_mps", self.nominal_speed_mps)), 0.05)
        return ToolResult(True, "estimate_time", {"distance_m": distance_m, "speed_mps": speed_mps, "seconds": distance_m / speed_mps})

    def _path_result(self, tool_name: str, path: List[str], **metadata: Any) -> ToolResult:
        if not path:
            return ToolResult(False, tool_name, data=metadata, error="路径规划失败")
        distance_m = self.path_planner.get_path_distance(path)
        valid = self.path_planner.validate_path(path)
        data = dict(metadata)
        data.update({"path": path, "distance_m": distance_m, "valid": valid})
        return ToolResult(valid, tool_name, data=data, error="" if valid else "路径邻接校验失败")

    def _start_node(self, arguments: Dict[str, Any]) -> str:
        start = arguments.get("start") or self.world_model.robot_state.current_node
        return str(start)

    def _string_list(self, value: Any) -> List[str]:
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        if isinstance(value, Iterable):
            return [str(item) for item in value if item]
        return []
