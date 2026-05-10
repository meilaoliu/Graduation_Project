#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
变电站智能巡检指挥官 V2.0
采用模块化架构，集成Dijkstra算法进行路径规划
"""

import rospy
import sys
import os
import queue
import threading

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# 确保从包目录加载本地 utils 模块
PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from utils import IntentNormalizer, LLMClient, PathPlanner, RuntimeEventLog, TaskAgentRuntime, WaypointManager, SegmentScheduler

class SubstationNlpCommanderV2:
    """变电站智能巡检指挥官 V2.0"""

    STAGE_PRESERVING_ABORT_REASONS = {
        "agent_retry_failed_target",
        "agent_skip_target",
        "agent_reorder_remaining_targets",
        "agent_resume_remaining_task",
        "agent_go_charge",
    }
    
    def __init__(self):
        rospy.init_node('substation_nlp_commander_v2', anonymous=True)
        
        # 初始化各个模块
        self.llm_client = LLMClient()
        self.path_planner = PathPlanner()
        self.intent_normalizer = IntentNormalizer(self.path_planner.graph.get_all_locations().keys())
        self.waypoint_manager = WaypointManager()
        self.runtime_event_log = RuntimeEventLog(max_events=int(rospy.get_param('~runtime_context_events', 40)))
        self._active_task_context = None
        self._last_task_summary = ""
        self._pending_task_stages = []
        self._multi_stage_command = ""
        self._multi_stage_total = 0
        self._cmd_queue: "queue.Queue[tuple]" = queue.Queue()
        self._chat_out_pub = rospy.Publisher('/chat_out', String, queue_size=20)
        rospy.Subscriber('/chat_in', String, self._chat_in_cb, queue_size=20)
        rospy.Service('/reload_map', Trigger, self._handle_reload_map)
        
        # 设置回调函数
        self.waypoint_manager.set_callbacks(
            on_waypoint_reached=self.on_waypoint_reached,
            on_task_completed=self.on_task_completed
        )

        # 全局轨迹分段调度器 (新流程)
        self.use_global_traj = bool(rospy.get_param('~use_global_traj', True))
        if self.use_global_traj and SegmentScheduler is not None:
            self.segment_scheduler = SegmentScheduler(
                graph=self.path_planner.graph,
                get_current_xy=self.waypoint_manager.get_current_coordinates,
                say=lambda m: self._say(m),
                on_event=self._on_runtime_event,
            )
            rospy.loginfo("已启用全局轨迹分段调度模式 (use_global_traj=True)")
        else:
            self.segment_scheduler = None
            rospy.loginfo("使用经典逐航点导航模式 (use_global_traj=False)")

        self.agent_runtime = TaskAgentRuntime(
            path_planner=self.path_planner,
            handle_navigation_request=self.handle_navigation_request,
            segment_scheduler=self.segment_scheduler,
            waypoint_manager=self.waypoint_manager,
            llm_callable=self._agent_llm_call,
            say=lambda m: self._say(m),
            enabled=bool(rospy.get_param('~enable_agent_runtime', True)),
            max_tool_iterations=int(rospy.get_param('~agent_max_tool_iterations', 6)),
            min_tool_calls_before_final=int(rospy.get_param('~agent_min_tool_calls_before_final', 2)),
            min_confidence=float(rospy.get_param('~agent_min_confidence', 0.35)),
            max_retry_per_target=int(rospy.get_param('~agent_max_retry_per_target', 1)),
            max_consecutive_failures=int(rospy.get_param('~agent_max_consecutive_failures', 3)),
            nominal_speed_mps=float(rospy.get_param('~time_budget_nominal_speed', 0.8)),
            battery_full_range_m=float(rospy.get_param('~battery_full_range_m', 300.0)),
            battery_reserve_m=float(rospy.get_param('~battery_reserve_m', 15.0)),
        )

        rospy.loginfo("变电站智能巡检指挥官 V2.0 已就绪")

    # ------------------------- 聊天通道 -------------------------
    def _chat_in_cb(self, msg: String):
        """来自 /chat_in (例如 Web Dashboard 用户输入) 的指令。"""
        text = (msg.data or "").strip()
        if not text:
            return
        rospy.loginfo(f"[chat_in] {text}")
        self._cmd_queue.put(('chat', text))

    def _handle_reload_map(self, _req):
        """ROS service: 重新从 OSM 文件加载地图 (供 dashboard 编辑器保存后调用)。"""
        try:
            used_osm = self.path_planner.graph.reload()
            n_nodes = len(self.path_planner.graph.locations)
            n_edges = len(self.path_planner.graph.osm_edges)
            self.intent_normalizer = IntentNormalizer(
                self.path_planner.graph.get_all_locations().keys()
            )
            if hasattr(self, "agent_runtime") and hasattr(self.agent_runtime, "action_executor"):
                charge_name, _, _ = self.path_planner.graph.get_charge_point()
                self.agent_runtime.action_executor.charge_point_name = charge_name
            msg = (f"地图已重新加载 (来源={'OSM' if used_osm else '内置'}, "
                   f"节点={n_nodes}, 边={n_edges})")
            rospy.loginfo("[reload_map] " + msg)
            self._say("🗺️ " + msg)
            return TriggerResponse(success=True, message=msg)
        except Exception as e:
            rospy.logerr(f"[reload_map] failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def _say(self, text: str):
        """同时输出到终端与 /chat_out，供外部 UI 显示。"""
        if text is None:
            return
        print(text)
        try:
            self._chat_out_pub.publish(String(data=str(text)))
        except Exception:
            pass

    def _on_runtime_event(self, event: dict):
        self.runtime_event_log.add(
            event.get("event_type", "runtime_event"),
            event.get("message", ""),
            **(event.get("data") or {}),
        )
        event_type = event.get("event_type", "runtime_event")
        stage_preserving_abort = event_type == "task_aborted" and self._is_stage_preserving_abort(event)
        if event_type in {"task_completed", "task_aborted", "time_budget_finished"} and not stage_preserving_abort:
            self._last_task_summary = self._build_task_context_summary(event)
            if event_type in {"task_completed", "task_aborted"}:
                self._active_task_context = None
        if event_type == "task_aborted" and not stage_preserving_abort:
            self._pending_task_stages = []
            self._multi_stage_command = ""
            self._multi_stage_total = 0
        if hasattr(self, "agent_runtime"):
            self._cmd_queue.put(('runtime_event', event))

    def _is_stage_preserving_abort(self, event: dict) -> bool:
        data = event.get("data") or {}
        return str(data.get("reason") or "") in self.STAGE_PRESERVING_ABORT_REASONS

    def _build_task_context_summary(self, terminal_event: dict) -> str:
        events = self.runtime_event_log.to_list()
        completed = []
        failures = []
        anomalies = []
        for item in events:
            event_type = item.get("event_type")
            data = item.get("data") or {}
            if event_type == "segment_finished":
                end_name = data.get("end_name") or data.get("target")
                if end_name and end_name not in completed:
                    completed.append(end_name)
            elif event_type == "segment_failed":
                target = data.get("failed_target") or data.get("target") or data.get("current_target")
                failures.append(f"{target or '未知目标'}:{item.get('message', '')}")
            elif event_type == "progress_anomaly":
                target = data.get("failed_target") or data.get("target") or data.get("current_target")
                reason = data.get("reason", "unknown")
                anomalies.append(f"{target or '未知目标'}:{reason}")

        plan = self._active_task_context or {}
        lines = ["上一任务摘要:"]
        if plan.get("command"):
            lines.append(f"- 用户指令: {plan.get('command')}")
        if plan.get("task_type") or plan.get("targets"):
            lines.append(
                f"- 任务类型: {plan.get('task_type', 'unknown')}; 目标: {plan.get('targets', [])}"
            )
        if plan.get("execution"):
            lines.append(f"- 执行约束: {plan.get('execution')}")
        lines.append(f"- 已完成目标: {completed if completed else '无'}")
        if anomalies:
            lines.append(f"- 进度异常: {anomalies[-5:]}")
        if failures:
            lines.append(f"- 失败记录: {failures[-5:]}")
        lines.append(
            f"- 结束状态: {terminal_event.get('event_type')} - {terminal_event.get('message', '')}"
        )
        return "\n".join(lines)

    def _agent_llm_call(self, messages, tools):
        response = self.llm_client.chat_json(
            messages=messages,
            temperature=float(rospy.get_param('~agent_temperature', 0.1)),
            max_tokens=int(rospy.get_param('~agent_max_tokens', 2048)),
        )
        if "error" in response:
            raise RuntimeError(response["error"])
        return response.get("parsed", response)

    def _runtime_context_text(self) -> str:
        parts = []
        if self._last_task_summary:
            parts.append(self._last_task_summary)
        if self.segment_scheduler is not None:
            scheduler_status = self.segment_scheduler.status()
            parts.append(
                "调度器状态: "
                f"active={scheduler_status.get('active')}, "
                f"current_segment={scheduler_status.get('current_segment')}, "
                f"kind={scheduler_status.get('current_segment_kind')}, "
                f"remaining={scheduler_status.get('remaining_segments')}, "
                f"battery={scheduler_status.get('battery_pct'):.1f}%, "
                f"low_battery={scheduler_status.get('low_battery')}, "
                f"returning_to_charge={scheduler_status.get('returning_to_charge')}"
            )
            parts.append("调度器近期反馈:\n" + scheduler_status.get("runtime_context", "无"))
        local_events = self.runtime_event_log.to_prompt_text()
        if local_events != "无":
            parts.append("节点近期反馈:\n" + local_events)
        if hasattr(self, "agent_runtime"):
            parts.append("Agent世界模型:\n" + self.agent_runtime.world_model.to_prompt_context(max_events=8))
        return "\n".join(parts) if parts else "无"
    
    def on_waypoint_reached(self, waypoint_name: str):
        """航点到达回调"""
        self._say(f"✅ 已到达设备点: {waypoint_name}")
    
    def on_task_completed(self):
        """任务完成回调"""
        self._say("🎉 巡检任务完成！准备接收新指令...")
    
    def handle_navigation_request(
        self,
        waypoint_sequence: list,
        task_description: str,
        task_type: str = None,
        execution_options: dict = None,
        target_specs: list = None,
        route_policy: dict = None,
    ) -> str:
        """
        处理导航请求
        
        Args:
            waypoint_sequence: 目标设备序列列表
            task_description: 任务描述
            task_type: LLM输出的任务类型，可选
            
        Returns:
            处理结果信息
        """
        current_pos = self.waypoint_manager.get_current_position_name()
        
        # 处理当前位置，确保能在图中找到对应的节点
        if current_pos == "未知位置":
            current_pos = "入口点"  # 默认起点
        elif current_pos not in self.path_planner.graph.locations:
            # 如果当前位置不是标准节点名称，尝试找到最近的节点
            current_coordinates = self.waypoint_manager.get_current_coordinates()
            if current_coordinates:
                # 根据坐标找最近的节点
                current_pos = self.path_planner.graph.find_nearest_node_name(
                    current_coordinates[0], current_coordinates[1]
                )
                rospy.loginfo(f"当前位置映射到最近节点: {current_pos}")
            else:
                current_pos = "入口点"  # 兜底方案
        
        execution_options = execution_options or {}
        target_specs = target_specs or [
            {"name": name, "stop_required": True, "photo_required": True}
            for name in waypoint_sequence
        ]
        route_policy = route_policy or {}
        if task_type == "go_charge" and self.segment_scheduler is not None:
            charge_name, _, _ = self.path_planner.graph.get_charge_point()
            result = self.segment_scheduler.start_charge_task(start_name=current_pos, reason="user_go_charge")
            planned_path = self.path_planner.plan_path_to_single_target(current_pos, charge_name) or [current_pos, charge_name]
            result += f"\n📏 路径总距离: {self.path_planner.get_path_distance(planned_path):.1f}米"
            result += f"\n🛣️ 详细路径: {' → '.join(planned_path)}"
            result += f"\n🛑 停留点: {charge_name}"
            return result

        # 智能分析航点序列，确定最佳路径规划策略
        planned_path = []
        
        if not waypoint_sequence and task_type != "full_inspection":
            return "❌ 未提供有效的航点序列"
        
        # 分析任务类型和目标
        task_analysis = self._analyze_waypoint_sequence(waypoint_sequence, task_description, task_type)
        rospy.loginfo(f"任务分析结果: {task_analysis}")
        
        if task_analysis["type"] == "single_target":
            # 单个目标 - 使用Dijkstra算法计算最短路径
            target = task_analysis["targets"][0]
            rospy.loginfo(f"单目标路径规划: {current_pos} -> {target}")
            planned_path = self.path_planner.plan_path_to_single_target(current_pos, target)
            
        elif task_analysis["type"] == "area_inspection":
            # 区域巡检：默认优化顺序；LLM 明确要求保序时按给定顺序补全拓扑路径。
            rospy.loginfo(f"区域巡检路径规划: {current_pos} -> {task_analysis['targets']}")
            if route_policy.get("optimize_order", True) and not execution_options.get("preserve_order", False):
                planned_path = self.path_planner.plan_multi_target_path(current_pos, task_analysis["targets"])
            else:
                planned_path = self.path_planner.plan_ordered_targets_path(current_pos, task_analysis["targets"])
            
        elif task_analysis["type"] == "full_inspection":
            # 完整巡检 - 使用预定义的最优路径
            rospy.loginfo(f"完整巡检路径规划从: {current_pos}")
            planned_path = self.path_planner.plan_full_inspection(current_pos)
            
        else:
            # 自定义多点任务保留用户或LLM给出的显式访问顺序
            user_targets = self._expand_and_validate_targets(waypoint_sequence)
            rospy.loginfo(f"自定义路径规划: {current_pos} -> {user_targets}")
            if user_targets:
                planned_path = self.path_planner.plan_ordered_targets_path(
                    current_pos,
                    user_targets,
                    deduplicate=not execution_options.get("preserve_order", False),
                )
        
        # 验证路径
        if not planned_path:
            return f"❌ 无法规划到目标位置的路径: {waypoint_sequence}"
        
        # 验证路径连通性
        if not self.path_planner.validate_path(planned_path):
            rospy.logwarn("警告：规划的路径可能不连通，但仍会尝试执行")
        
        # 根据实际路径生成一次巡检的停留访问序列，途经点不再重复停留/拍照。
        stop_sequence = self._build_stop_sequence(
            planned_path,
            task_analysis["targets"],
            task_analysis["type"],
            target_specs,
            include_start_target=self._should_include_start_target(
                planned_path,
                task_analysis["targets"],
                task_analysis["type"],
                target_specs,
            ),
        )
        include_start_target = bool(
            stop_sequence
            and planned_path
            and stop_sequence[0].get("name") == planned_path[0]
        )
        scheduler_execution_options = dict(execution_options)
        if include_start_target:
            scheduler_execution_options["include_start_target"] = True
        repeat_count = self._resolve_repeat_count(execution_options, planned_path, stop_sequence)
        if repeat_count > 1 and stop_sequence:
            repeated_targets = stop_sequence * repeat_count
            repeated_target_names = [spec["name"] for spec in repeated_targets]
            rospy.loginfo(f"重复巡检扩展为 {repeat_count} 遍: {repeated_target_names}")
            planned_path = self.path_planner.plan_ordered_targets_path(
                current_pos,
                repeated_target_names,
                deduplicate=False,
            )
            if not planned_path:
                return f"❌ 无法规划重复巡检路径: {stop_sequence} x {repeat_count}"
            stop_sequence = repeated_targets

        # 计算路径距离
        path_distance = self.path_planner.get_path_distance(planned_path)

        # 只按 stop_sequence 中的目标顺序停留。重复出现在路径中的已巡检目标会作为途经点通过。
        stop_flags, photo_flags = self._assign_execution_flags(
            planned_path,
            stop_sequence,
            include_start_target=include_start_target,
        )

        # 启动导航任务（按模式分支）
        if self.segment_scheduler is not None:
            # 解析 (name, (x,y), stop) 元组列表
            wp_tuples = []
            for name, stop, photo in zip(planned_path, stop_flags, photo_flags):
                coords = self.path_planner.graph.get_location_coordinates(name)
                if coords is None:
                    continue
                wp_tuples.append((name, coords, stop, photo))
            if not wp_tuples:
                return f"❌ 无法解析任何航点坐标: {planned_path}"
            result = self.segment_scheduler.start_task(wp_tuples, task_description, scheduler_execution_options)
        else:
            result = self.waypoint_manager.start_navigation_task(
                planned_path, task_description, stop_flags)
        
        # 添加路径信息
        result += f"\n📏 路径总距离: {path_distance:.1f}米"
        result += f"\n🛣️ 详细路径: {' → '.join(planned_path)}"
        result += f"\n🛑 停留点: {' → '.join([name for name, stop in zip(planned_path, stop_flags) if stop])}"
        photo_points = [name for name, photo in zip(planned_path, photo_flags) if photo]
        if photo_points:
            result += f"\n📷 拍照点: {' → '.join(photo_points)}"
        if repeat_count > 1:
            result += f"\n🔁 重复巡检: {repeat_count}遍"
        option_text = self._format_execution_options(execution_options)
        if option_text:
            result += f"\n⏱️ 执行约束: {option_text}"
        
        return result

    def _build_stop_sequence(
        self,
        planned_path: list,
        target_devices: list,
        task_type: str,
        target_specs: list = None,
        include_start_target: bool = False,
    ) -> list:
        """生成本轮真正需要停留拍照的目标访问序列。"""
        if not planned_path:
            return []

        spec_by_name = {}
        for spec in target_specs or []:
            name = spec.get("name") if isinstance(spec, dict) else None
            if name in self.path_planner.graph.locations and name not in spec_by_name:
                spec_by_name[name] = {
                    "name": name,
                    "stop_required": bool(spec.get("stop_required", True)),
                    "photo_required": bool(spec.get("photo_required", True)),
                }

        valid_targets = [name for name in target_devices if name in self.path_planner.graph.locations]
        if task_type == "full_inspection" and not valid_targets:
            valid_targets = self._default_inspection_targets()

        if not valid_targets:
            return [{"name": planned_path[-1], "stop_required": True, "photo_required": True}] if len(planned_path) > 1 else []

        # 区域/全站巡检通常由 Dijkstra 贪心决定实际访问顺序，按路径中的首次出现确定停留顺序。
        if task_type in {"area_inspection", "full_inspection"}:
            target_set = set(valid_targets)
            seen = set()
            sequence = []
            for index, name in enumerate(planned_path):
                if index == 0 and not include_start_target:
                    continue
                if name in target_set and name not in seen:
                    sequence.append(spec_by_name.get(name, {"name": name, "stop_required": True, "photo_required": True}))
                    seen.add(name)
            for name in valid_targets:
                if name not in seen:
                    sequence.append(spec_by_name.get(name, {"name": name, "stop_required": True, "photo_required": True}))
            return sequence

        return [
            spec_by_name.get(name, {"name": name, "stop_required": True, "photo_required": True})
            for name in valid_targets
        ]

    def _should_include_start_target(
        self,
        planned_path: list,
        target_devices: list,
        task_type: str,
        target_specs: list = None,
    ) -> bool:
        if not planned_path or not target_devices:
            return False
        if task_type not in {"single_target", "area_inspection", "full_inspection", "custom_path"}:
            return False
        start_name = planned_path[0]
        if start_name not in set(target_devices):
            return False
        for spec in target_specs or []:
            if isinstance(spec, dict) and spec.get("name") == start_name:
                return bool(spec.get("stop_required", True)) and bool(spec.get("photo_required", True))
        return True

    def _assign_execution_flags(
        self,
        planned_path: list,
        target_sequence: list,
        include_start_target: bool = False,
    ) -> tuple:
        """为全局 Waypoint 序列生成停留和拍照标志。"""
        if not planned_path:
            return [], []

        flags = [False] * len(planned_path)
        photo_flags = [False] * len(planned_path)
        next_target_index = 0
        for index, waypoint_name in enumerate(planned_path):
            if index == 0 and not include_start_target:
                continue
            if next_target_index >= len(target_sequence):
                break
            target_spec = target_sequence[next_target_index]
            if waypoint_name == target_spec["name"]:
                flags[index] = bool(target_spec.get("stop_required", True))
                photo_flags[index] = flags[index] and bool(target_spec.get("photo_required", True))
                next_target_index += 1

        if len(planned_path) > 1 and not any(flags):
            flags[-1] = True
            photo_flags[-1] = True
        return flags, photo_flags

    def _default_inspection_targets(self) -> list:
        """默认巡检目标：排除入口点和插值点。"""
        return [
            name for name in self.path_planner.graph.locations
            if name != "入口点" and not name.startswith("插值点")
        ]

    def _resolve_repeat_count(self, execution_options: dict, planned_path: list, stop_sequence: list) -> int:
        explicit_count = self._safe_positive_int(execution_options.get("repeat_count"), default=1)
        if explicit_count > 1:
            return min(explicit_count, self._max_repeat_cycles())
        return 1

    def _max_repeat_cycles(self) -> int:
        return max(1, int(rospy.get_param("~max_repeat_cycles", 20)))

    @staticmethod
    def _safe_positive_int(value, default: int = 1) -> int:
        try:
            parsed = int(value)
            return parsed if parsed > 0 else default
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _safe_float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _format_execution_options(self, execution_options: dict) -> str:
        parts = []
        if self._safe_positive_int(execution_options.get("repeat_count"), default=1) > 1:
            parts.append(f"重复{int(execution_options['repeat_count'])}遍")
        duration_minutes = self._safe_float(execution_options.get("duration_minutes"))
        if duration_minutes and duration_minutes > 0:
            parts.append(f"约{duration_minutes:g}分钟")
        until_text = execution_options.get("until_time")
        if until_text:
            parts.append(f"巡检到{until_text}")
        return "，".join(parts)
    
    def _analyze_waypoint_sequence(self, waypoint_sequence: list, task_description: str, task_type: str = None) -> dict:
        """分析航点序列，确定任务类型和目标"""
        rospy.loginfo(f"分析任务: 描述='{task_description}', 航点={waypoint_sequence}")
        normalized_task_type = (task_type or "").strip()

        if normalized_task_type in {"charge", "go_charge"}:
            charge_name, _, _ = self.path_planner.graph.get_charge_point()
            return {"type": "single_target", "targets": [charge_name]}

        if normalized_task_type in {"return_home", "go_home", "return_to_base"}:
            return {"type": "single_target", "targets": ["入口点"]}

        if normalized_task_type in {"single_target", "area_inspection", "full_inspection", "custom_path"}:
            rospy.loginfo(f"采用LLM输出的任务类型: {normalized_task_type}")
            if normalized_task_type == "full_inspection":
                return {"type": "full_inspection", "targets": waypoint_sequence}

            expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
            return {"type": normalized_task_type, "targets": expanded_targets}
        
        # 单目标判断
        if len(waypoint_sequence) == 1:
            expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
            rospy.loginfo(f"识别为单目标任务，扩展目标: {expanded_targets}")
            return {
                "type": "single_target",
                "targets": expanded_targets
            }
        
        # 多目标自定义路径
        expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
        rospy.loginfo(f"旧接口未提供 task_type，按自定义多目标任务处理: {expanded_targets}")
        return {
            "type": "custom_path",
            "targets": expanded_targets
        }
    
    def _expand_and_validate_targets(self, waypoint_sequence: list) -> list:
        """扩展和验证目标点"""
        expanded_targets = []
        
        for waypoint in waypoint_sequence:
            # 直接匹配
            if waypoint in self.path_planner.graph.locations:
                expanded_targets.append(waypoint)
                continue
            
            # 模糊匹配
            matched = self.path_planner.find_matching_waypoint(waypoint)
            if isinstance(matched, list):
                # 返回了一组设备点
                expanded_targets.extend(matched)
            elif matched:
                # 返回了单个设备点
                expanded_targets.append(matched)
            else:
                # 无匹配，尝试部分匹配
                for location in self.path_planner.graph.locations:
                    if waypoint in location or location in waypoint:
                        expanded_targets.append(location)
                        break
        
        # 去重并保持顺序
        seen = set()
        result = []
        for target in expanded_targets:
            if target not in seen:
                seen.add(target)
                result.append(target)
        
        return result
    
    def process_command_with_llm(self, command: str) -> str:
        """使用LLM处理指令"""
        
        # 获取当前状态信息
        status = self.waypoint_manager.get_current_status()
        current_pos_info = status["current_position"]
        available_locations = list(self.path_planner.graph.get_all_locations().keys())
        
        # 调用LLM处理指令
        llm_response = self.llm_client.process_inspection_command(
            command,
            current_pos_info,
            available_locations,
            runtime_context=self._runtime_context_text(),
        )
        
        # 处理LLM响应
        if "error" in llm_response:
            return llm_response["error"]
        
        if llm_response.get("success") and "parsed" in llm_response:
            return self._handle_structured_llm_result(llm_response["parsed"], command)

        # 兼容旧版 Function Calling 测试桩或历史实现
        if llm_response.get("success") and llm_response.get("function_name") == "navigate_robot_with_path":
            # 执行导航请求
            args = llm_response["arguments"]
            return self.handle_navigation_request(
                waypoint_sequence=args.get("waypoint_sequence", []),
                task_description=args.get("task_description", "")
            )
        
        return "❌ LLM响应格式错误"

    def _handle_structured_llm_result(self, parsed: dict, original_command: str) -> str:
        """将 CoT + JSON 语义解析结果转换为路径规划请求。"""
        normalized = self.intent_normalizer.normalize(parsed, original_command)
        if not normalized.get("success"):
            return f"❌ {normalized.get('error', 'LLM未返回有效结构化意图')}"

        if normalized.get("is_multi_stage"):
            return self._start_multi_stage_task(normalized, original_command)

        return self._start_normalized_stage(normalized, original_command)

    def _start_multi_stage_task(self, normalized: dict, original_command: str) -> str:
        stages = list(normalized.get("stages") or [])
        if not stages:
            return "❌ 多阶段任务没有可执行阶段"

        self._pending_task_stages = stages[1:]
        self._multi_stage_command = original_command
        self._multi_stage_total = len(stages)
        result = self._start_normalized_stage(
            stages[0],
            original_command,
            stage_label=f"第1/{self._multi_stage_total}阶段",
        )
        if result.startswith("❌"):
            self._pending_task_stages = []
            self._multi_stage_command = ""
            self._multi_stage_total = 0
            return result

        reasoning = normalized.get("reasoning")
        if reasoning:
            result += f"\n🧠 多阶段解析: {reasoning}"
        return result

    def _start_normalized_stage(self, normalized: dict, original_command: str, stage_label: str = "") -> str:
        """启动已归一化的单个任务阶段。"""
        target_specs = normalized["targets"]
        self._active_task_context = {
            "command": original_command,
            "stage": stage_label,
            "task_type": normalized.get("task_type"),
            "task_description": normalized.get("task_description"),
            "targets": [spec.get("name") for spec in target_specs if isinstance(spec, dict)],
            "execution": normalized.get("execution") or {},
            "route_policy": normalized.get("route_policy") or {},
        }

        if hasattr(self, "agent_runtime"):
            self.agent_runtime.on_user_command(original_command, normalized)

        task_type = normalized["task_type"]
        task_description = normalized["task_description"]
        target_names = [spec["name"] for spec in target_specs]

        result = self.handle_navigation_request(
            waypoint_sequence=target_names,
            task_description=task_description,
            task_type=task_type,
            execution_options=normalized["execution"],
            target_specs=target_specs,
            route_policy=normalized["route_policy"],
        )

        if stage_label and not result.startswith("❌"):
            result = f"🧩 {stage_label}: {task_description}\n" + result

        reasoning = normalized.get("reasoning")
        if reasoning:
            result += f"\n🧠 语义解析: {reasoning}"
        return result

    def _start_next_pending_stage(self) -> bool:
        if not self._pending_task_stages:
            self._multi_stage_command = ""
            self._multi_stage_total = 0
            return False

        stage = self._pending_task_stages.pop(0)
        stage_index = int(stage.get("stage_index") or (self._multi_stage_total - len(self._pending_task_stages)))
        result = self._start_normalized_stage(
            stage,
            self._multi_stage_command,
            stage_label=f"第{stage_index}/{self._multi_stage_total}阶段",
        )
        self._say(result)
        if result.startswith("❌"):
            self._pending_task_stages = []
            self._multi_stage_command = ""
            self._multi_stage_total = 0
        return True

    def _extract_target_names(self, target_devices: list) -> list:
        """从LLM返回的target_devices字段中提取按优先级排序的设备名称。"""
        normalized_devices = []
        for index, item in enumerate(target_devices):
            if isinstance(item, dict):
                name = item.get("name")
                priority = item.get("priority", index + 1)
            else:
                name = str(item)
                priority = index + 1

            if not name:
                continue

            try:
                priority_value = int(priority)
            except (TypeError, ValueError):
                priority_value = index + 1

            normalized_devices.append((priority_value, name))

        normalized_devices.sort(key=lambda item: item[0])
        return [name for _, name in normalized_devices]

    def show_help(self):
        """显示帮助信息"""
        print("=" * 70)
        print("🏭 变电站智能巡检指挥官 V2.0")
        print("基于图论的Dijkstra算法进行智能路径规划")
        print("=" * 70)
        print("📝 支持的指令类型:")
        print("  🎯 单点导航: '前往低压配电室1' / '去35kV配电箱2'")
        print("  🔍 区域巡检: '检查SVG无功补偿区' / '巡检变压器区域'")
        print("  📋 完整巡检: '完整巡检一遍' / '全面检查设备'")
        print("  🛠️ 任务控制: 'stop' (停止) / 'pause' (暂停) / 'resume' (恢复)")
        print("  📊 状态查询: 'status' (状态) / 'help' (帮助)")
        print("  🗺️ 图论工具: 'graph' (显示图结构)")
        print("=" * 70)
        print("🔧 算法特性:")
        print("  • 基于Dijkstra算法的最短路径规划")
        print("  • 考虑变电站物理布局的图拓扑设计")
        print("  • 自动避免跨区域直接跳跃")
        print("  • 支持多目标贪心路径优化")
        print("=" * 70)
    
    def show_status(self):
        """显示系统状态"""
        status = self.waypoint_manager.get_current_status()
        print("📊 系统状态:")
        print(f"  📍 当前位置: {status['current_position']}")
        print(f"  🎯 任务状态: {'执行中' if status['task_active'] else '空闲'}")
        print(f"  📋 剩余航点: {status['remaining_waypoints']}")
        if status['current_target']:
            print(f"  🎯 当前目标: {status['current_target']}")
        if status['waypoint_queue']:
            print(f"  🛣️ 路径队列: {' → '.join(status['waypoint_queue'])}")
        if hasattr(self, "agent_runtime"):
            agent_status = self.agent_runtime.status()
            print(f"  🧠 Agent Runtime: {'启用' if agent_status.get('enabled') else '关闭'}")
            last_action = agent_status.get("last_action_result") or {}
            if last_action:
                print(f"  🧠 最近动作: {last_action.get('action')} success={last_action.get('success')}")

    def _handle_runtime_event(self, event: dict):
        if not hasattr(self, "agent_runtime"):
            return
        event_type = event.get("event_type", "runtime_event")
        if event_type == "task_completed" and self._pending_task_stages:
            self.agent_runtime.observe_event(event)
            self._start_next_pending_stage()
            return
        if not self.agent_runtime.should_handle_event(event):
            self.agent_runtime.observe_event(event)
            return
        self._say(f"🧠 Agent Runtime 处理事件: {event_type}")
        record = self.agent_runtime.on_runtime_event(event, execute=True)
        if not record:
            return
        run_result = record.get("run_result") or {}
        action_result = record.get("action_result") or {}
        decision = run_result.get("decision") or {}
        validation = run_result.get("validation") or {}
        tool_trace = run_result.get("tool_trace") or []
        if decision:
            self._say(
                "🧠 Agent 决策: "
                f"{decision.get('action')} target={decision.get('target')} "
                f"confidence={decision.get('confidence')} tools={len(tool_trace)}"
            )
            if decision.get("reasoning"):
                self._say(f"🧠 决策依据: {decision.get('reasoning')}")
        if validation and not validation.get("allowed"):
            self._say(f"🛡️ 安全校验拒绝: {validation.get('reason')}")
        if action_result:
            message = action_result.get("message") or action_result.get("error")
            if message:
                self._say(f"🤖 Agent 执行结果: {message}")
    
    def show_graph_info(self):
        """显示图结构信息"""
        print("🗺️ 变电站拓扑图结构:")
        print(self.path_planner.graph.visualize_graph())
    
    def _stdin_reader(self):
        """后台线程：把 stdin 输入塞入命令队列。"""
        while not rospy.is_shutdown():
            try:
                # 提示符仍打印到终端，避免 dashboard-only 用户被打扰
                print("\n🎯 请输入巡检指令 > ", end="", flush=True)
                line = input()
            except (EOFError, KeyboardInterrupt):
                self._cmd_queue.put(('stdin', '__EOF__'))
                return
            self._cmd_queue.put(('stdin', line.strip()))

    def _process_command(self, command: str, source: str = 'stdin'):
        """处理一条指令（来自 stdin 或 /chat_in）。"""
        if not command:
            return
        if source == 'chat':
            self._say(f"📥 [chat] {command}")

        low = command.lower().strip()
        # 中英文同义词映射 (子串匹配，让 "停止巡检"/"暂停一下" 也能命中)
        STOP_KWS    = ('stop', '停止', '停下', '停车', '中止', '取消任务')
        PAUSE_KWS   = ('pause', '暂停')
        RESUME_KWS  = ('resume', '继续', '恢复')
        HELP_KWS    = ('help', '帮助', '?', '？')
        STATUS_KWS  = ('status', '状态', '当前状态')
        GRAPH_KWS   = ('graph', '地图', '查看地图')

        def _hit(kws):
            return any(k in low for k in kws)

        if _hit(HELP_KWS):
            self.show_help()
            return
        if _hit(STATUS_KWS):
            self.show_status()
            if self.segment_scheduler is not None:
                self._say(f"🔧 调度器状态: {self.segment_scheduler.status()}")
            return
        if _hit(GRAPH_KWS):
            self.show_graph_info()
            return
        if _hit(STOP_KWS):
            if self.segment_scheduler is not None and self.segment_scheduler.is_active():
                result = self.segment_scheduler.stop_task()
            else:
                result = self.waypoint_manager.stop_current_task()
            self._say(f"📋 {result}")
            return
        if _hit(PAUSE_KWS):
            result = self.waypoint_manager.pause_current_task()
            self._say(f"📋 {result}")
            return
        if _hit(RESUME_KWS):
            result = self.waypoint_manager.resume_current_task()
            self._say(f"📋 {result}")
            return

        self._say("🔄 正在分析指令并规划路径...")
        try:
            response = self.process_command_with_llm(command)
        except Exception as e:
            response = f"❌ LLM 处理异常: {e}"
            rospy.logerr(response)
        self._say(f"📋 {response}")

    def run(self):
        """运行主循环（消费 stdin 与 /chat_in 两路指令）。"""
        self.show_help()

        stdin_thread = threading.Thread(target=self._stdin_reader, daemon=True)
        stdin_thread.start()

        while not rospy.is_shutdown():
            try:
                source, command = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if command == '__EOF__':
                rospy.loginfo("🔴 stdin 关闭，仅保留 /chat_in 通道")
                continue

            if source == 'runtime_event':
                try:
                    self._handle_runtime_event(command)
                except Exception as e:
                    rospy.logerr(f"Agent Runtime 事件处理失败: {e}")
                    self._say(f"❌ Agent Runtime 事件处理失败: {e}")
                continue

            try:
                self._process_command(command, source=source)
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                rospy.loginfo("🔴 用户终止，关闭巡检系统")
                break
            except Exception as e:
                rospy.logerr(f"指令处理错误: {e}")
                self._say(f"❌ 指令处理出错: {e}")

def main():
    """主函数"""
    try:
        commander = SubstationNlpCommanderV2()
        if not rospy.is_shutdown():
            commander.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"系统启动失败: {e}")

if __name__ == '__main__':
    main()
