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

# 确保从包目录加载本地 utils 模块
PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from utils import SubstationGraph, LLMClient, PathPlanner, WaypointManager, SegmentScheduler

class SubstationNlpCommanderV2:
    """变电站智能巡检指挥官 V2.0"""
    
    def __init__(self):
        rospy.init_node('substation_nlp_commander_v2', anonymous=True)
        
        # 初始化各个模块
        self.llm_client = LLMClient()
        self.path_planner = PathPlanner()
        self.waypoint_manager = WaypointManager()
        
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
            )
            rospy.loginfo("已启用全局轨迹分段调度模式 (use_global_traj=True)")
        else:
            self.segment_scheduler = None
            rospy.loginfo("使用经典逐航点导航模式 (use_global_traj=False)")

        # 聊天通道：与 Web Dashboard / 其他外部 UI 双向交互
        self._cmd_queue: "queue.Queue[tuple]" = queue.Queue()
        self._chat_out_pub = rospy.Publisher('/chat_out', String, queue_size=20)
        rospy.Subscriber('/chat_in', String, self._chat_in_cb, queue_size=20)
        
        rospy.loginfo("变电站智能巡检指挥官 V2.0 已就绪")

    # ------------------------- 聊天通道 -------------------------
    def _chat_in_cb(self, msg: String):
        """来自 /chat_in (例如 Web Dashboard 用户输入) 的指令。"""
        text = (msg.data or "").strip()
        if not text:
            return
        rospy.loginfo(f"[chat_in] {text}")
        self._cmd_queue.put(('chat', text))

    def _say(self, text: str):
        """同时输出到终端与 /chat_out，供外部 UI 显示。"""
        if text is None:
            return
        print(text)
        try:
            self._chat_out_pub.publish(String(data=str(text)))
        except Exception:
            pass
    
    def on_waypoint_reached(self, waypoint_name: str):
        """航点到达回调"""
        self._say(f"✅ 已到达设备点: {waypoint_name}")
    
    def on_task_completed(self):
        """任务完成回调"""
        self._say("🎉 巡检任务完成！准备接收新指令...")
    
    def handle_navigation_request(self, waypoint_sequence: list, task_description: str, task_type: str = None) -> str:
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
            # 区域巡检 - 使用多目标优化
            rospy.loginfo(f"区域巡检路径规划: {current_pos} -> {task_analysis['targets']}")
            planned_path = self.path_planner.plan_multi_target_path(current_pos, task_analysis["targets"])
            
        elif task_analysis["type"] == "full_inspection":
            # 完整巡检 - 使用预定义的最优路径
            rospy.loginfo(f"完整巡检路径规划从: {current_pos}")
            planned_path = self.path_planner.plan_full_inspection(current_pos)
            
        else:
            # 自定义多点任务保留用户或LLM给出的显式访问顺序
            user_targets = self._expand_and_validate_targets(waypoint_sequence)
            rospy.loginfo(f"自定义路径规划: {current_pos} -> {user_targets}")
            if user_targets:
                planned_path = self.path_planner.plan_ordered_targets_path(current_pos, user_targets)
        
        # 验证路径
        if not planned_path:
            return f"❌ 无法规划到目标位置的路径: {waypoint_sequence}"
        
        # 验证路径连通性
        if not self.path_planner.validate_path(planned_path):
            rospy.logwarn("警告：规划的路径可能不连通，但仍会尝试执行")
        
        # 计算路径距离
        path_distance = self.path_planner.get_path_distance(planned_path)
        
        # 根据目标设备集合生成停留标志：目标设备、起点和终点需要停留，过渡点仅通过。
        stop_flags = self._assign_stop_flags(planned_path, task_analysis["targets"])

        # 启动导航任务（按模式分支）
        if self.segment_scheduler is not None:
            # 解析 (name, (x,y), stop) 元组列表
            wp_tuples = []
            for name, stop in zip(planned_path, stop_flags):
                coords = self.path_planner.graph.get_location_coordinates(name)
                if coords is None:
                    continue
                wp_tuples.append((name, coords, stop))
            if not wp_tuples:
                return f"❌ 无法解析任何航点坐标: {planned_path}"
            result = self.segment_scheduler.start_task(wp_tuples, task_description)
        else:
            result = self.waypoint_manager.start_navigation_task(
                planned_path, task_description, stop_flags)
        
        # 添加路径信息
        result += f"\n📏 路径总距离: {path_distance:.1f}米"
        result += f"\n🛣️ 详细路径: {' → '.join(planned_path)}"
        result += f"\n🛑 停留点: {' → '.join([name for name, stop in zip(planned_path, stop_flags) if stop])}"
        
        return result

    def _assign_stop_flags(self, planned_path: list, target_devices: list) -> list:
        """为全局 Waypoint 序列生成停留标志。"""
        if not planned_path:
            return []

        target_set = set(target_devices)
        last_index = len(planned_path) - 1
        return [
            index == 0 or index == last_index or waypoint_name in target_set
            for index, waypoint_name in enumerate(planned_path)
        ]
    
    def _analyze_waypoint_sequence(self, waypoint_sequence: list, task_description: str, task_type: str = None) -> dict:
        """分析航点序列，确定任务类型和目标"""
        
        # 根据任务描述和航点数量判断任务类型
        description_lower = task_description.lower()
        rospy.loginfo(f"分析任务: 描述='{task_description}', 航点={waypoint_sequence}")
        normalized_task_type = (task_type or "").strip()

        if normalized_task_type in {"single_target", "area_inspection", "full_inspection", "custom_path"}:
            rospy.loginfo(f"采用LLM输出的任务类型: {normalized_task_type}")
            if normalized_task_type == "full_inspection":
                return {"type": "full_inspection", "targets": waypoint_sequence}

            expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
            return {"type": normalized_task_type, "targets": expanded_targets}
        
        # 完整巡检判断
        if any(keyword in description_lower for keyword in ["完整", "全面", "所有", "整个变电站", "巡检一圈"]):
            rospy.loginfo("识别为完整巡检任务")
            return {
                "type": "full_inspection",
                "targets": waypoint_sequence
            }
        
        # 区域巡检判断
        area_keywords = ["svg", "无功补偿", "低压配电室", "高压配电", "变压器", "35kv", "区域", "区"]
        if any(keyword in description_lower for keyword in area_keywords):
            # 扩展区域内的所有设备
            expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
            rospy.loginfo(f"识别为区域巡检任务，扩展目标: {expanded_targets}")
            return {
                "type": "area_inspection", 
                "targets": expanded_targets
            }
        
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
        rospy.loginfo(f"识别为自定义多目标任务，扩展目标: {expanded_targets}")
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
            command, current_pos_info, available_locations
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
        task_type = parsed.get("task_type", "").strip()
        task_description = parsed.get("task_description") or original_command
        target_devices = parsed.get("target_devices", [])
        target_names = self._extract_target_names(target_devices)

        if task_type != "full_inspection" and not target_names:
            return "❌ LLM未返回有效目标设备，请换一种更明确的巡检指令。"

        result = self.handle_navigation_request(
            waypoint_sequence=target_names,
            task_description=task_description,
            task_type=task_type,
        )

        reasoning = parsed.get("reasoning")
        if reasoning:
            result += f"\n🧠 语义解析: {reasoning}"
        return result

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