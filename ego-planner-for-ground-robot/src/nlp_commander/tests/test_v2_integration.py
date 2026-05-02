#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2系统集成测试，模拟完整流程但不依赖ROS
"""

import sys
import os

# 添加 nlp_commander 包根目录到路径
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from utils.llm_utils import LLMClient
from utils.path_planner import PathPlanner

class MockV2Commander:
    """模拟V2指挥官，不依赖ROS"""
    
    def __init__(self):
        self.llm_client = LLMClient()
        self.path_planner = PathPlanner()
        self.current_position_name = "入口点"
    
    def handle_navigation_request(self, waypoint_sequence: list, task_description: str) -> str:
        """处理导航请求"""
        current_pos = self.current_position_name
        
        # 智能分析航点序列，确定最佳路径规划策略
        planned_path = []
        
        if not waypoint_sequence:
            return "❌ 未提供有效的航点序列"
        
        # 分析任务类型和目标
        task_analysis = self._analyze_waypoint_sequence(waypoint_sequence, task_description)
        
        if task_analysis["type"] == "single_target":
            # 单个目标 - 使用Dijkstra算法计算最短路径
            target = task_analysis["targets"][0]
            planned_path = self.path_planner.plan_path_to_single_target(current_pos, target)
            
        elif task_analysis["type"] == "area_inspection":
            # 区域巡检 - 使用多目标优化
            planned_path = self.path_planner.plan_multi_target_path(current_pos, task_analysis["targets"])
            
        elif task_analysis["type"] == "full_inspection":
            # 完整巡检 - 使用预定义的最优路径
            planned_path = self.path_planner.plan_full_inspection(current_pos)
            
        else:
            # 直接使用用户指定的路径，但验证连通性
            user_targets = self._expand_and_validate_targets(waypoint_sequence)
            if user_targets:
                planned_path = self.path_planner.plan_multi_target_path(current_pos, user_targets)
        
        # 验证路径
        if not planned_path:
            return f"❌ 无法规划到目标位置的路径: {waypoint_sequence}"
        
        # 验证路径连通性
        if not self.path_planner.validate_path(planned_path):
            print("⚠️ 警告：规划的路径可能不连通，但仍会尝试执行")
        
        # 计算路径距离
        path_distance = self.path_planner.get_path_distance(planned_path)
        
        # 构建返回信息
        result = f"✅ 任务启动: {task_description}"
        result += f"\n📏 路径总距离: {path_distance:.1f}米"
        result += f"\n🛣️ 详细路径: {' → '.join(planned_path)}"
        result += f"\n📊 任务类型: {task_analysis['type']}"
        
        return result
    
    def _analyze_waypoint_sequence(self, waypoint_sequence: list, task_description: str) -> dict:
        """分析航点序列，确定任务类型和目标"""
        
        # 根据任务描述和航点数量判断任务类型
        description_lower = task_description.lower()
        
        # 完整巡检判断
        if any(keyword in description_lower for keyword in ["完整", "全面", "所有", "整个变电站"]):
            return {
                "type": "full_inspection",
                "targets": waypoint_sequence
            }
        
        # 区域巡检判断
        area_keywords = ["svg", "无功补偿", "低压配电室", "高压配电", "变压器", "35kv", "区域", "区"]
        if any(keyword in description_lower for keyword in area_keywords):
            # 扩展区域内的所有设备
            expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
            return {
                "type": "area_inspection", 
                "targets": expanded_targets
            }
        
        # 单目标判断
        if len(waypoint_sequence) == 1:
            expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
            return {
                "type": "single_target",
                "targets": expanded_targets
            }
        
        # 默认为自定义路径
        expanded_targets = self._expand_and_validate_targets(waypoint_sequence)
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
        current_pos_info = f"{self.current_position_name} (9.1, 27.5)"
        available_locations = list(self.path_planner.graph.get_all_locations().keys())
        
        # 调用LLM处理指令
        llm_response = self.llm_client.process_inspection_command(
            command, current_pos_info, available_locations
        )
        
        # 处理LLM响应
        if "error" in llm_response:
            return llm_response["error"]
        
        if llm_response.get("success") and "parsed" in llm_response:
            parsed = llm_response["parsed"]
            args = {
                "waypoint_sequence": self._extract_target_names(parsed.get("target_devices", [])),
                "task_description": parsed.get("task_description", command),
            }
            return self.handle_navigation_request(
                waypoint_sequence=args.get("waypoint_sequence", []),
                task_description=args.get("task_description", "")
            )
        
        return "❌ LLM响应格式错误"

    def _extract_target_names(self, target_devices: list) -> list:
        """从新版 LLM JSON 中提取目标设备名称。"""
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

def test_v2_integration():
    """测试V2系统集成"""
    print("🧪 V2系统集成测试")
    print("=" * 60)
    
    commander = MockV2Commander()
    
    # 测试用例
    test_commands = [
        "去高压配电区1看一下",
        "完整巡检一遍",
        "检查SVG区域",
        "前往35kV配电箱2",
        "巡检所有变压器"
    ]
    
    for i, command in enumerate(test_commands, 1):
        print(f"\n🎯 测试 {i}: {command}")
        print("-" * 50)
        
        try:
            response = commander.process_command_with_llm(command)
            print(response)
            
        except Exception as e:
            print(f"❌ 异常: {e}")
            import traceback
            traceback.print_exc()

def main():
    """主函数"""
    try:
        test_v2_integration()
        print("\n" + "=" * 60)
        print("✅ V2系统集成测试完成！")
        print("=" * 60)
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 