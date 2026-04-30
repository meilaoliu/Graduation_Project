# -*- coding: utf-8 -*-
"""
路径规划器模块
负责处理从当前位置到目标位置的路径规划
"""

import re
from typing import List, Optional, Union, Tuple, Dict
from .graph_utils import SubstationGraph

class PathPlanner:
    """路径规划器类"""
    
    def __init__(self):
        self.graph = SubstationGraph()
        
        # 设备区域分组，用于模糊匹配
        self.device_groups = {
            "svg": ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"],
            "无功补偿": ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"],
            "低压配电室": ["低压配电室1", "低压配电室2", "低压配电室3"],
            "高压配电": ["高压配电区巡检点1", "高压配电区巡检点2", "高压配电区巡检点3"],
            "变压器": ["变压器区1", "变压器区2", "变压器区3"],
            "35kv": ["35kv配电箱1", "35kv配电箱2", "35kv配电箱3"],
        }
        
        # 完整巡检路径
        self.full_inspection_tour = [
            "入口点", "插值点1", "低压配电室1", "低压配电室2", "低压配电室3",
            "高压配电区巡检点1", "高压配电区巡检点2", "高压配电区巡检点3",
            "3SVG无功补偿区", "2SVG无功补偿区", "1SVG无功补偿区",
            "35kv配电箱1", "35kv配电箱2", "35kv配电箱3", "插值点1", "入口点"
        ]
    
    def find_matching_waypoint(self, requested_name: str) -> Union[str, List[str], None]:
        """
        模糊匹配航点名称，提高容错性
        
        Args:
            requested_name: 用户请求的航点名称
            
        Returns:
            匹配的航点名称、航点列表或None
        """
        requested_lower = requested_name.lower().replace(" ", "")
        
        # 直接匹配
        if requested_name in self.graph.locations:
            return requested_name
        
        requested_index = self._extract_requested_index(requested_lower)

        # 带编号的请求优先匹配单个设备，避免“35kV配电箱2”被扩展成整个35kV区域
        if requested_index:
            for keyword, waypoints in self.device_groups.items():
                if keyword in requested_lower:
                    for wp in waypoints:
                        if self._extract_requested_index(wp.lower()) == requested_index:
                            return wp

        # 区域关键字匹配
        for keyword, waypoints in self.device_groups.items():
            if keyword in requested_lower:
                return waypoints  # 返回整个组
        
        return None

    def _extract_requested_index(self, name: str) -> Optional[str]:
        """提取设备编号，排除 35kV 中的电压等级数字。"""
        normalized = name.lower().replace(" ", "")
        normalized = re.sub(r"35\s*k?v", "", normalized)
        numbers = re.findall(r"\d+", normalized)
        return numbers[-1] if numbers else None
    
    def plan_path_to_single_target(self, current_pos: str, target: str) -> List[str]:
        """
        规划到单个目标的路径
        
        Args:
            current_pos: 当前位置名称
            target: 目标位置名称
            
        Returns:
            路径点列表
        """
        # 确保起点和终点都在图中
        if current_pos not in self.graph.locations or target not in self.graph.locations:
            return []
        
        # 使用Dijkstra算法计算最短路径
        path = self.graph.dijkstra(current_pos, target)
        return path
    
    def plan_multi_target_path(self, current_pos: str, targets: List[str]) -> List[str]:
        """
        规划到多个目标的路径。

        对无显式顺序的目标集合，采用论文中描述的基于 Dijkstra 最短路代价的
        贪心策略：每一步选择从当前位置出发代价最小的尚未访问目标，并将相邻
        目标之间的最短路径拼接成全局 Waypoint 序列。
        
        Args:
            current_pos: 当前位置名称
            targets: 目标位置列表
            
        Returns:
            完整路径点列表
        """
        valid_targets = self._deduplicate_valid_targets(targets)
        if not valid_targets:
            return []

        if len(valid_targets) == 1:
            return self.plan_path_to_single_target(current_pos, valid_targets[0])

        remaining = valid_targets[:]
        final_path = []
        current_location = current_pos

        while remaining:
            best_target = None
            best_path = []
            best_distance = float('inf')

            for target in remaining:
                candidate_path = self.plan_path_to_single_target(current_location, target)
                if not candidate_path:
                    continue
                candidate_distance = self.get_path_distance(candidate_path)
                if candidate_distance < best_distance:
                    best_target = target
                    best_path = candidate_path
                    best_distance = candidate_distance

            if best_target is None:
                return []

            self._append_segment(final_path, best_path)
            current_location = best_target
            remaining.remove(best_target)

        return final_path

    def plan_ordered_targets_path(self, current_pos: str, targets: List[str]) -> List[str]:
        """按用户或大模型给定的显式顺序访问目标，并用 Dijkstra 补全中间路径。"""
        ordered_targets = self._deduplicate_valid_targets(targets)
        if not ordered_targets:
            return []

        final_path = []
        current_location = current_pos

        for target in ordered_targets:
            segment = self.plan_path_to_single_target(current_location, target)
            if not segment:
                return []
            self._append_segment(final_path, segment)
            current_location = target

        return final_path

    def _deduplicate_valid_targets(self, targets: List[str]) -> List[str]:
        """过滤未知目标并按首次出现顺序去重。"""
        result = []
        seen = set()
        for target in targets:
            if target in self.graph.locations and target not in seen:
                result.append(target)
                seen.add(target)
        return result

    def _append_segment(self, final_path: List[str], segment: List[str]):
        """拼接两段路径，自动去除首尾重复节点。"""
        if not segment:
            return
        if final_path and final_path[-1] == segment[0]:
            final_path.extend(segment[1:])
        else:
            final_path.extend(segment)
    
    def _create_complete_region_path(self, sorted_targets: List[str], start_index: int) -> List[str]:
        """创建完整的区域访问路径，确保访问所有目标"""
        if not sorted_targets:
            return []
        
        if len(sorted_targets) == 1:
            return sorted_targets
        
        # 确保访问所有目标的策略
        if start_index == 0:
            # 从第一个开始，按顺序访问
            return sorted_targets
        elif start_index == len(sorted_targets) - 1:
            # 从最后一个开始，逆序访问
            return sorted_targets[::-1]
        else:
            # 从中间开始，采用最短遍历策略
            # 比较两种方案：先访问后半部分还是先访问前半部分
            forward_path = sorted_targets[start_index:] + sorted_targets[:start_index]
            backward_path = sorted_targets[start_index::-1] + sorted_targets[-1:start_index:-1]
            
            # 选择路径更短的方案（简单启发式）
            if len(forward_path) <= len(backward_path):
                return forward_path
            else:
                return backward_path
    
    def _group_targets_by_region(self, targets: List[str]) -> Dict[str, List[str]]:
        """按区域对目标进行分组"""
        region_groups = {
            "低压配电室区": [],
            "变压器区": [],
            "高压配电区": [],
            "SVG无功补偿区": [],
            "35kV配电箱区": [],
            "其他": []
        }
        
        for target in targets:
            if "低压配电室" in target:
                region_groups["低压配电室区"].append(target)
            elif "变压器区" in target:
                region_groups["变压器区"].append(target)
            elif "高压配电区" in target:
                region_groups["高压配电区"].append(target)
            elif "SVG" in target or "无功补偿" in target:
                region_groups["SVG无功补偿区"].append(target)
            elif "35kv配电箱" in target:
                region_groups["35kV配电箱区"].append(target)
            else:
                region_groups["其他"].append(target)
        
        # 移除空的区域
        return {k: v for k, v in region_groups.items() if v}
    
    def _plan_region_visit_order(self, current_pos: str, grouped_targets: Dict[str, List[str]]) -> List[str]:
        """规划区域访问顺序，考虑当前位置和物理布局"""
        
        # 预定义的区域访问顺序（从东到西）
        preferred_order = [
            "低压配电室区",
            "35kV配电箱区", 
            "变压器区",
            "高压配电区",
            "SVG无功补偿区",
            "其他"
        ]
        
        # 获取当前位置所在的区域
        current_region = self._get_region_type(current_pos)
        available_regions = [region for region in preferred_order if region in grouped_targets]
        
        # 如果当前位置在某个目标区域中，优先完成当前区域
        # 需要处理SVG区域的特殊命名
        region_mapping = {
            "SVG": "SVG无功补偿区",
            "变压器区": "变压器区",
            "低压配电室": "低压配电室区",
            "高压配电区": "高压配电区",
            "35kv配电箱": "35kV配电箱区"
        }
        
        current_region_name = region_mapping.get(current_region)
        if current_region_name and current_region_name in available_regions:
            # 将当前区域移到最前面
            ordered_regions = [current_region_name]
            ordered_regions.extend([r for r in available_regions if r != current_region_name])
            return ordered_regions
        
        # 否则按照预定义顺序
        return available_regions
    
    def _optimize_region_internal_path(self, entry_point: str, region_targets: List[str]) -> List[str]:
        """优化区域内部的访问路径"""
        if not region_targets:
            return []
        
        if len(region_targets) == 1:
            return self.plan_path_to_single_target(entry_point, region_targets[0])
        
        # 直接使用entry_point作为起点，确保访问所有目标
        return self._solve_region_tsp_optimized(entry_point, region_targets)
    
    def _find_best_region_entry(self, current_pos: str, region_targets: List[str]) -> str:
        """找到进入区域的最佳入口点"""
        if not region_targets:
            return current_pos
        
        # 计算到每个区域目标的距离，选择最近的作为入口
        best_entry = region_targets[0]
        min_distance = float('inf')
        
        for target in region_targets:
            if current_pos in self.graph.locations and target in self.graph.locations:
                distance = self.graph.euclidean_distance(current_pos, target)
                if distance < min_distance:
                    min_distance = distance
                    best_entry = target
        
        return best_entry
    
    def _solve_region_tsp_optimized(self, region_entry: str, targets: List[str]) -> List[str]:
        """解决区域内的旅行商问题（TSP）- 优化版本"""
        if not targets:
            return []
        
        # 对于同一区域内的目标，按照逻辑顺序排序
        sorted_targets = self._sort_targets_by_logical_order(targets)
        
        # 如果入口点就在目标中，从它开始，确保访问所有目标
        if region_entry in sorted_targets:
            # 重新排序，让入口点作为起点，但要访问所有目标
            start_index = sorted_targets.index(region_entry)
            
            # 为了确保访问所有目标，采用智能排序
            if start_index == 0:
                # 如果已经是第一个，按顺序访问
                return sorted_targets
            elif start_index == len(sorted_targets) - 1:
                # 如果是最后一个，逆序访问
                return sorted_targets[::-1]
            else:
                # 如果在中间，需要考虑最优遍历策略
                # 先访问当前位置到序列末尾，再访问开头到当前位置之前
                return sorted_targets[start_index:] + sorted_targets[:start_index]
        else:
            # 入口点不在目标中，需要先到达最近的目标，然后访问所有目标
            closest_target = self._find_closest_target(region_entry, sorted_targets)
            
            # 检查是否需要跨区域路径
            entry_region = self._get_region_type(region_entry)
            target_region = self._get_region_type(closest_target)
            
            if entry_region == target_region:
                # 同一区域内，直接从最近目标开始访问所有目标
                start_index = sorted_targets.index(closest_target)
                if start_index == 0:
                    return sorted_targets
                elif start_index == len(sorted_targets) - 1:
                    return sorted_targets[::-1]
                else:
                    return sorted_targets[start_index:] + sorted_targets[:start_index]
            else:
                # 不同区域，需要完整路径
                path_to_region = self.plan_path_to_single_target(region_entry, closest_target)
                
                # 从最近的目标开始，访问所有目标
                start_index = sorted_targets.index(closest_target)
                if start_index == 0:
                    region_path = sorted_targets
                elif start_index == len(sorted_targets) - 1:
                    region_path = sorted_targets[::-1]
                else:
                    region_path = sorted_targets[start_index:] + sorted_targets[:start_index]
                
                # 合并路径，避免重复入口点
                if path_to_region and region_path:
                    # 如果path_to_region的最后一个点和region_path的第一个点相同，跳过重复
                    if path_to_region[-1] == region_path[0]:
                        return path_to_region + region_path[1:]
                    else:
                        return path_to_region + region_path
                else:
                    return path_to_region or region_path
    
    def _find_closest_target(self, start: str, targets: List[str]) -> str:
        """找到距离起点最近的目标"""
        if not targets:
            return start
        
        closest = targets[0]
        min_distance = float('inf')
        
        for target in targets:
            if start in self.graph.locations and target in self.graph.locations:
                distance = self.graph.euclidean_distance(start, target)
                if distance < min_distance:
                    min_distance = distance
                    closest = target
        
        return closest
    
    def _build_region_continuous_path(self, ordered_targets: List[str]) -> List[str]:
        """在区域内构建连续路径，避免重复访问"""
        if not ordered_targets:
            return []
        
        if len(ordered_targets) == 1:
            return ordered_targets
        
        # 检查是否是同一区域的连续设备
        region_type = self._get_region_type(ordered_targets[0])
        
        if region_type and all(self._get_region_type(t) == region_type for t in ordered_targets):
            # 同一区域内，直接按顺序访问，不需要重复路径
            return ordered_targets
        else:
            # 不同区域，需要计算实际路径
            full_path = []
            for i in range(len(ordered_targets)):
                current = ordered_targets[i-1] if i > 0 else ordered_targets[0]
                target = ordered_targets[i]
                
                if i == 0:
                    full_path.append(target)
                else:
                    segment = self.plan_path_to_single_target(current, target)
                    if segment:
                        full_path.extend(segment[1:])  # 跳过起点避免重复
                    
            return full_path
    
    def _get_region_type(self, target: str) -> str:
        """获取设备所属的区域类型"""
        if "低压配电室" in target:
            return "低压配电室"
        elif "变压器区" in target:
            return "变压器区"
        elif "高压配电区" in target:
            return "高压配电区"
        elif "SVG" in target or "无功补偿" in target:
            return "SVG"
        elif "35kv配电箱" in target:
            return "35kv配电箱"
        else:
            return "其他"
    
    def _sort_targets_by_logical_order(self, targets: List[str]) -> List[str]:
        """按照逻辑顺序对目标进行排序 - 支持多区域混合"""
        
        # 定义各区域内的逻辑顺序
        order_maps = {
            "低压配电室": ["低压配电室1", "低压配电室2", "低压配电室3"],
            "变压器区": ["变压器区1", "变压器区2", "变压器区3"],
            "高压配电区": ["高压配电区巡检点1", "高压配电区巡检点2", "高压配电区巡检点3"],
            "SVG": ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"],
            "35kv配电箱": ["35kv配电箱1", "35kv配电箱2", "35kv配电箱3"]
        }
        
        # 按区域对目标分组并排序
        sorted_result = []
        unmatched_targets = []
        
        # 为每个区域找到对应的目标并按顺序添加
        for prefix, ordered_list in order_maps.items():
            region_targets = [item for item in ordered_list if item in targets]
            if region_targets:
                sorted_result.extend(region_targets)
        
        # 添加不在任何预定义区域中的目标
        for target in targets:
            if target not in sorted_result:
                unmatched_targets.append(target)
        
        return sorted_result + unmatched_targets
    
    def plan_area_inspection(self, current_pos: str, area_keyword: str) -> List[str]:
        """
        规划区域巡检路径
        
        Args:
            current_pos: 当前位置名称
            area_keyword: 区域关键字
            
        Returns:
            区域巡检路径
        """
        area_devices = self.device_groups.get(area_keyword.lower(), [])
        if not area_devices:
            return []
        
        return self.plan_multi_target_path(current_pos, area_devices)
    
    def plan_full_inspection(self, current_pos: str) -> List[str]:
        """
        规划完整变电站巡检路径
        
        Args:
            current_pos: 当前位置名称
            
        Returns:
            完整巡检路径
        """
        # 找到当前位置在完整巡检路径中的最佳插入点
        if current_pos in self.full_inspection_tour:
            # 如果当前位置在路径中，从当前位置开始
            start_index = self.full_inspection_tour.index(current_pos)
            reordered_tour = (self.full_inspection_tour[start_index:] + 
                            self.full_inspection_tour[1:start_index + 1])
            return reordered_tour
        else:
            # 如果当前位置不在路径中，找最近的点开始
            nearest_point = self._find_nearest_tour_point(current_pos)
            if nearest_point:
                path_to_nearest = self.plan_path_to_single_target(current_pos, nearest_point)
                start_index = self.full_inspection_tour.index(nearest_point)
                tour_path = (self.full_inspection_tour[start_index:] + 
                           self.full_inspection_tour[1:start_index + 1])
                
                # 合并路径
                if path_to_nearest and len(path_to_nearest) > 1:
                    return path_to_nearest[:-1] + tour_path
                else:
                    return tour_path
            else:
                return self.full_inspection_tour
    
    def _find_nearest_tour_point(self, current_pos: str) -> Optional[str]:
        """找到完整巡检路径中离当前位置最近的点"""
        if current_pos not in self.graph.locations:
            return self.full_inspection_tour[0]  # 默认返回入口点
        
        min_distance = float('inf')
        nearest_point = None
        
        for point in self.full_inspection_tour:
            if point in self.graph.locations:
                distance = self.graph.euclidean_distance(current_pos, point)
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = point
        
        return nearest_point
    
    def validate_path(self, path: List[str]) -> bool:
        """验证路径是否有效（所有相邻点都有连接）"""
        if len(path) < 2:
            return True
        
        for i in range(len(path) - 1):
            current = path[i]
            next_point = path[i + 1]
            
            if next_point not in self.graph.get_neighbors(current):
                return False
        
        return True
    
    def get_path_distance(self, path: List[str]) -> float:
        """计算路径总距离"""
        if len(path) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(len(path) - 1):
            total_distance += self.graph.euclidean_distance(path[i], path[i + 1])
        
        return total_distance 