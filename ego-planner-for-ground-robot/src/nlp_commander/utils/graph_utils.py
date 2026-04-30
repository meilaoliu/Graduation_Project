# -*- coding: utf-8 -*-
"""
变电站图论工具模块

提供 Dijkstra 算法和变电站拓扑图管理。
支持从 OSM 地图文件加载设备坐标，
若 OSM 文件缺失则回退到内置坐标配置。
"""

import math
import os
import heapq
from typing import Dict, List, Tuple, Optional

from .osm_map_loader import load_graph_from_osm

class Vertex:
    """顶点类"""
    def __init__(self, id: int, x: float, y: float, name: str):
        self.id = id
        self.x = x
        self.y = y
        self.name = name
    
    def __str__(self):
        return f"Vertex({self.name}, {self.x}, {self.y})"
    
    def __repr__(self):
        return self.__str__()

class SubstationGraph:
    """变电站拓扑图类"""
    
    def __init__(self, osm_path: Optional[str] = None):
        """
        初始化变电站拓扑图。

        Args:
            osm_path: 可选的 OSM 地图路径；若为 None，则尝试使用默认
                     'maps/substation.osm'，若仍失败则使用内置坐标。
        """
        # 变电站设备坐标
        self.locations: Dict[str, Tuple[float, float]] = {}
        self.osm_edges: List[Tuple[str, str]] = []

        # 优先尝试从 OSM 文件加载
        osm_candidate = osm_path or "maps/substation.osm"
        try:
            self.locations, self.osm_edges = load_graph_from_osm(osm_candidate)
        except Exception:
            # 回退到内置坐标 (兼容旧版本)
            self.locations = {
                "入口点": (9, 27),
                "插值点1": (3, 28),
                "低压配电室1": (-8, 29),
                "低压配电室2": (-26, 29),
                "低压配电室3": (-43, 29),
                "高压配电区巡检点1": (-31, 13),
                "高压配电区巡检点2": (-32, -1),
                "高压配电区巡检点3": (-32, -17),
                "变压器区1": (-28, 14),
                "变压器区2": (-29, 0.91),
                "变压器区3": (-28, -11),
                "插值点2": (-30, -17),
                "3SVG无功补偿区": (-43, -27),
                "2SVG无功补偿区": (-23, -27),
                "1SVG无功补偿区": (-8, -27),
                "插值点3": (-1, -25),
                "35kv配电箱1": (-2, -14),
                "35kv配电箱2": (-2, 1),
                "35kv配电箱3": (-2, 17),
                "插值点4": (6, -25),
                "插值点5": (6, -14),
                "插值点6": (8, -14),
                "插值点7": (8, 4),
            }
        
        # 创建顶点字典
        self.vertices = {}
        for i, (name, coords) in enumerate(self.locations.items()):
            self.vertices[name] = Vertex(i, coords[0], coords[1], name)
        
        # 构建邻接表 - 根据变电站物理布局设计连接关系
        self.adjacency_list = self._build_adjacency_list()
    
    def _build_adjacency_list(self) -> Dict[str, List[str]]:
        """构建邻接表，基于变电站的物理布局"""
        adj_list = {}
        
        # 若完整版 OSM 已包含 way 拓扑，优先使用地图中的连接关系。
        connections = self.osm_edges or [
            # 入口区域连接
            ("入口点", "插值点1"),
            ("插值点1", "低压配电室1"),
            ("插值点1", "35kv配电箱3"),
            
            # 低压配电室区域 (东侧，从北到南)
            ("低压配电室1", "低压配电室2"),
            ("低压配电室2", "低压配电室3"),
            ("低压配电室1", "35kv配电箱3"),
            ("低压配电室2", "变压器区1"),
            ("低压配电室2", "高压配电区巡检点1"),
            ("低压配电室3", "高压配电区巡检点1"),

            # 35kV配电箱区域 (中间，从南到北)
            ("35kv配电箱3", "35kv配电箱2"),
            ("35kv配电箱2", "35kv配电箱1"),
            
            # 高压配电区域 (从高压配电到变压器区域)
            ("高压配电区巡检点1", "高压配电区巡检点2"),
            ("高压配电区巡检点2", "高压配电区巡检点3"),
            ("变压器区1", "变压器区2"),
            ("变压器区2", "变压器区3"),
            # 插值点2区域
            ("插值点2" , "高压配电区巡检点3"),
            ("插值点2" , "变压器区3"),
            ("插值点2" , "3SVG无功补偿区"),
            # SVG无功补偿区域 (西侧，从南到北)
            ("3SVG无功补偿区", "2SVG无功补偿区"),
            ("2SVG无功补偿区", "1SVG无功补偿区"),
            
            # 连接SVG区域到35kV区域
            ("1SVG无功补偿区", "35kv配电箱1"),
            ("2SVG无功补偿区", "35kv配电箱2"),
            ("3SVG无功补偿区", "高压配电区巡检点3"),

            #插值点3区域
            ("插值点3" , "35kv配电箱1"),
            ("插值点3" , "1SVG无功补偿区"),
            ("插值点3" , "插值点4"),
            
            ("插值点4" , "插值点5"),
            ("插值点5" , "插值点6"),
            ("插值点6" , "插值点7"),
            ("插值点7" , "入口点")
 

        ]
        
        # 初始化邻接表
        for name in self.vertices.keys():
            adj_list[name] = []
        
        # 添加连接 (无向图，双向连接)
        for v1, v2 in connections:
            if v1 not in adj_list or v2 not in adj_list:
                continue
            if v2 not in adj_list[v1]:
                adj_list[v1].append(v2)
            if v1 not in adj_list[v2]:
                adj_list[v2].append(v1)
        
        return adj_list
    
    def get_neighbors(self, vertex_name: str) -> List[str]:
        """获取顶点的邻居"""
        return self.adjacency_list.get(vertex_name, [])
    
    def euclidean_distance(self, name1: str, name2: str) -> float:
        """计算两点间的欧氏距离"""
        v1 = self.vertices[name1]
        v2 = self.vertices[name2]
        return math.sqrt((v1.x - v2.x)**2 + (v1.y - v2.y)**2)
    
    def dijkstra(self, start: str, end: str) -> List[str]:
        """
        使用Dijkstra算法计算最短路径
        
        Args:
            start: 起点名称
            end: 终点名称
            
        Returns:
            路径顶点名称列表，如果无路径则返回空列表
        """
        if start not in self.vertices or end not in self.vertices:
            return []
        
        # 距离字典
        distances = {name: float('inf') for name in self.vertices}
        distances[start] = 0
        
        # 前驱节点字典
        predecessors = {}
        
        # 优先队列 [(距离, 顶点名)]
        pq = [(0, start)]
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in visited:
                continue
                
            visited.add(current)
            
            # 找到终点，提前退出
            if current == end:
                break
            
            # 检查所有邻居
            for neighbor in self.get_neighbors(current):
                if neighbor in visited:
                    continue
                
                # 计算新距离
                edge_weight = self.euclidean_distance(current, neighbor)
                new_dist = current_dist + edge_weight
                
                # 如果找到更短路径
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    predecessors[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
        
        # 重建路径
        if end not in predecessors and start != end:
            return []  # 无路径
        
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = predecessors.get(current)
        
        path.reverse()
        return path
    
    def find_nearest_location(self, x: float, y: float) -> str:
        """根据坐标找到最近的设备点"""
        min_distance = float('inf')
        nearest_location = "未知位置"
        
        for location_name, coords in self.locations.items():
            distance = math.sqrt((x - coords[0])**2 + (y - coords[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_location = location_name
        
        # 如果距离最近位置超过5米，认为是在路径中
        if min_distance > 5.0:
            return f"距离{nearest_location}约{min_distance:.1f}米处"
        else:
            return nearest_location
    
    def find_nearest_node_name(self, x: float, y: float) -> str:
        """根据坐标找到最近的节点名称（用于路径规划）"""
        min_distance = float('inf')
        nearest_location = "入口点"  # 默认值
        
        for location_name, coords in self.locations.items():
            distance = math.sqrt((x - coords[0])**2 + (y - coords[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_location = location_name
        
        return nearest_location
    
    def get_location_coordinates(self, name: str) -> Optional[Tuple[float, float]]:
        """获取设备点坐标"""
        return self.locations.get(name)
    
    def get_all_locations(self) -> Dict[str, Tuple[float, float]]:
        """获取所有设备点"""
        return self.locations.copy()
    
    def visualize_graph(self) -> str:
        """可视化图结构（文本形式）"""
        result = "变电站设备连接图:\n"
        for vertex_name, neighbors in self.adjacency_list.items():
            coords = self.locations[vertex_name]
            result += f"{vertex_name} ({coords[0]}, {coords[1]}) -> {neighbors}\n"
        return result 