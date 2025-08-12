#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试Dijkstra算法和变电站图结构
"""

import sys
import os

# 添加utils模块到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))

from utils.graph_utils import SubstationGraph
from utils.path_planner import PathPlanner

def test_graph_structure():
    """测试图结构"""
    print("=" * 60)
    print("🗺️ 测试变电站图结构")
    print("=" * 60)
    
    graph = SubstationGraph()
    
    # 显示所有位置
    print("📍 所有设备位置:")
    for name, coords in graph.get_all_locations().items():
        print(f"  {name}: {coords}")
    
    print(f"\n📊 图统计信息:")
    print(f"  设备点总数: {len(graph.locations)}")
    print(f"  顶点总数: {len(graph.vertices)}")
    
    # 显示连接关系
    print(f"\n🔗 连接关系:")
    for vertex_name, neighbors in graph.adjacency_list.items():
        if neighbors:  # 只显示有连接的
            print(f"  {vertex_name} → {neighbors}")

def test_dijkstra_algorithm():
    """测试Dijkstra算法"""
    print("\n" + "=" * 60)
    print("🧭 测试Dijkstra算法")
    print("=" * 60)
    
    graph = SubstationGraph()
    
    # 测试用例
    test_cases = [
        ("入口点", "35kv配电箱1"),
        ("入口点", "3SVG无功补偿区"),
        ("低压配电室1", "1SVG无功补偿区"),
        ("35kv配电箱3", "变压器区3"),
        ("高压配电区巡检点1", "高压配电区巡检点3"),
    ]
    
    for start, end in test_cases:
        print(f"\n🎯 路径规划: {start} → {end}")
        path = graph.dijkstra(start, end)
        
        if path:
            print(f"  ✅ 路径: {' → '.join(path)}")
            
            # 计算距离
            total_distance = 0
            for i in range(len(path) - 1):
                distance = graph.euclidean_distance(path[i], path[i + 1])
                total_distance += distance
                print(f"     {path[i]} → {path[i + 1]}: {distance:.2f}米")
            
            print(f"  📏 总距离: {total_distance:.2f}米")
        else:
            print(f"  ❌ 无路径")

def test_path_planner():
    """测试路径规划器"""
    print("\n" + "=" * 60)
    print("🛠️ 测试路径规划器")
    print("=" * 60)
    
    planner = PathPlanner()
    
    # 测试单目标路径规划
    print("🎯 单目标路径规划测试:")
    path = planner.plan_path_to_single_target("入口点", "35kv配电箱2")
    print(f"  入口点 → 35kv配电箱2: {path}")
    
    # 测试多目标路径规划
    print("\n🎯 多目标路径规划测试:")
    targets = ["低压配电室1", "低压配电室2", "低压配电室3"]
    path = planner.plan_multi_target_path("入口点", targets)
    print(f"  巡检低压配电室: {path}")
    
    # 测试区域巡检
    print("\n🎯 区域巡检测试:")
    svg_path = planner.plan_area_inspection("入口点", "svg")
    print(f"  SVG区域巡检: {svg_path}")
    
    # 测试完整巡检
    print("\n🎯 完整巡检测试:")
    full_path = planner.plan_full_inspection("入口点")
    print(f"  完整巡检路径 (前10个点): {full_path[:10] if len(full_path) > 10 else full_path}")
    print(f"  完整路径长度: {len(full_path)} 个点")
    
    # 测试模糊匹配
    print("\n🎯 模糊匹配测试:")
    test_inputs = ["svg", "低压配电室", "变压器", "35kv", "无功补偿"]
    for input_name in test_inputs:
        matched = planner.find_matching_waypoint(input_name)
        print(f"  '{input_name}' → {matched}")

def test_graph_connectivity():
    """测试图的连通性"""
    print("\n" + "=" * 60)
    print("🔗 测试图连通性")
    print("=" * 60)
    
    graph = SubstationGraph()
    all_locations = list(graph.locations.keys())
    
    # 检查是否所有点都能从入口点到达
    start_point = "入口点"
    unreachable_points = []
    
    for location in all_locations:
        if location != start_point:
            path = graph.dijkstra(start_point, location)
            if not path:
                unreachable_points.append(location)
    
    if unreachable_points:
        print(f"❌ 从{start_point}无法到达的点: {unreachable_points}")
    else:
        print(f"✅ 所有设备点都可以从{start_point}到达")
    
    # 检查孤立点
    isolated_points = []
    for location in all_locations:
        neighbors = graph.get_neighbors(location)
        if not neighbors:
            isolated_points.append(location)
    
    if isolated_points:
        print(f"❌ 孤立点 (无连接): {isolated_points}")
    else:
        print("✅ 没有孤立点，图结构良好")

def main():
    """主测试函数"""
    print("🧪 变电站巡检系统 - Dijkstra算法测试")
    
    try:
        test_graph_structure()
        test_dijkstra_algorithm()
        test_path_planner()
        test_graph_connectivity()
        
        print("\n" + "=" * 60)
        print("✅ 所有测试完成！")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 