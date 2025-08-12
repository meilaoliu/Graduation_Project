#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试LLM功能，不依赖ROS
"""

import sys
import os

# 添加utils模块到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))

from utils.llm_utils import LLMClient
from utils.graph_utils import SubstationGraph

def test_llm_function_call():
    """测试LLM的Function Call功能"""
    print("🧪 测试LLM Function Call功能")
    print("=" * 50)
    
    # 初始化客户端
    llm_client = LLMClient()
    graph = SubstationGraph()
    
    # 模拟当前位置
    current_pos_info = "入口点 (9.1, 27.5)"
    available_locations = list(graph.get_all_locations().keys())
    
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
        print("-" * 30)
        
        try:
            response = llm_client.process_inspection_command(
                command=command,
                current_pos_info=current_pos_info,
                available_locations=available_locations
            )
            
            if "error" in response:
                print(f"❌ 错误: {response['error']}")
            else:
                print(f"✅ 成功调用函数: {response.get('function_name')}")
                print(f"📋 参数: {response.get('arguments')}")
                
        except Exception as e:
            print(f"❌ 异常: {e}")

def main():
    """主函数"""
    try:
        test_llm_function_call()
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 