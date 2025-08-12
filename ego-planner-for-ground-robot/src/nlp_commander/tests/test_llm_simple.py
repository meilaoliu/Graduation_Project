#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的LLM测试，直接测试API调用
"""

import os
import json
from openai import OpenAI

def test_simple_llm_call():
    """简单测试LLM的Function Call"""
    print("🧪 简单LLM Function Call测试")
    print("=" * 50)
    
    # 清理代理环境变量
    for proxy_var in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
        os.environ.pop(proxy_var, None)
    
    # 初始化客户端
    client = OpenAI(
        api_key="sk-905de984fe624c8d91db26b4f081a676",
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    )
    
    # 工具定义
    tools = [
        {
            "type": "function",
            "function": {
                "name": "navigate_robot_with_path",
                "description": "执行机器人巡检任务。根据用户指令规划巡检路径。",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "waypoint_sequence": {
                            "type": "array",
                            "description": "按顺序排列的航点名称列表，例如：['入口点', '插值点1', '低压配电室1', '高压配电区巡检点1']。注意：只需要指定目标设备点，系统会自动使用Dijkstra算法计算最优路径。",
                            "items": {
                                "type": "string"
                            }
                        },
                        "task_description": {
                            "type": "string",
                            "description": "任务描述，说明这次巡检的目的，例如：'前往高压配电区巡检点1进行设备检查'"
                        }
                    },
                    "required": ["waypoint_sequence", "task_description"]
                }
            }
        }
    ]
    
    # 系统提示词
    system_prompt = """你是一个专业的变电站巡检机器人智能助手。你的任务是理解用户的巡检指令，并调用navigate_robot_with_path函数来执行。

🤖 机器人当前位置: 入口点 (9.1, 27.5)

📍 全部可用设备点:
   • 入口点
   • 插值点1
   • 低压配电室1
   • 低压配电室2
   • 低压配电室3
   • 高压配电区巡检点1
   • 高压配电区巡检点2
   • 高压配电区巡检点3
   • 变压器区1
   • 变压器区2
   • 变压器区3
   • 3SVG无功补偿区
   • 2SVG无功补偿区
   • 1SVG无功补偿区
   • 35kv配电箱1
   • 35kv配电箱2
   • 35kv配电箱3

🎯 航点选择策略:
- 单个设备: 去单个设备点，如"去低压配电室1" → waypoint_sequence: ["低压配电室1"]
- 区域巡检: 检查某区域所有设备，如"检查SVG区" → waypoint_sequence: ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"]
- 完整巡检: 完整巡检，如"完整巡检一遍" → waypoint_sequence: [所有设备点列表]
- 自定义路径: 多个具体设备点 → waypoint_sequence: ["设备1", "设备2"]

⚠️ 重要: 必须调用navigate_robot_with_path函数来执行巡检任务！"""

    # 测试指令
    test_commands = [
        "去高压配电区1看一下",
        "完整巡检一遍",
        "检查SVG区域"
    ]
    
    for i, command in enumerate(test_commands, 1):
        print(f"\n🎯 测试 {i}: {command}")
        print("-" * 30)
        
        user_message = f"""用户巡检指令: {command}

🤖 机器人当前状态:
- 位置: 入口点 (9.1, 27.5)

请分析指令并调用相应的导航函数。记住：
1. 只指定最终目标设备点，不要指定中间路径
2. 系统会自动使用Dijkstra算法计算最优路径
3. 必须调用navigate_robot_with_path函数"""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
        
        try:
            response = client.chat.completions.create(
                model="qwen-plus",
                messages=messages,
                tools=tools,
                tool_choice="required",
            )
            
            response_message = response.choices[0].message
            tool_calls = response_message.tool_calls
            
            print(f"📨 原始响应 - tool_calls: {tool_calls}")
            print(f"📨 原始响应 - content: {response_message.content}")
            
            if tool_calls:
                tool_call = tool_calls[0]
                function_name = tool_call.function.name
                function_args = json.loads(tool_call.function.arguments)
                
                print(f"✅ 成功调用函数: {function_name}")
                print(f"📋 参数: {function_args}")
            else:
                print("❌ 没有tool_calls")
                
        except Exception as e:
            print(f"❌ 异常: {e}")

if __name__ == "__main__":
    test_simple_llm_call() 