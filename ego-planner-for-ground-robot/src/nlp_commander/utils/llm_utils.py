# -*- coding: utf-8 -*-
"""
LLM工具模块
负责与大语言模型的交互
"""

import os
import base64
import json
from openai import OpenAI
from typing import Dict, List, Optional, Any

try:
    import rospy
except ImportError:
    # Mock rospy for testing without ROS
    class MockRospy:
        def logerr(self, msg): print(f"ERROR: {msg}")
        def loginfo(self, msg): print(f"INFO: {msg}")
        def logwarn(self, msg): print(f"WARN: {msg}")
    rospy = MockRospy()

class LLMClient:
    """大语言模型客户端"""
    
    def __init__(self, api_key: str = None, base_url: str = None):
        # 清理代理环境变量
        for proxy_var in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
            os.environ.pop(proxy_var, None)
        
        # 初始化客户端
        self.client = OpenAI(
            api_key=api_key or "sk-905de984fe624c8d91db26b4f081a676",
            base_url=base_url or "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        # 变电站俯视图路径
        self.map_image_path = "/home/leo/Graduation_Project/ego-planner-for-ground-robot/src/nlp_commander/变电站俯视标记图.jpg"
    
    def encode_image_to_base64(self, image_path: str) -> Optional[str]:
        """将图像文件编码为base64格式"""
        try:
            with open(image_path, "rb") as image_file:
                return base64.b64encode(image_file.read()).decode('utf-8')
        except Exception as e:
            rospy.logerr(f"图像编码失败: {e}")
            return None
    
    def create_tools_definition(self) -> List[Dict[str, Any]]:
        """创建Function Call工具定义"""
        return [
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
    
    def create_system_prompt(self, current_pos_info: str, available_locations: List[str]) -> str:
        """创建系统提示词"""
        return f"""你是一个专业的变电站巡检机器人智能助手。你的任务是理解用户的巡检指令，并调用navigate_robot_with_path函数来执行。

🤖 **机器人当前位置**: {current_pos_info}

🏭 **变电站设备分布**:
变电站设备按物理位置分为三个主要区域：
- **东侧区域**: 入口点、插值点1、低压配电室1-3
- **中间区域**: 35kV配电箱1-3、高压配电区巡检点1-3、变压器区1-3  
- **西侧区域**: 1SVG无功补偿区、2SVG无功补偿区、3SVG无功补偿区

📍 **全部可用设备点**: 
{chr(10).join([f"   • {loc}" for loc in available_locations])}

🛠️ **路径规划说明**:
系统使用Dijkstra算法自动计算最优路径，你只需要：
1. 理解用户意图，确定目标设备点
2. 调用navigate_robot_with_path函数，提供航点序列

🎯 **航点选择策略**:
- **单个设备**: 用户想去单个设备点
  例如: "去低压配电室1" → waypoint_sequence: ["低压配电室1"]
  
- **区域巡检**: 用户想检查某个区域的所有设备
  例如: "检查SVG区" → waypoint_sequence: ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"]
  
- **完整巡检**: 用户想完整巡检所有设备
  例如: "完整巡检一遍" → waypoint_sequence: [所有设备点列表]
  
- **自定义路径**: 用户指定了具体设备点
  例如: "先去低压配电室1，再去35kV配电箱2" → waypoint_sequence: ["低压配电室1", "35kv配电箱2"]

🔍 **设备名称匹配**:
支持模糊匹配，例如:
- "高压配电区1" → "高压配电区巡检点1" 
- "SVG区" → ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"]
- "变压器" → ["变压器区1", "变压器区2", "变压器区3"]

⚠️ **重要**: 必须调用navigate_robot_with_path函数来执行巡检任务！"""
    
    def process_inspection_command(self, command: str, current_pos_info: str, available_locations: List[str]) -> Dict[str, Any]:
        """
        处理巡检指令
        
        Args:
            command: 用户指令
            current_pos_info: 当前位置信息
            available_locations: 可用位置列表
            
        Returns:
            LLM响应结果
        """
        # 构建消息 - 纯文本模式，不上传图片
        system_prompt = self.create_system_prompt(current_pos_info, available_locations)
        tools = self.create_tools_definition()
        
        user_message = f"""用户巡检指令: {command}

🤖 机器人当前状态:
- 位置: {current_pos_info}

请分析指令并调用相应的导航函数。记住：
1. 只指定最终目标设备点，不要指定中间路径
2. 系统会自动使用Dijkstra算法计算最优路径
3. 必须调用navigate_robot_with_path函数

📍 可用设备点参考: {', '.join(available_locations)}"""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
        
        try:
            response = self.client.chat.completions.create(
                model="qwen-plus",  # 使用文本模型
                messages=messages,
                tools=tools,
                tool_choice="required",  # 强制使用工具调用
            )
            
            response_message = response.choices[0].message
            tool_calls = response_message.tool_calls
            
            rospy.loginfo(f"LLM响应 - tool_calls: {tool_calls}")
            rospy.loginfo(f"LLM响应 - content: {response_message.content}")
            
            if tool_calls:
                rospy.loginfo(f"收到Function Call: {len(tool_calls)}个")
                tool_call = tool_calls[0]
                function_name = tool_call.function.name
                rospy.loginfo(f"调用函数: {function_name}")
                
                if function_name == "navigate_robot_with_path":
                    function_args = json.loads(tool_call.function.arguments)
                    rospy.loginfo(f"函数参数: {function_args}")
                    return {
                        "success": True,
                        "function_name": function_name,
                        "arguments": function_args
                    }
                else:
                    return {"error": f"❌ 未知函数: {function_name}"}
            else:
                rospy.logwarn("大模型没有返回Function Call")
                # 尝试从响应内容中解析
                content = response_message.content
                if content and "navigate_robot_with_path" in content:
                    try:
                        import re
                        json_match = re.search(r'\{[\s\S]*\}', content)
                        if json_match:
                            parsed_json = json.loads(json_match.group())
                            if "arguments" in parsed_json:
                                rospy.loginfo(f"从内容中解析到参数: {parsed_json['arguments']}")
                                return {
                                    "success": True,
                                    "function_name": "navigate_robot_with_path",
                                    "arguments": parsed_json["arguments"]
                                }
                    except Exception as parse_error:
                        rospy.logerr(f"解析响应JSON失败: {parse_error}")
                
                return {"error": f"❌ 大模型没有正确调用导航函数。响应内容: {content}"}
                
        except Exception as e:
            rospy.logerr(f"大模型调用错误: {e}")
            return {"error": f"❌ 大模型调用失败: {str(e)}"}
    
    def get_available_models(self) -> List[str]:
        """获取可用的模型列表"""
        try:
            models = self.client.models.list()
            return [model.id for model in models.data]
        except Exception as e:
            rospy.logerr(f"获取模型列表失败: {e}")
            return [] 