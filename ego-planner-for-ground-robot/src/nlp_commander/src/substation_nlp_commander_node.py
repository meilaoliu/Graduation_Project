#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import os
import base64
from openai import OpenAI
import json
import math

class SubstationNlpCommander:
    def __init__(self):
        rospy.init_node('substation_nlp_commander', anonymous=True)
        
        # ROS Publisher
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # ROS Subscriber
        self.odom_sub = rospy.Subscriber('/odom_adjust', Odometry, self.odom_callback, queue_size=1)

        # 真实变电站设备坐标 (根据中文说明.md)
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
            "3SVG无功补偿区": (-43, -27),
            "2SVG无功补偿区": (-23, -27),
            "1SVG无功补偿区": (-8, -27),
            "35kv配电箱1": (-2, -14),
            "35kv配电箱2": (-2, 1),
            "35kv配电箱3": (-2, 17),
        }

        # 推荐的完整巡检路径
        self.full_inspection_tour = [
            "入口点", "插值点1", "低压配电室1", "低压配电室2", "低压配电室3",
            "高压配电区巡检点1", "高压配电区巡检点2", "高压配电区巡检点3",
            "3SVG无功补偿区", "2SVG无功补偿区", "1SVG无功补偿区",
            "35kv配电箱1", "35kv配电箱2", "35kv配电箱3", "插值点2", "入口点"
        ]

        # 任务队列和状态
        self.waypoint_queue = []
        self.current_goal = None
        self.goal_tolerance = 2.0  # meters
        self.task_active = False
        
        # 当前位置信息
        self.current_position = None
        self.current_position_name = "未知位置"

        # 变电站俯视图路径
        self.map_image_path = "/home/leo/Graduation_Project/ego-planner-for-ground-robot/src/nlp_commander/变电站俯视标记图.jpg"

        # 清理代理环境变量
        for proxy_var in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
            os.environ.pop(proxy_var, None)

        api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("OPENAI_API_KEY")
        if not api_key:
            rospy.logwarn("未设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY，LLM 调用会失败。")

        # LLM客户端初始化
        self.client = OpenAI(
            api_key=api_key or "EMPTY_API_KEY",
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        # Function Call工具定义
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "navigate_robot_with_path",
                    "description": "执行机器人巡检路径规划和导航。必须调用此函数来实际执行巡检任务。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "waypoint_sequence": {
                                "type": "array",
                                "description": "按顺序排列的航点名称列表，例如：['入口点', '插值点1', '低压配电室1', '高压配电区巡检点1']",
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
        
        self.available_functions = {
            "navigate_robot_with_path": self.handle_navigation_request,
        }

        rospy.loginfo("变电站智能巡检指挥官已就绪，支持多模态路径规划。")

    def find_nearest_location(self, current_pos):
        """根据当前坐标找到最近的已知位置名称"""
        if not current_pos:
            return "未知位置"
        
        min_distance = float('inf')
        nearest_location = "未知位置"
        
        for location_name, coords in self.locations.items():
            distance = math.sqrt((current_pos.x - coords[0])**2 + (current_pos.y - coords[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_location = location_name
        
        # 如果距离最近位置超过5米，认为是在路径中
        if min_distance > 5.0:
            return f"距离{nearest_location}约{min_distance:.1f}米处"
        else:
            return nearest_location

    def encode_image_to_base64(self, image_path):
        """将图像文件编码为base64格式"""
        try:
            with open(image_path, "rb") as image_file:
                return base64.b64encode(image_file.read()).decode('utf-8')
        except Exception as e:
            rospy.logerr(f"图像编码失败: {e}")
            return None

    def find_matching_waypoint(self, requested_name):
        """模糊匹配航点名称，提高容错性"""
        requested_lower = requested_name.lower()
        
        # 直接匹配
        if requested_name in self.locations:
            return requested_name
        
        # 模糊匹配规则
        matching_rules = {
            "svg": ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"],
            "无功补偿": ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"],
            "低压配电室": ["低压配电室1", "低压配电室2", "低压配电室3"],
            "高压配电": ["高压配电区巡检点1", "高压配电区巡检点2", "高压配电区巡检点3"],
            "变压器": ["变压器区1", "变压器区2", "变压器区3"],
            "35kv": ["35kv配电箱1", "35kv配电箱2", "35kv配电箱3"],
        }
        
        # 查找匹配的航点组
        for keyword, waypoints in matching_rules.items():
            if keyword in requested_lower:
                return waypoints  # 返回整个组
        
        # 单独数字匹配
        import re
        number_match = re.search(r'(\d+)', requested_name)
        if number_match:
            number = number_match.group(1)
            for keyword, waypoints in matching_rules.items():
                if keyword in requested_lower:
                    for wp in waypoints:
                        if number in wp:
                            return wp
        
        return None

    def handle_navigation_request(self, waypoint_sequence, task_description):
        """处理导航请求"""
        self.waypoint_queue.clear()
        self.current_goal = None
        self.task_active = True

        if not waypoint_sequence:
            return "未提供有效的航点序列。"

        # 验证并添加有效航点到队列
        valid_waypoints = []
        for waypoint_name in waypoint_sequence:
            if waypoint_name in self.locations:
                valid_waypoints.append((waypoint_name, self.locations[waypoint_name]))
            else:
                # 尝试模糊匹配
                matched = self.find_matching_waypoint(waypoint_name)
                if matched:
                    if isinstance(matched, list):
                        # 返回了一组航点，添加所有
                        for wp in matched:
                            valid_waypoints.append((wp, self.locations[wp]))
                        rospy.loginfo(f"'{waypoint_name}' 匹配到区域: {matched}")
                    else:
                        # 返回了单个航点
                        valid_waypoints.append((matched, self.locations[matched]))
                        rospy.loginfo(f"'{waypoint_name}' 匹配到: {matched}")
                else:
                    rospy.logwarn(f"航点 '{waypoint_name}' 未找到匹配项")

        if not valid_waypoints:
            self.task_active = False
            return "所有指定的航点均无效。"

        self.waypoint_queue = valid_waypoints
        self.process_next_waypoint()
        
        waypoint_names = [wp[0] for wp in valid_waypoints]
        rospy.loginfo(f"开始执行任务: {task_description}")
        return f"任务启动: {task_description}\n巡检路径: {' → '.join(waypoint_names)}"

    def process_next_waypoint(self):
        """处理下一个航点"""
        if not self.waypoint_queue:
            rospy.loginfo("巡检任务完成，所有航点均已到达。")
            self.current_goal = None
            self.task_active = False
            print("\n✅ 巡检任务完成！准备接收新指令...")
            return

        location_name, coords = self.waypoint_queue[0]
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = coords[0]
        pose_msg.pose.position.y = coords[1]
        pose_msg.pose.position.z = 0.75
        pose_msg.pose.orientation.w = 1.0

        self.current_goal = pose_msg
        self.goal_pub.publish(self.current_goal)
        rospy.loginfo(f"🎯 前往: {location_name} 坐标({coords[0]}, {coords[1]})")

    def odom_callback(self, msg):
        """里程计回调，检查是否到达目标"""
        # 更新当前位置信息
        self.current_position = msg.pose.pose.position
        self.current_position_name = self.find_nearest_location(self.current_position)
        
        if self.current_goal is None or not self.task_active:
            return

        current_pos = msg.pose.pose.position
        goal_pos = self.current_goal.pose.position
        
        distance = math.sqrt((current_pos.x - goal_pos.x)**2 + (current_pos.y - goal_pos.y)**2)
        
        if distance < self.goal_tolerance:
            reached_waypoint = self.waypoint_queue[0][0]
            rospy.loginfo(f"✅ 已到达: {reached_waypoint}")
            self.waypoint_queue.pop(0)
            self.process_next_waypoint()

    def process_command_with_llm(self, command):
        """使用多模态大模型处理指令"""
        
        # 编码变电站俯视图
        encoded_image = self.encode_image_to_base64(self.map_image_path)
        if not encoded_image:
            return "❌ 无法加载变电站布局图，请检查图片文件。"

        # 构建多模态消息
        current_pos_info = "未知"
        if self.current_position:
            current_pos_info = f"{self.current_position_name} - 坐标({self.current_position.x:.1f}, {self.current_position.y:.1f})"
        
        system_prompt = f"""你是一个专业的变电站巡检机器人路径规划助手。

🎯 **重要**: 你必须调用 `navigate_robot_with_path` 函数来执行所有巡检任务！

🤖 **机器人当前位置**: {current_pos_info}

🏭 **变电站布局说明（从东到西）：**
- **最右侧(东)**: 400V低压配电室区域 (低压配电室1-3)
- **中间区域**: 从上到下依次为高压配电区、变压器区、35kV配电箱区  
- **最左侧(西)**: SVG/无功补偿区

📍 **可用设备点：**
{json.dumps(list(self.locations.keys()), indent=2, ensure_ascii=False)}

🛣️ **路径规划关键原则：**
1. **从当前位置出发**: 基于机器人实际位置规划最短路径
2. **遵循物理布局**: 必须按照东→西或西→东的顺序通过各区域
3. **必经中转点**: 
   - 从东侧到中间区域: 必须经过低压配电室区域
   - 从中间到西侧: 需要合理的中转点
   - 不能跨越区域直接跳跃
4. **区域访问顺序**: 入口点 → 低压配电室区 → 高压配电区/变压器区/35kV区 → SVG区

📋 **正确路径示例：**
- 去高压配电区1: ["当前位置", "低压配电室1", "低压配电室2", "高压配电区巡检点1"]
- 去SVG区: ["当前位置", "低压配电室1", "低压配电室2", "低压配电室3", "3SVG无功补偿区"]
- 区域巡检: 当用户说"检查SVG无功补偿区"时，应访问该区域的所有设备点

🔧 **Function Call格式：**
```json
{{
  "name": "navigate_robot_with_path",
  "arguments": {{
    "waypoint_sequence": ["航点1", "航点2", "航点3"],
    "task_description": "任务描述"
  }}
}}
```

⚠️ **关键注意事项**: 
1. 绝对不能跨区域直接跳跃
2. 必须遵循物理空间的连续性
3. 区域巡检要包含该区域的所有相关设备
4. 必须调用函数，不要只描述！"""

        messages = [
            {
                "role": "user", 
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{encoded_image}"
                        }
                    },
                    {
                        "type": "text", 
                        "text": f"""用户巡检指令: {command}

🤖 机器人当前状态:
- 位置: {current_pos_info}
- 任务状态: {'执行中' if self.task_active else '空闲'}

请根据变电站布局图和机器人当前位置，制定从当前位置到目标的最优巡检路径。
考虑路径的连续性和效率，避免不必要的绕行。"""
                    }
                ]
            }
        ]

        try:
            response = self.client.chat.completions.create(
                model="qwen-vl-plus",  # 使用视觉模型
                messages=[{"role": "system", "content": system_prompt}] + messages,
                tools=self.tools,
                tool_choice="required",  # 强制使用工具调用
            )
            
            response_message = response.choices[0].message
            tool_calls = response_message.tool_calls

            if tool_calls:
                rospy.loginfo(f"收到Function Call: {len(tool_calls)}个")
                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    rospy.loginfo(f"调用函数: {function_name}")
                    function_to_call = self.available_functions.get(function_name)
                    if function_to_call:
                        function_args = json.loads(tool_call.function.arguments)
                        rospy.loginfo(f"函数参数: {function_args}")
                        return function_to_call(**function_args)
                    else:
                        return f"❌ 未找到函数: {function_name}"
            else:
                rospy.logwarn("大模型没有返回Function Call")
                # 如果没有tool_calls，尝试解析响应内容中的JSON
                content = response_message.content
                if content and "navigate_robot_with_path" in content:
                    try:
                        # 尝试从响应内容中提取JSON
                        import re
                        json_match = re.search(r'\{[\s\S]*\}', content)
                        if json_match:
                            parsed_json = json.loads(json_match.group())
                            if "arguments" in parsed_json:
                                args = parsed_json["arguments"]
                                return self.handle_navigation_request(**args)
                    except Exception as parse_error:
                        rospy.logerr(f"解析响应JSON失败: {parse_error}")
                
                return "❌ 大模型没有正确调用导航函数"

        except Exception as e:
            rospy.logerr(f"大模型调用错误: {e}")
            return f"❌ 大模型调用失败: {str(e)}"

    def run(self):
        rospy.loginfo("🤖 变电站智能巡检系统启动")
        print("=" * 60)
        print("🏭 变电站智能巡检指挥官")
        print("支持功能: 多模态路径规划、智能航点选择、区域巡检")
        print("=" * 60)
        print("📝 指令示例:")
        print("  • '巡检所有低压配电室'")
        print("  • '检查变压器区域'") 
        print("  • '前往35kV配电箱2'")
        print("  • '完整巡检一遍'")
        print("  • '检查SVG无功补偿区'")
        print("=" * 60)
        
        while not rospy.is_shutdown():
            try:
                # 显示当前位置信息
                current_info = "未知位置"
                if self.current_position:
                    current_info = f"{self.current_position_name} ({self.current_position.x:.1f}, {self.current_position.y:.1f})"
                
                print(f"📍 当前位置: {current_info}")
                print("🎯 请输入巡检指令 > ", end="", flush=True)
                command = input()
                if command.strip():
                    print("🔄 正在分析指令并规划路径...")
                    response = self.process_command_with_llm(command.strip())
                    print(f"📋 {response}")
                else:
                    print("⚠️  请输入有效的巡检指令...")
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                rospy.loginfo("🔴 用户终止，关闭巡检系统")
                break
            except EOFError:
                rospy.loginfo("🔴 输入结束，关闭巡检系统")
                break
            except Exception as e:
                rospy.logerr(f"输入处理错误: {e}")
                print("❌ 输入错误，请重试...")

if __name__ == '__main__':
    try:
        commander = SubstationNlpCommander()
        if not rospy.is_shutdown():
            commander.run()
    except rospy.ROSInterruptException:
        pass 