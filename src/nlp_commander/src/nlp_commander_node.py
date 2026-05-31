#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import os
from openai import OpenAI
import json
import math

class NlpCommander:
    def __init__(self):
        rospy.init_node('nlp_commander', anonymous=True)
        
        # ROS Publisher
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # ROS Subscriber
        self.odom_sub = rospy.Subscriber('/odom_adjust', Odometry, self.odom_callback, queue_size=1)

        # Predefined locations
        self.locations = {
            "入口点": (-20, 0),
            "配电柜区域": (-15, -9),
            "变压器1": (5, 0),
            "变压器2": (6, 15),
            "开关设备1": (2, -10),
            "开关设备2": (3, 9),
        }
        self.full_inspection_tour = ["入口点", "配电柜区域", "开关设备1", "变压器1", "开关设备2", "变压器2", "入口点"]

        # Waypoint queue and state
        self.waypoint_queue = []
        self.current_goal = None
        self.goal_tolerance = 2.0  # meters
        self.task_active = False  # Flag to control debug output

        # Unset proxy environment variables
        for proxy_var in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
            os.environ.pop(proxy_var, None)

        api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("OPENAI_API_KEY")
        if not api_key:
            rospy.logwarn("未设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY，LLM 调用会失败。")

        # LLM Client Initialization
        self.client = OpenAI(
            api_key=api_key or "EMPTY_API_KEY",
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "navigate_robot",
                    "description": "按顺序导航机器人到一个或多个指定位置。对于'巡检一圈'或类似指令，也使用此函数。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "locations": {
                                "type": "array",
                                "description": "一个或多个目标位置名称的列表。要执行完整巡检，请传入'full_tour'。",
                                "items": {
                                    "type": "string"
                                }
                            }
                        },
                        "required": ["locations"]
                    }
                }
            }
        ]
        
        self.available_functions = {
            "navigate_robot": self.handle_navigation_request,
        }

        rospy.loginfo("NLP Commander is ready for multi-waypoint navigation.")

    def handle_navigation_request(self, locations):
        """Receives a list of location names and starts the navigation sequence."""
        self.waypoint_queue.clear()
        self.current_goal = None
        self.task_active = True  # Start new task
        
        if not locations:
            return "指令中未提供有效位置。"

        if len(locations) == 1 and locations[0] == 'full_tour':
            location_names = self.full_inspection_tour
            rospy.loginfo("Full inspection tour requested.")
        else:
            location_names = locations

        for name in location_names:
            if name in self.locations:
                self.waypoint_queue.append((name, self.locations[name]))
            else:
                rospy.logwarn(f"Location '{name}' not found in predefined locations.")
        
        if not self.waypoint_queue:
            self.task_active = False
            return "所有指定的位置均无效。"

        self.process_next_waypoint()
        return f"收到导航任务，将按顺序前往: {', '.join([loc[0] for loc in self.waypoint_queue])}"
    
    def process_next_waypoint(self):
        """Pops the next waypoint from the queue and publishes it."""
        if not self.waypoint_queue:
            rospy.loginfo("任务队列已完成，所有航点均已到达。")
            self.current_goal = None
            self.task_active = False  # Task completed
            print("\n准备接收新指令...")
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
        rospy.loginfo(f"正在前往下一个目标: {location_name} at ({coords[0]}, {coords[1]})")

    def odom_callback(self, msg):
        """Checks if the robot has reached the current goal."""
        if self.current_goal is None or not self.task_active:
            return

        current_pos = msg.pose.pose.position
        goal_pos = self.current_goal.pose.position
        
        distance = math.sqrt((current_pos.x - goal_pos.x)**2 + (current_pos.y - goal_pos.y)**2)
        
        # Debug info every 50 callbacks (roughly every 0.25 seconds at 200Hz)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        #if self._debug_counter % 50 == 0:
            #rospy.loginfo(f"当前位置: ({current_pos.x:.2f}, {current_pos.y:.2f}), 目标: ({goal_pos.x:.2f}, {goal_pos.y:.2f}), 距离: {distance:.2f}m")
        
        if distance < self.goal_tolerance:
            rospy.loginfo(f"已到达目标: {self.waypoint_queue[0][0]}")
            self.waypoint_queue.pop(0)
            self.process_next_waypoint()

    def process_command_with_llm(self, command):
        """Processes a natural language command using LLM with function calling."""
        system_prompt = f"""你是一个智能机器人助手。你的任务是解析用户的导航指令。
可用的位置有: {list(self.locations.keys())}。
用户可能会要求去单个地点、多个地点，或者进行"巡检一圈"。
当用户要求"巡检一圈"时，你应该调用`navigate_robot`函数，并将locations参数设置为 `['full_tour']`。
对于其他导航指令，提取所有提到的地点名称，并按顺序放入列表中，然后调用`navigate_robot`函数。
"""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": command}
        ]
        
        try:
            response = self.client.chat.completions.create(
                model="qwen-plus",
                messages=messages,
                tools=self.tools,
                tool_choice="auto",
            )
            
            response_message = response.choices[0].message
            tool_calls = response_message.tool_calls

            if tool_calls:
                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    function_to_call = self.available_functions.get(function_name)
                    if function_to_call:
                        function_args = json.loads(tool_call.function.arguments)
                        return function_to_call(**function_args)
                    else:
                        return f"Function {function_name} not found."

            return response_message.content if response_message.content else "我不知道如何执行该指令。"

        except Exception as e:
            rospy.logerr(f"Error calling LLM: {e}")
            return "调用大模型时出错。"

    def run(self):
        rospy.loginfo("请输入指令 (例如, '去配电柜区域', '先去变压器1再去开关设备1', '巡检一圈'):")
        while not rospy.is_shutdown():
            try:
                print("> ", end="", flush=True)
                command = input()
                if command.strip():
                    response = self.process_command_with_llm(command.strip())
                    print(f"响应: {response}")
                else:
                    print("请输入有效指令...")
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                rospy.loginfo("Shutting down NLP Commander.")
                break
            except EOFError:
                rospy.loginfo("输入结束，关闭 NLP Commander。")
                break
            except Exception as e:
                rospy.logerr(f"输入处理错误: {e}")
                print("输入错误，请重试...")

if __name__ == '__main__':
    try:
        commander = NlpCommander()
        if not rospy.is_shutdown():
            commander.run()
    except rospy.ROSInterruptException:
        pass 