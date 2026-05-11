# -*- coding: utf-8 -*-
"""
航点管理器模块
负责管理任务队列和机器人导航控制
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from typing import List, Tuple, Optional, Callable
from .graph_utils import SubstationGraph

class WaypointManager:
    """航点管理器类"""
    
    def __init__(self, goal_topic: str = '/move_base_simple/goal', 
                 odom_topic: str = '/odom_adjust'):
        
        # 必须先初始化所有属性，因为odom_callback会立即使用
        # 图管理
        self.graph = SubstationGraph()
        
        # 任务队列和状态
        self.waypoint_queue: List[Tuple[str, Tuple[float, float], bool]] = []
        self.current_goal: Optional[PoseStamped] = None
        self.goal_tolerance = 2.0  # meters
        self.task_active = False
        
        # 当前位置信息
        self.current_position: Optional[object] = None
        self.current_position_name = "未知位置"
        
        # ROS通信设置
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        
        # 经典逐航点模式需要该订阅者；全局轨迹模式不应因启动顺序卡死。
        goal_wait_timeout = float(rospy.get_param('~goal_connection_timeout', 3.0))
        rospy.loginfo("等待目标发布者建立连接...")
        t0 = rospy.Time.now()
        while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() >= goal_wait_timeout:
                break
            rospy.sleep(0.1)
        if self.goal_pub.get_num_connections() > 0:
            rospy.loginfo(f"目标发布者已连接，订阅者数量: {self.goal_pub.get_num_connections()}")
        else:
            rospy.logwarn("目标发布者暂无订阅者，继续启动；全局轨迹模式可忽略此警告。")
        
        # 回调函数
        self.on_waypoint_reached: Optional[Callable[[str], None]] = None
        self.on_task_completed: Optional[Callable[[], None]] = None
        
        rospy.loginfo("航点管理器初始化完成")
    
    def set_callbacks(self, on_waypoint_reached: Callable[[str], None] = None,
                     on_task_completed: Callable[[], None] = None):
        """设置回调函数"""
        self.on_waypoint_reached = on_waypoint_reached
        self.on_task_completed = on_task_completed
    
    def start_navigation_task(self, waypoint_path: List[str], task_description: str = "", stop_flags: List[bool] = None) -> str:
        """
        开始导航任务
        
        Args:
            waypoint_path: 航点路径列表
            task_description: 任务描述
            
        Returns:
            任务启动结果信息
        """
        self.waypoint_queue.clear()
        self.current_goal = None
        self.task_active = True
        
        if not waypoint_path:
            self.task_active = False
            return "未提供有效的航点路径。"
        
        if stop_flags is None or len(stop_flags) != len(waypoint_path):
            stop_flags = [True] * len(waypoint_path)

        # 验证并添加有效航点到队列
        valid_waypoints = []
        for waypoint_name, stop_required in zip(waypoint_path, stop_flags):
            coords = self.graph.get_location_coordinates(waypoint_name)
            if coords:
                valid_waypoints.append((waypoint_name, coords, stop_required))
            else:
                rospy.logwarn(f"航点 '{waypoint_name}' 不在已知位置列表中")
        
        if not valid_waypoints:
            self.task_active = False
            return "所有指定的航点均无效。"
        
        self.waypoint_queue = valid_waypoints
        self.process_next_waypoint()
        
        waypoint_names = [wp[0] for wp in valid_waypoints]
        stop_names = [wp[0] for wp in valid_waypoints if wp[2]]
        rospy.loginfo(f"开始执行任务: {task_description}")
        return f"任务启动: {task_description}\n巡检路径: {' → '.join(waypoint_names)}\n停留点: {' → '.join(stop_names)}"
    
    def process_next_waypoint(self):
        """处理下一个航点"""
        if not self.waypoint_queue:
            rospy.loginfo("巡检任务完成，所有航点均已到达。")
            self.current_goal = None
            self.task_active = False
            
            if self.on_task_completed:
                self.on_task_completed()
            return
        
        location_name, coords, stop_required = self.waypoint_queue[0]
        
        # 创建目标点消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = coords[0]
        pose_msg.pose.position.y = coords[1]
        pose_msg.pose.position.z = 0.75
        pose_msg.pose.orientation.w = 1.0
        
        self.current_goal = pose_msg
        
        # 确保消息能够正确发布
        rospy.sleep(0.1)  # 小延迟确保连接稳定
        self.goal_pub.publish(self.current_goal)
        
        # 确认发布成功
        waypoint_type = "停留点" if stop_required else "过渡点"
        if self.goal_pub.get_num_connections() > 0:
            rospy.loginfo(f"🎯 前往{waypoint_type}: {location_name} 坐标({coords[0]}, {coords[1]}) - 目标已发布给{self.goal_pub.get_num_connections()}个订阅者")
        else:
            rospy.logwarn(f"⚠️ 警告: 目标发布但没有订阅者 - {location_name}")
            # 重试发布
            rospy.sleep(0.5)
            self.goal_pub.publish(self.current_goal)
            rospy.loginfo(f"🔄 重新发布目标: {location_name}")
    
    def odom_callback(self, msg: Odometry):
        """里程计回调，检查是否到达目标"""
        # 更新当前位置信息
        self.current_position = msg.pose.pose.position
        self.current_position_name = self.graph.find_nearest_location(
            self.current_position.x, self.current_position.y
        )
        
        if self.current_goal is None or not self.task_active:
            return
        
        # 检查是否到达目标
        current_pos = msg.pose.pose.position
        goal_pos = self.current_goal.pose.position
        
        distance = math.sqrt(
            (current_pos.x - goal_pos.x)**2 + (current_pos.y - goal_pos.y)**2
        )
        
        if distance < self.goal_tolerance:
            reached_waypoint, _, stop_required = self.waypoint_queue[0]
            if stop_required:
                rospy.loginfo(f"✅ 已到达停留点: {reached_waypoint}")
            else:
                rospy.loginfo(f"✅ 已通过过渡点: {reached_waypoint}")
            
            # 仅对需要停留的航点触发到达回调
            if stop_required and self.on_waypoint_reached:
                self.on_waypoint_reached(reached_waypoint)
            
            # 移除已到达的航点，处理下一个
            self.waypoint_queue.pop(0)
            self.process_next_waypoint()
    
    def stop_current_task(self) -> str:
        """停止当前任务"""
        self.waypoint_queue.clear()
        self.current_goal = None
        self.task_active = False
        rospy.loginfo("任务已停止")
        return "✅ 当前任务已停止"
    
    def pause_current_task(self) -> str:
        """暂停当前任务"""
        if self.task_active:
            self.task_active = False
            rospy.loginfo("任务已暂停")
            return "⏸️ 任务已暂停"
        else:
            return "❌ 当前没有活跃的任务"
    
    def resume_current_task(self) -> str:
        """恢复当前任务"""
        if not self.task_active and self.waypoint_queue:
            self.task_active = True
            self.process_next_waypoint()
            rospy.loginfo("任务已恢复")
            return "▶️ 任务已恢复"
        else:
            return "❌ 没有可恢复的任务"
    
    def get_current_status(self) -> dict:
        """获取当前状态信息"""
        current_info = "未知位置"
        if self.current_position:
            current_info = f"{self.current_position_name} ({self.current_position.x:.1f}, {self.current_position.y:.1f})"
        
        return {
            "current_position": current_info,
            "task_active": self.task_active,
            "remaining_waypoints": len(self.waypoint_queue),
            "waypoint_queue": [wp[0] for wp in self.waypoint_queue],
            "stop_points": [wp[0] for wp in self.waypoint_queue if wp[2]],
            "current_target": self.waypoint_queue[0][0] if self.waypoint_queue else None
        }
    
    def get_current_position_name(self) -> str:
        """获取当前位置名称"""
        return self.current_position_name
    
    def get_current_coordinates(self) -> Optional[Tuple[float, float]]:
        """获取当前坐标"""
        if self.current_position:
            return (self.current_position.x, self.current_position.y)
        return None
    
    def add_waypoint_to_queue(self, waypoint_name: str) -> bool:
        """向队列添加航点"""
        coords = self.graph.get_location_coordinates(waypoint_name)
        if coords:
            self.waypoint_queue.append((waypoint_name, coords, True))
            if self.task_active and self.current_goal is None:
                self.process_next_waypoint()
            return True
        return False
    
    def clear_waypoint_queue(self):
        """清空航点队列"""
        self.waypoint_queue.clear()
        rospy.loginfo("航点队列已清空")
    
    def set_goal_tolerance(self, tolerance: float):
        """设置目标容忍度"""
        self.goal_tolerance = tolerance
        rospy.loginfo(f"目标容忍度设置为: {tolerance}米") 
