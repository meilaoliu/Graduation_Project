# -*- coding: utf-8 -*-
"""
航点管理器模块
负责跟踪机器人当前位置，供 NLP 调度器选择拓扑起点。
"""

import rospy
from nav_msgs.msg import Odometry
from typing import Tuple, Optional
from .graph_utils import SubstationGraph

class WaypointManager:
    """机器人当前位置跟踪器。"""
    
    def __init__(self, odom_topic: str = '/odom_adjust'):
        
        # 必须先初始化所有属性，因为odom_callback会立即使用
        # 图管理
        self.graph = SubstationGraph()
        
        # 当前位置信息
        self.current_position: Optional[object] = None
        self.current_position_name = "未知位置"
        
        # ROS通信设置
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        
        rospy.loginfo("位置跟踪器初始化完成")
    
    def odom_callback(self, msg: Odometry):
        """里程计回调，更新当前位置信息。"""
        # 更新当前位置信息
        self.current_position = msg.pose.pose.position
        self.current_position_name = self.graph.find_nearest_location(
            self.current_position.x, self.current_position.y
        )
    
    def stop_current_task(self) -> str:
        """兼容旧调用；实际任务停止由 SegmentScheduler 负责。"""
        return "✅ 当前没有活跃任务"
    
    def pause_current_task(self) -> str:
        """分段全局轨迹模式暂不支持暂停。"""
        return "❌ 分段全局轨迹模式暂不支持暂停，请使用停止任务"
    
    def resume_current_task(self) -> str:
        """分段全局轨迹模式暂不支持恢复。"""
        return "❌ 分段全局轨迹模式暂不支持恢复，请重新下达任务"
    
    def get_current_status(self) -> dict:
        """获取当前状态信息"""
        current_info = "未知位置"
        if self.current_position:
            current_info = f"{self.current_position_name} ({self.current_position.x:.1f}, {self.current_position.y:.1f})"
        
        return {
            "current_position": current_info,
            "task_active": False,
            "remaining_waypoints": 0,
            "waypoint_queue": [],
            "stop_points": [],
            "current_target": None
        }
    
    def get_current_position_name(self) -> str:
        """获取当前位置名称"""
        return self.current_position_name
    
    def get_current_coordinates(self) -> Optional[Tuple[float, float]]:
        """获取当前坐标"""
        if self.current_position:
            return (self.current_position.x, self.current_position.y)
        return None
