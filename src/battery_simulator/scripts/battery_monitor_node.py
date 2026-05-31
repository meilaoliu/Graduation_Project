#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
battery_monitor_node
====================
虚拟电量模型，与机器人物理引擎/样条类型无关。

电量更新（每 dt 秒）:
    drain = base_rate * dt
          + (k_v * |v| + k_w * |w|) * dt          # 运动额外消耗
    +拍照: 每次 /photo_event 扣 photo_cost

发布:
    /battery_state          BatteryState   1Hz
    /low_battery_alert      Bool           当低于 low_threshold 一次性 latched

服务:
    /charge                 std_srvs/Trigger  开始充电（充满到 100%）
    /reset_battery          std_srvs/Trigger  电量重置到 100%
"""

import math
import os
import threading
import xml.etree.ElementTree as ET

import rospy
from std_msgs.msg import Header, Bool
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry

from battery_simulator.msg import BatteryState

try:
    from inspection_services.msg import PhotoEvent
    HAVE_PHOTO_EVENT = True
except ImportError:
    HAVE_PHOTO_EVENT = False


class BatteryMonitor:
    def __init__(self):
        # 参数（rosparam）
        self.initial_pct = float(rospy.get_param('~initial_percentage', 100.0))
        self.base_rate = float(rospy.get_param('~base_rate', 0.005))      # %/s 静止待机
        self.k_v = float(rospy.get_param('~k_v', 0.05))                    # %·s/m
        self.k_w = float(rospy.get_param('~k_w', 0.02))                    # %·s/rad
        self.photo_cost = float(rospy.get_param('~photo_cost', 0.3))       # %/photo
        self.low_threshold = float(rospy.get_param('~low_threshold', 20.0))
        self.critical_threshold = float(rospy.get_param('~critical_threshold', 5.0))
        self.charge_rate = float(rospy.get_param('~charge_rate', 5.0))     # %/s
        self.update_rate_hz = float(rospy.get_param('~update_rate_hz', 10.0))
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 1.0))
        self.odom_topic = rospy.get_param('~odom_topic', '/odom_adjust')
        self.nominal_v = float(rospy.get_param('~nominal_v', 1.0))         # 静止时估算续航的标称速度
        self.enforce_charge_location = bool(rospy.get_param('~enforce_charge_location', True))
        self.charge_x = float(rospy.get_param('~charge_x', 9.0))
        self.charge_y = float(rospy.get_param('~charge_y', 27.0))
        self.charge_radius = float(rospy.get_param('~charge_radius', 1.0))
        self.charge_point_name = rospy.get_param('~charge_point_name', '')
        self.charge_osm_path = rospy.get_param('~charge_osm_path', '')

        self.pct = self.initial_pct
        self.charging = False
        self.last_v = 0.0
        self.last_w = 0.0
        self.current_xy = None
        self.lock = threading.Lock()
        self.alert_sent = False

        # ROS 接口
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        if HAVE_PHOTO_EVENT:
            rospy.Subscriber('/photo_event', PhotoEvent, self.photo_cb, queue_size=10)
        else:
            rospy.logwarn("[battery] inspection_services.msg.PhotoEvent not found; photo_cost disabled.")

        self.state_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
        self.alert_pub = rospy.Publisher('/low_battery_alert', Bool, queue_size=1, latch=True)

        rospy.Service('/charge', Trigger, self.handle_charge)
        rospy.Service('/reset_battery', Trigger, self.handle_reset)

        # 计时器
        self.update_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate_hz), self.update_cb)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.publish_cb)

        rospy.loginfo("[battery] init pct=%.1f, base=%.4f%%/s, k_v=%.3f, k_w=%.3f, photo=%.2f%%",
                      self.pct, self.base_rate, self.k_v, self.k_w, self.photo_cost)

    def _default_osm_path(self):
        try:
            import rospkg
            return os.path.join(rospkg.RosPack().get_path('nlp_commander'), 'maps', 'substation.osm')
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.abspath(os.path.join(here, '..', '..', 'nlp_commander', 'maps', 'substation.osm'))

    def _resolve_charge_xy(self):
        path = self.charge_osm_path or self._default_osm_path()
        if not path or not os.path.exists(path):
            return self.charge_x, self.charge_y

        try:
            root = ET.parse(path).getroot()
        except Exception as e:
            rospy.logwarn("[battery] failed to parse charge osm %s: %s", path, e)
            return self.charge_x, self.charge_y

        named_candidates = {}
        role_candidate = None
        for node in root.findall('node'):
            lat = node.get('lat')
            lon = node.get('lon')
            if lat is None or lon is None:
                continue
            tags = {tag.get('k'): tag.get('v') for tag in node.findall('tag')}
            name = tags.get('name', '')
            try:
                xy = (float(lon), float(lat))
            except ValueError:
                continue
            if name:
                named_candidates[name] = xy
            if tags.get('role') == 'charge_point' or tags.get('device_type') == 'charge_point':
                role_candidate = xy

        if self.charge_point_name and self.charge_point_name in named_candidates:
            return named_candidates[self.charge_point_name]
        if role_candidate is not None:
            return role_candidate
        for name in ('充电口', '充电点', '入口点'):
            if name in named_candidates:
                return named_candidates[name]
        return self.charge_x, self.charge_y

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        with self.lock:
            self.current_xy = (float(p.x), float(p.y))
            self.last_v = math.sqrt(v.x * v.x + v.y * v.y)
            self.last_w = abs(w.z)

    def photo_cb(self, _msg):
        with self.lock:
            if not self.charging:
                self.pct = max(0.0, self.pct - self.photo_cost)

    def update_cb(self, event):
        dt = 1.0 / self.update_rate_hz
        with self.lock:
            if self.charging:
                self.pct = min(100.0, self.pct + self.charge_rate * dt)
                if self.pct >= 100.0 - 1e-3:
                    self.charging = False
                    self.alert_sent = False  # 充满后允许下一次低电报警
                    rospy.loginfo("[battery] fully charged.")
            else:
                drain = self.base_rate * dt + (self.k_v * self.last_v + self.k_w * self.last_w) * dt
                self.pct = max(0.0, self.pct - drain)

            # 低电报警 (latched 一次)
            if not self.charging and not self.alert_sent and self.pct <= self.low_threshold:
                self.alert_pub.publish(Bool(data=True))
                self.alert_sent = True
                rospy.logwarn("[battery] LOW BATTERY ALERT @ %.1f%%", self.pct)

    def publish_cb(self, event):
        with self.lock:
            pct = self.pct
            charging = self.charging
            v = self.last_v

        # 估算剩余里程：稳态线速度下，drain ≈ (base + k_v*v) * (d / v)
        # v 太小（停车时）退回到一个标称巡航速度 nominal_v，让 UI 始终能看到一个合理估计
        v_eff = v if v > 0.05 else self.nominal_v
        if v_eff > 1e-3:
            drain_per_m = self.base_rate / v_eff + self.k_v
            remain = pct / max(drain_per_m, 1e-6)
        else:
            remain = float('nan')

        if charging:
            status = "charging"
        elif pct <= self.critical_threshold:
            status = "critical"
        elif pct <= self.low_threshold:
            status = "low"
        else:
            status = "ok"

        msg = BatteryState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.percentage = pct
        msg.charging = charging
        msg.estimated_remaining_distance_m = remain if remain == remain else 0.0  # NaN→0
        msg.status = status
        self.state_pub.publish(msg)

    def handle_charge(self, _req):
        with self.lock:
            if self.enforce_charge_location:
                if self.current_xy is None:
                    return TriggerResponse(success=False, message="charge rejected: odom unavailable")
                charge_x, charge_y = self._resolve_charge_xy()
                distance = math.hypot(self.current_xy[0] - charge_x, self.current_xy[1] - charge_y)
                if distance > self.charge_radius:
                    return TriggerResponse(
                        success=False,
                        message=f"charge rejected: distance to charger {distance:.2f}m > {self.charge_radius:.2f}m",
                    )
            self.charging = True
        rospy.loginfo("[battery] charging started @ %.1f%%", self.pct)
        return TriggerResponse(success=True, message="charging started")

    def handle_reset(self, _req):
        pct = float(rospy.get_param("~reset_to_percentage", 100.0))
        pct = max(0.0, min(100.0, pct))
        with self.lock:
            self.pct = pct
            self.charging = False
            self.alert_sent = False
        try:
            rospy.delete_param("~reset_to_percentage")
        except Exception:
            pass
        rospy.loginfo("[battery] reset to %.1f%%", pct)
        return TriggerResponse(success=True, message=f"battery reset to {pct:.1f}%")


def main():
    rospy.init_node('battery_monitor_node')
    BatteryMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
