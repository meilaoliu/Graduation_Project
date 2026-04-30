# -*- coding: utf-8 -*-
"""
分段调度器 SegmentScheduler
============================
- 将 (waypoint_path, stop_flags) 切分为多个"段"，每段以停留点为终点
- 按段调用 ego_planner 的全局轨迹接口（/global_waypoints）
- 段完成后等待 /segment_done，触发 /take_photo 服务，dwell，进入下一段
- 监听 /low_battery_alert：当前段结束后插入返航段→/charge→续接剩余段

与 B-spline / MINCO 双版本规划器兼容：所有交互都通过高层 ROS 接口完成。
"""

import math
import threading
from collections import namedtuple
from typing import List, Tuple, Optional, Callable

import rospy
from std_msgs.msg import UInt32, Bool, Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

from .graph_utils import SubstationGraph

# 软依赖：拍照与电量服务可选
try:
    from inspection_services.srv import TakePhoto, TakePhotoRequest
    HAVE_PHOTO_SRV = True
except ImportError:
    HAVE_PHOTO_SRV = False

try:
    from std_srvs.srv import Trigger
    HAVE_TRIGGER = True
except ImportError:
    HAVE_TRIGGER = False

try:
    from battery_simulator.msg import BatteryState
    HAVE_BATTERY_MSG = True
except ImportError:
    HAVE_BATTERY_MSG = False


Waypoint = namedtuple('Waypoint', ['name', 'x', 'y', 'stop_required'])
Segment = namedtuple('Segment', ['index', 'waypoints', 'end_name', 'end_xy', 'kind'])
# kind: 'mission' 正常巡检段 | 'return_charge' 返航充电段 | 'resume' 充满后回任务起点


class SegmentScheduler:
    """分段执行 + 低电量返航 + 拍照同步"""

    def __init__(
        self,
        graph: SubstationGraph,
        get_current_xy: Callable[[], Optional[Tuple[float, float]]],
        say: Callable[[str], None] = None,
        photo_dir: str = None,  # currently only forwarded to label
    ):
        self.graph = graph
        self.get_current_xy = get_current_xy
        self.say = say or (lambda m: rospy.loginfo(m))

        # 参数
        self.dwell_seconds = float(rospy.get_param('~dwell_seconds', 3.0))
        self.segment_done_timeout = float(rospy.get_param('~segment_done_timeout', 90.0))
        self.arrive_tolerance = float(rospy.get_param('~arrive_tolerance', 0.5))  # 与 FSM dist_to_goal_actual 匹配
        self.charge_full_threshold = float(rospy.get_param('~charge_full_threshold', 95.0))
        self.charge_wait_timeout = float(rospy.get_param('~charge_wait_timeout', 120.0))
        self.low_battery_threshold = float(rospy.get_param('~low_battery_threshold', 20.0))
        self.global_waypoints_topic = rospy.get_param('~global_waypoints_topic', '/global_waypoints')
        self.frame_id = rospy.get_param('~map_frame', 'map')
        self.waypoint_z = float(rospy.get_param('~waypoint_z', 0.75))  # 与 FSM odom_pos_.z 匹配，避免 3D 距离永不收敛

        # ROS 接口
        self.gwp_pub = rospy.Publisher(self.global_waypoints_topic, Path,
                                       queue_size=2, latch=False)
        rospy.Subscriber('/segment_done', UInt32, self._on_segment_done, queue_size=10)
        rospy.Subscriber('/low_battery_alert', Bool, self._on_low_battery, queue_size=1)
        if HAVE_BATTERY_MSG:
            rospy.Subscriber('/battery_state', BatteryState, self._on_battery, queue_size=1)
        self._cur_xy_from_odom = None
        rospy.Subscriber('/odom_adjust', Odometry, self._on_odom, queue_size=1)

        # 状态
        self._lock = threading.Lock()
        self._segments: List[Segment] = []          # 待执行段队列
        self._current_segment: Optional[Segment] = None
        self._segment_done_evt = threading.Event()
        # mission-wide 单调递增段编号，避免任务间复用 id 引起的 stale done 命中
        self._next_seg_id = 1
        self._expected_done_seq = -1
        self._last_done_seq = -1
        self._task_active = False
        self._abort_flag = False
        self._low_battery = False
        self._returning_to_charge = False
        self._battery_pct = 100.0
        self._battery_charging = False

        self._worker: Optional[threading.Thread] = None

        # 等待 /global_waypoints 订阅者建立
        t0 = rospy.Time.now()
        while self.gwp_pub.get_num_connections() == 0 and \
                (rospy.Time.now() - t0).to_sec() < 3.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

    # ------------------------ 公共 API ------------------------
    def start_task(self, waypoints: List[Tuple[str, Tuple[float, float], bool]],
                   task_description: str = "") -> str:
        """waypoints: list of (name, (x, y), stop_required)；首尾应为停留点"""
        if not waypoints:
            return "❌ 无有效航点"

        with self._lock:
            if self._task_active:
                return "❌ 已有任务在执行，先 stop 再下达新任务"
            self._abort_flag = False
            # 不清 _low_battery：保留 latched 警报，避免低电量下重启任务漏检
            # 改为根据当前实测 pct 重新判断
            self._low_battery = self._battery_pct <= self.low_battery_threshold
            self._returning_to_charge = False
            self._segments = self._split_into_segments(waypoints, kind='mission')
            if not self._segments:
                return ("❌ 任务未生成有效段（可能仅含起点/无停留目标）；"
                        "请确认指令包含至少一个目标设备")
            self._task_active = True

        seg_brief = " | ".join(f"→{s.end_name}" for s in self._segments)
        self.say(f"🛣️ 任务分段（{len(self._segments)}段）: {seg_brief}")

        if self._low_battery:
            self.say("⚠️ 启动时电量已低，将先返航充电后再执行任务。")
            self._inject_return_charge_segment()

        self._worker = threading.Thread(
            target=self._run_loop, name='segment_scheduler', daemon=True)
        self._worker.start()
        return f"✅ 任务启动: {task_description}（共{len(self._segments)}段）"

    def stop_task(self) -> str:
        with self._lock:
            self._abort_flag = True
            self._segments = []
            self._task_active = False
        self._segment_done_evt.set()
        # 主动给 FSM 发一个"原地"目标，打断当前正在执行的轨迹
        self._publish_halt_path()
        return "🛑 已请求停止当前任务"

    def _publish_halt_path(self):
        """发布一条单点 Path 指向当前 odom 位置，让 FSM 把目标切到原地。"""
        cur = self.get_current_xy() or self._cur_xy_from_odom
        if cur is None:
            rospy.logwarn("[scheduler] no odom yet, cannot publish halt path")
            return
        path = Path()
        path.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        # 用一个新的、远超已知 seq 的 id，避免和后续真实段冲突
        with self._lock:
            self._next_seg_id += 1
            path.header.seq = self._next_seg_id
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = cur[0]
        ps.pose.position.y = cur[1]
        ps.pose.position.z = self.waypoint_z
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
        try:
            self.gwp_pub.publish(path)
            rospy.loginfo(f"[scheduler] published HALT path seq={path.header.seq} "
                          f"at ({cur[0]:.2f},{cur[1]:.2f})")
        except Exception as e:
            rospy.logwarn(f"[scheduler] halt publish failed: {e}")

    def is_active(self) -> bool:
        with self._lock:
            return self._task_active

    def status(self) -> dict:
        with self._lock:
            cur = self._current_segment
            return {
                "active": self._task_active,
                "current_segment": cur.end_name if cur else None,
                "current_segment_kind": cur.kind if cur else None,
                "remaining_segments": len(self._segments),
                "low_battery": self._low_battery,
                "returning_to_charge": self._returning_to_charge,
                "battery_pct": self._battery_pct,
            }

    # ------------------------ 段拆分 ------------------------
    @staticmethod
    def _split_into_segments(waypoints, kind='mission') -> List[Segment]:
        """把 waypoints 按 stop_required=True 切段。每段终点必为停留点。
        起点（首个 stop）只跳过一次，避免连续 stop 时丢段。"""
        segs: List[Segment] = []
        buf: List[Waypoint] = []
        idx = 0
        skipped_initial_start = False
        for name, (x, y), stop in waypoints:
            buf.append(Waypoint(name, x, y, stop))
            if stop:
                # 仅跳过第一次出现的"单一起点段"
                if (not skipped_initial_start) and len(buf) == 1 and not segs:
                    skipped_initial_start = True
                    buf = []
                    continue
                segs.append(Segment(
                    index=idx,
                    waypoints=list(buf),
                    end_name=buf[-1].name,
                    end_xy=(buf[-1].x, buf[-1].y),
                    kind=kind,
                ))
                idx += 1
                buf = []
        if buf:  # 末尾未以停留点结束（异常输入）
            segs.append(Segment(
                index=idx,
                waypoints=list(buf),
                end_name=buf[-1].name,
                end_xy=(buf[-1].x, buf[-1].y),
                kind=kind,
            ))
        return segs

    # ------------------------ 主循环 ------------------------
    def _run_loop(self):
        try:
            while not rospy.is_shutdown():
                with self._lock:
                    if self._abort_flag:
                        break
                    if not self._segments:
                        break
                    seg = self._segments.pop(0)
                    # 分配 mission-wide 单调递增 id，避免任务/段间复用 → stale done 误判
                    seg = seg._replace(index=self._next_seg_id)
                    self._next_seg_id += 1
                    self._current_segment = seg
                    self._expected_done_seq = seg.index
                    self._last_done_seq = -1
                    self._segment_done_evt.clear()

                self.say(f"▶️ 段 #{seg.index} ({seg.kind}) → 终点: {seg.end_name}")
                ok = self._execute_segment(seg)
                if not ok:
                    self.say(f"❌ 段 #{seg.index} 执行失败/超时；任务终止")
                    break

                # 段完成动作（按 kind 分支）
                if seg.kind == 'mission':
                    self._do_photo_if_needed(seg)
                    rospy.sleep(self.dwell_seconds)
                elif seg.kind == 'return_charge':
                    charged_ok = self._do_charge_cycle()
                    if not charged_ok:
                        self.say("❌ 充电失败/未达阈值，停止剩余任务以保安全。")
                        with self._lock:
                            self._segments = []
                        break

                # 低电量插队（仅 mission 段后才检查）
                with self._lock:
                    pending_mission = [s for s in self._segments if s.kind == 'mission']
                    need_return = (self._low_battery and
                                   not self._returning_to_charge and
                                   pending_mission and
                                   seg.kind == 'mission')
                if need_return:
                    self._inject_return_charge_segment()

            with self._lock:
                done = not self._segments and not self._abort_flag
                self._task_active = False
                self._current_segment = None

            if done:
                self.say("🎉 全部段执行完毕，任务完成。")
            elif self._abort_flag:
                self.say("🛑 任务被外部停止。")
        except Exception as e:
            rospy.logerr(f"[scheduler] 异常退出: {e}")
            with self._lock:
                self._task_active = False
                self._current_segment = None

    def _execute_segment(self, seg: Segment) -> bool:
        """发布段的全局 waypoints，等待 /segment_done（首选）或 odom 兜底。"""
        path = Path()
        path.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        path.header.seq = seg.index
        for wp in seg.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = path.header.stamp
            ps.pose.position.x = wp.x
            ps.pose.position.y = wp.y
            ps.pose.position.z = self.waypoint_z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if not path.poses:
            return True

        rospy.sleep(0.05)
        self.gwp_pub.publish(path)
        rospy.loginfo(f"[scheduler] published /global_waypoints seg#{seg.index} "
                      f"with {len(path.poses)} pts → {seg.end_name}")

        deadline = rospy.Time.now() + rospy.Duration(self.segment_done_timeout)
        rate = rospy.Rate(10)
        # 兜底：odom 容差非常严格 (0.5m，与 FSM 对齐) 且要求速度近零，避免抢跑
        while not rospy.is_shutdown():
            if self._abort_flag:
                return False
            with self._lock:
                done_match = (self._segment_done_evt.is_set() and
                              self._last_done_seq == self._expected_done_seq)
            if done_match:
                return True
            cur = self.get_current_xy() or self._cur_xy_from_odom
            if cur is not None:
                d = math.hypot(cur[0] - seg.end_xy[0], cur[1] - seg.end_xy[1])
                if d < self.arrive_tolerance:
                    rospy.loginfo(f"[scheduler] seg#{seg.index} arrived by odom (d={d:.2f}m)")
                    return True
            if rospy.Time.now() > deadline:
                rospy.logwarn(f"[scheduler] seg#{seg.index} timeout after "
                              f"{self.segment_done_timeout}s")
                return False
            rate.sleep()
        return False

    def _do_photo_if_needed(self, seg: Segment):
        """到达停留点 → 调用 /take_photo 服务"""
        if not seg.end_name:
            return
        if not HAVE_PHOTO_SRV:
            self.say("📷 (拍照服务未编译，跳过)")
            return
        try:
            rospy.wait_for_service('/take_photo', timeout=2.0)
            srv = rospy.ServiceProxy('/take_photo', TakePhoto)
            req = TakePhotoRequest(label=seg.end_name)
            resp = srv(req)
            if resp.success:
                self.say(f"📷 已在 {seg.end_name} 拍照: {resp.filepath}")
            else:
                self.say(f"⚠️ 拍照失败 @ {seg.end_name}: {resp.message}")
        except (rospy.ROSException, rospy.ServiceException) as e:
            self.say(f"⚠️ /take_photo 不可用: {e}")

    def _inject_return_charge_segment(self):
        """构造一个段：从当前位置（即上一段终点）→ 充电点。"""
        try:
            cname, cx, cy = self.graph.get_charge_point()
        except Exception as e:
            self.say(f"⚠️ 无法获取充电点: {e}；放弃自动返航。")
            return

        with self._lock:
            charge_seg = Segment(
                index=0,  # 会在主循环 pop 时重排
                waypoints=[Waypoint(cname, cx, cy, True)],
                end_name=cname,
                end_xy=(cx, cy),
                kind='return_charge',
            )
            # 插队到队首，剩余 mission 段保持顺序
            self._segments.insert(0, charge_seg)
            self._returning_to_charge = True
        self.say(f"🔋 低电量触发返航：插入充电段 → {cname}")

    def _do_charge_cycle(self) -> bool:
        """到达充电点后调用 /charge，等待充满。返回是否成功（达阈值）。"""
        if not HAVE_TRIGGER:
            self.say("⚠️ std_srvs/Trigger 不可用，跳过充电。")
            return False
        try:
            rospy.wait_for_service('/charge', timeout=2.0)
            srv = rospy.ServiceProxy('/charge', Trigger)
            resp = srv()
            self.say(f"🔌 开始充电: {resp.message}")
        except (rospy.ROSException, rospy.ServiceException) as e:
            self.say(f"⚠️ /charge 调用失败: {e}")
            return False

        deadline = rospy.Time.now() + rospy.Duration(self.charge_wait_timeout)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            with self._lock:
                pct = self._battery_pct
                charging = self._battery_charging
                aborted = self._abort_flag
            if aborted:
                return False
            if pct >= self.charge_full_threshold and not charging:
                self.say(f"🔋 充电完成 ({pct:.0f}%)，恢复任务。")
                with self._lock:
                    self._low_battery = False
                    self._returning_to_charge = False
                return True
            if rospy.Time.now() > deadline:
                # 超时但若已基本充足仍可继续
                if pct >= self.charge_full_threshold:
                    with self._lock:
                        self._low_battery = False
                        self._returning_to_charge = False
                    return True
                self.say(f"⚠️ 充电超时 ({pct:.0f}%)，电量未达安全阈值。")
                return False
            rate.sleep()
        return False

    # ------------------------ ROS 回调 ------------------------
    def _on_segment_done(self, msg: UInt32):
        with self._lock:
            self._last_done_seq = int(msg.data)
            self._segment_done_evt.set()
        rospy.loginfo(f"[scheduler] /segment_done received id={int(msg.data)} "
                      f"(expected={self._expected_done_seq})")

    def _on_low_battery(self, msg: Bool):
        if bool(msg.data):
            with self._lock:
                self._low_battery = True
            rospy.logwarn("[scheduler] /low_battery_alert received → 将于当前段结束后返航")

    def _on_battery(self, msg):
        with self._lock:
            self._battery_pct = float(msg.percentage)
            self._battery_charging = bool(msg.charging)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_xy_from_odom = (p.x, p.y)
