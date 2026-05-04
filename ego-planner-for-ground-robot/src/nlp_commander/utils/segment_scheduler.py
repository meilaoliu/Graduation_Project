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
import time
from collections import namedtuple
from typing import Any, Dict, List, Tuple, Optional, Callable

import rospy
from std_msgs.msg import UInt32, Bool, Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

from .graph_utils import SubstationGraph
from .runtime_policy import BatteryPolicy, RuntimeEventLog

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


Waypoint = namedtuple('Waypoint', ['name', 'x', 'y', 'stop_required', 'photo_required'])
Segment = namedtuple('Segment', ['index', 'waypoints', 'end_name', 'end_xy', 'kind', 'photo_required'])
# kind: 'mission' 正常巡检段 | 'return_charge' 返航充电段 | 'resume' 充满后回任务起点


class SegmentScheduler:
    """分段执行 + 低电量返航 + 拍照同步"""

    def __init__(
        self,
        graph: SubstationGraph,
        get_current_xy: Callable[[], Optional[Tuple[float, float]]],
        say: Callable[[str], None] = None,
        on_event: Callable[[Dict[str, Any]], None] = None,
        photo_dir: str = None,  # currently only forwarded to label
    ):
        self.graph = graph
        self.get_current_xy = get_current_xy
        self.say = say or (lambda m: rospy.loginfo(m))
        self.on_event = on_event or (lambda event: None)

        # 参数
        self.dwell_seconds = float(rospy.get_param('~dwell_seconds', 3.0))
        self.segment_done_timeout = float(rospy.get_param('~segment_done_timeout', 90.0))
        self.arrive_tolerance = float(rospy.get_param('~arrive_tolerance', 0.5))  # 与 FSM dist_to_goal_actual 匹配
        self.charge_full_threshold = float(rospy.get_param('~charge_full_threshold', 95.0))
        self.charge_wait_timeout = float(rospy.get_param('~charge_wait_timeout', 120.0))
        self.low_battery_threshold = float(rospy.get_param('~low_battery_threshold', 20.0))
        self.enable_predictive_charging = bool(rospy.get_param('~enable_predictive_charging', True))
        self.battery_policy = BatteryPolicy(
            full_range_m=float(rospy.get_param('~battery_full_range_m', 300.0)),
            reserve_m=float(rospy.get_param('~battery_reserve_m', 15.0)),
        )
        self.time_budget_min_remaining_s = float(rospy.get_param('~time_budget_min_remaining_s', 30.0))
        self.max_time_budget_cycles = int(rospy.get_param('~max_time_budget_cycles', 20))
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
        self._mission_targets: List[Tuple[str, bool]] = []
        self._task_start_epoch: Optional[float] = None
        self._deadline_epoch: Optional[float] = None
        self._time_budget_cycle_index = 0
        self.event_log = RuntimeEventLog(max_events=int(rospy.get_param('~runtime_context_events', 40)))

        self._worker: Optional[threading.Thread] = None

        # 等待 /global_waypoints 订阅者建立
        t0 = rospy.Time.now()
        while self.gwp_pub.get_num_connections() == 0 and \
                (rospy.Time.now() - t0).to_sec() < 3.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def _emit_event(self, event_type: str, message: str, **data: Any):
        event = self.event_log.add(event_type, message, **data)
        try:
            self.on_event(event)
        except Exception as exc:
            rospy.logwarn(f"[scheduler] runtime event callback failed: {exc}")

    def _resolve_deadline_epoch(self, execution_options: Dict[str, Any]) -> Optional[float]:
        duration = self._safe_float(execution_options.get('duration_minutes'))
        if duration is not None and duration > 0:
            return time.time() + duration * 60.0

        until_epoch = self._safe_float(execution_options.get('until_epoch'))
        if until_epoch is not None and until_epoch > 0:
            # 兼容毫秒级时间戳。
            if until_epoch > 1000000000000:
                until_epoch /= 1000.0
            return until_epoch

        until_time = execution_options.get('until_time')
        if isinstance(until_time, str):
            deadline = self._parse_today_clock_time(until_time.strip())
            if deadline is not None:
                return deadline
        return None

    @staticmethod
    def _safe_float(value: Any) -> Optional[float]:
        if value is None or value == "":
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _parse_today_clock_time(value: str) -> Optional[float]:
        if not value:
            return None
        parts = value.replace('：', ':').split(':')
        if len(parts) < 2:
            return None
        try:
            hour = int(parts[0])
            minute = int(parts[1])
            second = int(parts[2]) if len(parts) > 2 else 0
        except ValueError:
            return None
        if not (0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 59):
            return None

        now_tuple = time.localtime()
        deadline_tuple = (
            now_tuple.tm_year, now_tuple.tm_mon, now_tuple.tm_mday,
            hour, minute, second,
            now_tuple.tm_wday, now_tuple.tm_yday, now_tuple.tm_isdst,
        )
        deadline = time.mktime(deadline_tuple)
        if deadline <= time.time():
            deadline += 24 * 3600
        return deadline

    def _time_budget_data_unlocked(self) -> Dict[str, Any]:
        now = time.time()
        data: Dict[str, Any] = {}
        if self._task_start_epoch is not None:
            data['elapsed_s'] = round(max(0.0, now - self._task_start_epoch), 1)
        if self._deadline_epoch is not None:
            data['deadline_epoch'] = round(self._deadline_epoch, 3)
            data['remaining_time_s'] = round(self._deadline_epoch - now, 1)
            data['time_budget_cycle'] = self._time_budget_cycle_index
        return data

    def _time_budget_data(self) -> Dict[str, Any]:
        with self._lock:
            return self._time_budget_data_unlocked()

    # ------------------------ 公共 API ------------------------
    def start_task(self, waypoints: List[Tuple[str, Tuple[float, float], bool]],
                   task_description: str = "",
                   execution_options: Optional[Dict[str, Any]] = None) -> str:
        """waypoints: list of (name, (x, y), stop_required[, photo_required])。"""
        if not waypoints:
            return "❌ 无有效航点"

        execution_options = execution_options or {}

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
            self._mission_targets = [(seg.end_name, seg.photo_required) for seg in self._segments]
            self._task_start_epoch = time.time()
            self._deadline_epoch = self._resolve_deadline_epoch(execution_options)
            self._time_budget_cycle_index = 1 if self._deadline_epoch is not None else 0
            self._task_active = True
            initial_segment_count = len(self._segments)
            self.event_log.clear()
            self._emit_event(
                "task_started",
                task_description,
                segment_count=initial_segment_count,
                **self._time_budget_data_unlocked(),
            )

            has_time_budget = self._deadline_epoch is not None

        seg_brief = " | ".join(f"→{s.end_name}" for s in self._segments)
        self.say(f"🛣️ 任务分段（{len(self._segments)}段）: {seg_brief}")
        if has_time_budget:
            self.say("⏱️ 限时任务采用运行时续巡：先执行一轮，后续按剩余时间动态追加。")

        if self._low_battery:
            self.say("⚠️ 启动时电量已低，将先返航充电后再执行任务。")
            self._inject_return_charge_segment(reason="battery_below_threshold_at_start")
            initial_segment_count += 1

        self._worker = threading.Thread(
            target=self._run_loop, name='segment_scheduler', daemon=True)
        self._worker.start()
        if has_time_budget:
            return f"✅ 任务启动: {task_description}（首轮{initial_segment_count}段，限时动态续巡）"
        return f"✅ 任务启动: {task_description}（共{initial_segment_count}段）"

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
                "time_budget": self._time_budget_data_unlocked(),
                "runtime_context": self.event_log.to_prompt_text(),
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
        normalized_waypoints = []
        for item in waypoints:
            if len(item) == 3:
                name, (x, y), stop = item
                photo = bool(stop)
            else:
                name, (x, y), stop, photo = item
            normalized_waypoints.append((name, (x, y), bool(stop), bool(photo)))

        for name, (x, y), stop, photo in normalized_waypoints:
            buf.append(Waypoint(name, x, y, stop, photo))
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
                    photo_required=buf[-1].photo_required,
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
                photo_required=buf[-1].photo_required,
            ))
        return segs

    # ------------------------ 主循环 ------------------------
    def _run_loop(self):
        try:
            while not rospy.is_shutdown():
                self._maybe_preempt_for_energy()
                with self._lock:
                    if self._abort_flag:
                        break
                    if not self._segments:
                        should_extend = self._deadline_epoch is not None
                    else:
                        should_extend = False

                if should_extend:
                    if self._append_time_budget_cycle():
                        continue
                    break

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
                self._emit_event(
                    "segment_started",
                    f"开始执行到 {seg.end_name}",
                    segment_id=seg.index,
                    kind=seg.kind,
                    **self._time_budget_data(),
                )
                ok = self._execute_segment(seg)
                if not ok:
                    self.say(f"❌ 段 #{seg.index} 执行失败/超时；任务终止")
                    self._emit_event(
                        "segment_failed",
                        f"到 {seg.end_name} 的段执行失败或超时",
                        segment_id=seg.index,
                        kind=seg.kind,
                        **self._time_budget_data(),
                    )
                    break
                self._emit_event(
                    "segment_finished",
                    f"已到达 {seg.end_name}",
                    segment_id=seg.index,
                    kind=seg.kind,
                    **self._time_budget_data(),
                )

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
                    self._repair_next_mission_after_charge(seg.end_name)

                # 低电量插队（仅 mission 段后才检查）
                with self._lock:
                    pending_mission = [s for s in self._segments if s.kind == 'mission']
                    need_return = (self._low_battery and
                                   not self._returning_to_charge and
                                   pending_mission and
                                   seg.kind == 'mission')
                if need_return:
                    self._inject_return_charge_segment(reason="battery_below_threshold")

            with self._lock:
                done = not self._segments and not self._abort_flag
                self._task_active = False
                self._current_segment = None

            if done:
                self.say("🎉 全部段执行完毕，任务完成。")
                self._emit_event("task_completed", "全部段执行完毕")
            elif self._abort_flag:
                self.say("🛑 任务被外部停止。")
                self._emit_event("task_aborted", "任务被外部停止")
        except Exception as e:
            rospy.logerr(f"[scheduler] 异常退出: {e}")
            with self._lock:
                self._task_active = False
                self._current_segment = None
            self._emit_event("scheduler_error", str(e))

    def _append_time_budget_cycle(self) -> bool:
        with self._lock:
            if self._abort_flag or not self._task_active:
                return False
            if self._deadline_epoch is None or not self._mission_targets:
                return False
            remaining_time_s = self._deadline_epoch - time.time()
            cycle_index = self._time_budget_cycle_index
            target_specs = list(self._mission_targets)

        if remaining_time_s <= self.time_budget_min_remaining_s:
            self._emit_event(
                "time_budget_finished",
                "剩余时间不足，不再追加下一轮巡检",
                remaining_time_s=round(remaining_time_s, 1),
                completed_cycles=cycle_index,
            )
            return False

        if cycle_index >= self.max_time_budget_cycles:
            self._emit_event(
                "time_budget_cycle_limit",
                "已达到限时巡检最大续巡轮数",
                remaining_time_s=round(remaining_time_s, 1),
                completed_cycles=cycle_index,
            )
            return False

        start_name = self._current_graph_node_name()
        next_segments = self._build_cycle_segments(start_name, target_specs)
        if not next_segments:
            self._emit_event(
                "time_budget_extend_failed",
                "无法为下一轮限时巡检生成拓扑路径",
                start_name=start_name,
            )
            return False

        with self._lock:
            if self._abort_flag or self._segments:
                return False
            self._segments = next_segments
            self._time_budget_cycle_index += 1
            new_cycle_index = self._time_budget_cycle_index

        self.say(
            f"⏱️ 限时巡检剩余约{remaining_time_s / 60.0:.1f}分钟，"
            f"开始第{new_cycle_index}轮动态续巡。"
        )
        self._emit_event(
            "time_budget_cycle_started",
            "根据剩余时间动态追加一轮巡检",
            cycle=new_cycle_index,
            start_name=start_name,
            segment_count=len(next_segments),
            remaining_time_s=round(remaining_time_s, 1),
        )
        return True

    def _build_cycle_segments(self, start_name: str, target_specs: List[Tuple[str, bool]]) -> List[Segment]:
        segments: List[Segment] = []
        current_name = start_name
        for target_name, photo_required in target_specs:
            if target_name not in self.graph.locations:
                continue
            route = self._plan_topology_route_to(target_name, start_name=current_name)
            waypoints = self._route_to_waypoints(
                route,
                final_stop_name=target_name,
                final_photo_required=photo_required,
            )
            coords = self.graph.get_location_coordinates(target_name)
            if coords is None:
                continue
            if not waypoints:
                waypoints = [Waypoint(target_name, coords[0], coords[1], True, photo_required)]
            segments.append(Segment(
                index=0,
                waypoints=waypoints,
                end_name=target_name,
                end_xy=(coords[0], coords[1]),
                kind='mission',
                photo_required=photo_required,
            ))
            current_name = target_name
        return segments

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
        if not seg.photo_required:
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

    def _maybe_preempt_for_energy(self):
        """Before starting the next mission segment, ensure it can still reach charge after that target."""
        if not self.enable_predictive_charging:
            return

        with self._lock:
            if self._returning_to_charge or not self._segments:
                return
            next_seg = self._segments[0]
            if next_seg.kind != 'mission':
                return
            battery_pct = self._battery_pct
            low_battery = self._low_battery
            current_segment = self._current_segment

        if low_battery:
            self._inject_return_charge_segment(reason="battery_below_threshold")
            return

        try:
            charge_name, _, _ = self.graph.get_charge_point()
        except Exception as exc:
            rospy.logwarn(f"[scheduler] cannot evaluate energy guard: {exc}")
            return

        start_name = current_segment.end_name if current_segment is not None else self._current_graph_node_name()
        route_to_next = self._plan_topology_route_to(next_seg.end_name, start_name=start_name)
        route_next_to_charge = self._plan_topology_route_to(charge_name, start_name=next_seg.end_name)
        distance_to_next = self._route_distance(route_to_next)
        distance_next_to_charge = self._route_distance(route_next_to_charge)
        decision = self.battery_policy.evaluate(battery_pct, distance_to_next, distance_next_to_charge)

        if not decision.should_charge:
            return

        self.say(
            "🔋 预测到达下一目标后剩余续航不足，先返航充电："
            f"可用约{decision.available_range_m:.1f}m，"
            f"需要约{decision.required_range_m:.1f}m"
        )
        self._emit_event(
            "energy_guard",
            "预测续航不足，先执行充电段",
            next_target=next_seg.end_name,
            battery_pct=round(battery_pct, 1),
            distance_to_next_m=round(distance_to_next, 1),
            distance_next_to_charge_m=round(distance_next_to_charge, 1),
            margin_m=round(decision.margin_m, 1),
        )
        self._inject_return_charge_segment(start_name=start_name, reason=decision.reason)

    def _route_distance(self, route: List[str]) -> float:
        if not route or len(route) < 2:
            return 0.0
        total = 0.0
        for index in range(len(route) - 1):
            total += self.graph.euclidean_distance(route[index], route[index + 1])
        return total

    def _inject_return_charge_segment(self, start_name: Optional[str] = None, reason: str = "low_battery"):
        """构造一个段：从当前位置（即上一段终点）→ 充电点。"""
        try:
            cname, cx, cy = self.graph.get_charge_point()
        except Exception as e:
            self.say(f"⚠️ 无法获取充电点: {e}；放弃自动返航。")
            return

        route = self._plan_topology_route_to(cname, start_name=start_name)
        waypoints = self._route_to_waypoints(route, final_stop_name=cname, final_photo_required=False)
        if not waypoints:
            waypoints = [Waypoint(cname, cx, cy, True, False)]

        with self._lock:
            if self._segments and self._segments[0].kind == 'return_charge':
                return
            charge_seg = Segment(
                index=0,  # 会在主循环 pop 时重排
                waypoints=waypoints,
                end_name=cname,
                end_xy=(cx, cy),
                kind='return_charge',
                photo_required=False,
            )
            # 插队到队首，剩余 mission 段保持顺序
            self._segments.insert(0, charge_seg)
            self._returning_to_charge = True
        route_text = " → ".join(route) if route else cname
        self.say(f"🔋 低电量触发返航：插入充电段 {route_text}")
        self._emit_event("return_charge_inserted", f"插入充电段 {route_text}", reason=reason)

    def _plan_topology_route_to(self, target_name: str, start_name: Optional[str] = None) -> List[str]:
        """用变电站拓扑图规划当前位置到目标点的全局节点序列。"""
        if target_name not in self.graph.locations:
            return []

        current_name = start_name or self._current_graph_node_name()
        if current_name not in self.graph.locations:
            current_name = target_name

        route = self.graph.dijkstra(current_name, target_name)
        if route:
            return route
        return [current_name, target_name] if current_name != target_name else [target_name]

    def _current_graph_node_name(self) -> str:
        cur = self.get_current_xy() or self._cur_xy_from_odom
        if cur is not None:
            try:
                return self.graph.find_nearest_node_name(cur[0], cur[1])
            except Exception:
                pass
        with self._lock:
            if self._current_segment is not None:
                return self._current_segment.end_name
        return "入口点"

    def _route_to_waypoints(
        self,
        route: List[str],
        final_stop_name: str,
        final_photo_required: bool = False,
    ) -> List[Waypoint]:
        if not route:
            return []
        waypoint_names = route[1:] if len(route) > 1 else route
        waypoints: List[Waypoint] = []
        for name in waypoint_names:
            coords = self.graph.get_location_coordinates(name)
            if coords is None:
                continue
            stop_required = name == final_stop_name
            waypoints.append(
                Waypoint(name, coords[0], coords[1], stop_required, stop_required and final_photo_required)
            )
        return waypoints

    def _repair_next_mission_after_charge(self, charge_name: str):
        """充电后把下一段 mission 改写为从充电点出发的拓扑路径。"""
        with self._lock:
            next_index = None
            next_seg = None
            for index, seg in enumerate(self._segments):
                if seg.kind == 'mission':
                    next_index = index
                    next_seg = seg
                    break

        if next_seg is None:
            return

        route = self._plan_topology_route_to(next_seg.end_name, start_name=charge_name)
        waypoints = self._route_to_waypoints(
            route,
            final_stop_name=next_seg.end_name,
            final_photo_required=next_seg.photo_required,
        )
        if not waypoints:
            return

        with self._lock:
            if next_index is not None and next_index < len(self._segments):
                current_seg = self._segments[next_index]
                if current_seg.kind == 'mission' and current_seg.end_name == next_seg.end_name:
                    self._segments[next_index] = current_seg._replace(waypoints=waypoints)
        self.say(f"🔁 充电后按拓扑恢复任务: {' → '.join(route)}")

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
            self._emit_event("battery_low", "收到低电量告警", battery_pct=round(self._battery_pct, 1))

    def _on_battery(self, msg):
        previous_low = False
        with self._lock:
            previous_low = self._battery_pct <= self.low_battery_threshold
            self._battery_pct = float(msg.percentage)
            self._battery_charging = bool(msg.charging)
            current_low = self._battery_pct <= self.low_battery_threshold
        if current_low and not previous_low:
            self._emit_event("battery_low", "电量低于阈值", battery_pct=round(self._battery_pct, 1))

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_xy_from_odom = (p.x, p.y)
