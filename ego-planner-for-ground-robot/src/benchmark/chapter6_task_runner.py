#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第六章端到端巡检实验任务运行与统计脚本。

一键跑完全部任务（推荐）：
  ./run_chapter6_experiments.sh
  ./run_chapter6_experiments.sh --with-launch    # 自动启动 inspection_full 后连续跑完
  python3 chapter6_task_runner.py --suite --fresh

单条/分类调试：
  python3 chapter6_task_runner.py --task-id T01
  python3 chapter6_task_runner.py --type 低电量返航
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

try:
    import yaml
except ImportError:
    yaml = None

try:
    import rospy
    from std_msgs.msg import String, UInt32, Bool
    from std_srvs.srv import Trigger
except ImportError:
    rospy = None

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_YAML = SCRIPT_DIR / "chapter6_tasks.yaml"
DEFAULT_CSV = SCRIPT_DIR / "chapter6_results.csv"
DEFAULT_LOG_DIR = SCRIPT_DIR / "chapter6_log"
DEFAULT_OBSIDIAN = Path.home() / "文档/Obsidian Vault/毕业设计/13-2026-05-21 第六章实验任务集与记录表.md"

SEGMENT_RE = re.compile(r"任务分段[（(]\d+段[）)][:：]\s*(.+)")
TARGET_RE = re.compile(r"→\s*([^|]+)")
FAIL_RE = re.compile(r"任务终止|无法解析|指令处理出错|执行失败/超时|充电失败/未达阈值|停止剩余任务")
SUCCESS_RE = re.compile(r"全部段执行完毕|任务完成")
CHARGE_RE = re.compile(r"开始充电|充电完成|返航")
LOW_BATT_RE = re.compile(r"低电量触发返航")
PHOTO_RE = re.compile(r"已在\s*(.+?)\s*拍照")


def _norm_name(name: str) -> str:
    return re.sub(r"\s+", "", name.strip().lower())


def load_tasks(yaml_path: Path) -> List[Dict[str, Any]]:
    if yaml is None:
        raise RuntimeError("PyYAML not installed: pip install pyyaml")
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return list(data.get("tasks") or [])


def parse_segment_targets(text: str) -> List[str]:
    m = SEGMENT_RE.search(text)
    if not m:
        return []
    body = m.group(1)
    return [t.strip() for t in TARGET_RE.findall(body)]


def is_terminal_failure(text: str) -> bool:
    """只把最终失败视为失败；Agent 重试过程中的“失败/停止”不应提前终止统计。"""
    if "任务被外部停止" in text:
        return False
    if "之前的失败" in text or "Agent Runtime 处理事件" in text:
        return False
    return bool(FAIL_RE.search(text))


def targets_match(actual: Sequence[str], expected: Sequence[str], order_required: bool) -> Optional[bool]:
    if not expected:
        return None
    act = [_norm_name(x) for x in actual]
    exp = [_norm_name(x) for x in expected]
    if not act:
        return False
    if order_required:
        return act == exp
    return set(act) == set(exp)


@dataclass
class RunRecord:
    task_id: str
    task_type: str
    run_index: int
    command: str
    expected_targets: List[str]
    actual_targets: List[str]
    parse_ok: Optional[bool]
    task_done: bool
    segment_ok: bool
    photo_ok: Optional[bool]
    charge_ok: Optional[bool]
    low_battery_seen: bool
    failure_reason: str
    duration_sec: float
    init_battery: float
    events: List[str] = field(default_factory=list)


def wait_for_ros_stack(timeout_sec: float = 180.0) -> bool:
    if rospy is None:
        return False
    if not rospy.core.is_initialized():
        rospy.init_node("chapter6_stack_wait", anonymous=True)
    deadline = time.time() + timeout_sec
    required = {"/chat_in", "/chat_out"}
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            names = {topic for topic, _ in rospy.get_published_topics()}
        except Exception:
            names = set()
        if required.issubset(names):
            rospy.loginfo("[chapter6] ROS stack ready: %s", ", ".join(sorted(required)))
            rospy.sleep(2.0)
            return True
        rospy.sleep(2.0)
    return False


def build_suite_execution_plan(tasks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """按 YAML 顺序展开；上下文任务若前置任务尚未排入队列，则先插入完整前置任务。"""
    by_id = {t["id"]: t for t in tasks}
    scheduled = set()
    plan: List[Dict[str, Any]] = []
    for task in tasks:
        prereq_id = task.get("prerequisite_task")
        if prereq_id and prereq_id not in scheduled and prereq_id in by_id:
            plan.append(by_id[prereq_id])
            scheduled.add(prereq_id)
        if task["id"] not in scheduled:
            plan.append(task)
            scheduled.add(task["id"])
    return plan


class Chapter6TaskRunner:
    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self._ros_ready = False
        self.chat_pub = None
        self.chat_messages: List[str] = []
        self.photos: List[str] = []
        self.segments_done = 0
        self.segments_failed = 0
        self.low_battery = False
        self.charge_started = False
        self.charge_done = False
        self.charge_started_count = 0
        self.charge_done_count = 0
        self.task_success = False
        self.success_count = 0
        self.task_failed = False
        self.failure_reason = ""
        self.actual_targets: List[str] = []

    def reset_run_state(self):
        self.chat_messages.clear()
        self.photos.clear()
        self.segments_done = 0
        self.segments_failed = 0
        self.low_battery = False
        self.charge_started = False
        self.charge_done = False
        self.charge_started_count = 0
        self.charge_done_count = 0
        self.task_success = False
        self.success_count = 0
        self.task_failed = False
        self.failure_reason = ""
        self.actual_targets = []

    def _append_targets(self, targets: Sequence[str]):
        existing = {_norm_name(t) for t in self.actual_targets}
        for target in targets:
            target = target.strip()
            if not target:
                continue
            key = _norm_name(target)
            if key not in existing:
                self.actual_targets.append(target)
                existing.add(key)

    def _chat_cb(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        self.chat_messages.append(text)
        if SEGMENT_RE.search(text):
            self._append_targets(parse_segment_targets(text))
        if "终点: 充电口" in text or "停留点: 充电口" in text or "已启动返航充电任务" in text:
            self._append_targets(["充电口"])
        if PHOTO_RE.search(text):
            m = PHOTO_RE.search(text)
            if m:
                self.photos.append(m.group(1).strip())
        if LOW_BATT_RE.search(text):
            self.low_battery = True
        if "开始充电" in text:
            self.charge_started = True
            self.charge_started_count += 1
        if "充电完成" in text:
            self.charge_done = True
            self.charge_done_count += 1
        if SUCCESS_RE.search(text):
            self.task_success = True
            self.success_count += 1
        if is_terminal_failure(text):
            self.task_failed = True
            if not self.failure_reason:
                self.failure_reason = text[:120]

    def _segment_done_cb(self, _msg: UInt32):
        self.segments_done += 1

    def setup_ros(self):
        if self.dry_run or self._ros_ready or rospy is None:
            return
        self.chat_pub = rospy.Publisher("/chat_in", String, queue_size=10)
        rospy.Subscriber("/chat_out", String, self._chat_cb, queue_size=100)
        rospy.Subscriber("/segment_done", UInt32, self._segment_done_cb, queue_size=20)
        rospy.sleep(0.5)
        self._ros_ready = True

    def prepare_battery(self, init_battery: float):
        if self.dry_run:
            print(f"[dry-run] prepare_battery init={init_battery}%")
            return
        if rospy is None:
            return
        pct = float(init_battery)
        for param_name in ("/battery_monitor/reset_to_percentage", "/battery_monitor_node/reset_to_percentage"):
            try:
                rospy.set_param(param_name, pct)
            except Exception:
                pass
        try:
            rospy.wait_for_service("/reset_battery", timeout=8.0)
            reset = rospy.ServiceProxy("/reset_battery", Trigger)
            resp = reset()
            rospy.loginfo("[chapter6] battery reset: %s", resp.message)
            rospy.sleep(1.0)
        except Exception as e:
            rospy.logwarn("[chapter6] /reset_battery unavailable: %s", e)

    def send_stop(self):
        if self.dry_run or not self.chat_pub:
            return
        self.chat_pub.publish(String(data="停止"))
        rospy.loginfo("[chapter6] sent stop between tasks")
        rospy.sleep(4.0)

    def run_task(
        self,
        task: Dict[str, Any],
        run_index: int,
        timeout_sec: float,
        *,
        suite_mode: bool = False,
    ) -> RunRecord:
        self.reset_run_state()
        cmd = task["command"]
        expected = list(task.get("expected_targets") or [])
        order_required = bool(task.get("order_required", False))
        init_battery = float(task.get("init_battery", 100))
        require_photo = bool(task.get("require_photo", False))
        require_charge = bool(task.get("require_charge", False))
        min_charge_completions = int(task.get("min_charge_completions", 1 if require_charge else 0))
        min_task_completions = int(task.get("min_task_completions", 1))

        if self.dry_run:
            print(f"[dry-run] {task['id']} run={run_index}: {cmd}")
            return RunRecord(
                task_id=task["id"],
                task_type=task["type"],
                run_index=run_index,
                command=cmd,
                expected_targets=expected,
                actual_targets=[],
                parse_ok=None,
                task_done=False,
                segment_ok=False,
                photo_ok=None,
                charge_ok=None,
                low_battery_seen=False,
                failure_reason="dry-run",
                duration_sec=0.0,
                init_battery=init_battery,
            )

        self.setup_ros()
        self.prepare_battery(init_battery)

        if not suite_mode:
            prereq = task.get("prerequisite_command")
            if prereq and self.chat_pub:
                rospy.sleep(1.0)
                self.chat_pub.publish(String(data=str(prereq)))
                rospy.loginfo("[chapter6] prerequisite snippet: %s", prereq)
                rospy.sleep(8.0)

        rospy.sleep(0.5)
        t0 = time.time()
        self.chat_pub.publish(String(data=str(cmd)))
        rospy.loginfo("[chapter6] sent: %s", cmd)

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.task_failed:
                break
            if require_charge:
                if (
                    self.charge_done_count >= min_charge_completions
                    and self.success_count >= min_task_completions
                ):
                    break
            elif self.task_success:
                break
            if time.time() - t0 > timeout_sec:
                self.task_failed = True
                self.failure_reason = "timeout"
                break
            rate.sleep()

        duration = time.time() - t0
        parse_ok = targets_match(self.actual_targets, expected, order_required)
        segment_ok = not self.task_failed or self.segments_done > 0
        if self.task_failed and self.segments_done == 0:
            segment_ok = False

        photo_ok = None
        if require_photo:
            photo_ok = len(self.photos) > 0

        charge_ok = None
        if require_charge:
            charge_ok = self.charge_done_count >= min_charge_completions

        if require_charge:
            task_done = (
                self.success_count >= min_task_completions
                and self.charge_done_count >= min_charge_completions
                and not self.task_failed
            )
        else:
            task_done = self.task_success and not self.task_failed

        return RunRecord(
            task_id=task["id"],
            task_type=task["type"],
            run_index=run_index,
            command=cmd,
            expected_targets=expected,
            actual_targets=list(self.actual_targets),
            parse_ok=parse_ok,
            task_done=task_done,
            segment_ok=segment_ok,
            photo_ok=photo_ok,
            charge_ok=charge_ok,
            low_battery_seen=self.low_battery,
            failure_reason=self.failure_reason,
            duration_sec=duration,
            init_battery=init_battery,
            events=self.chat_messages[-30:],
        )


def save_csv(records: List[RunRecord], path: Path, *, fresh: bool = False):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "task_id", "task_type", "run_index", "command",
        "expected_targets", "actual_targets",
        "parse_ok", "task_done", "segment_ok",
        "photo_ok", "charge_ok", "low_battery_seen",
        "failure_reason", "duration_sec", "init_battery",
    ]
    write_header = fresh or not path.exists()
    mode = "w" if fresh else "a"
    with path.open(mode, encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            w.writeheader()
        for r in records:
            row = asdict(r)
            row["expected_targets"] = "|".join(r.expected_targets)
            row["actual_targets"] = "|".join(r.actual_targets)
            del row["events"]
            w.writerow(row)


def save_json_log(record: RunRecord, log_dir: Path):
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"{record.task_id}_run{record.run_index}_{ts}.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(asdict(record), f, ensure_ascii=False, indent=2)


def aggregate_by_type(records: List[RunRecord]) -> Dict[str, Dict[str, Any]]:
    buckets: Dict[str, List[RunRecord]] = defaultdict(list)
    for r in records:
        buckets[r.task_type].append(r)

    summary = {}
    for ttype, rows in buckets.items():
        n = len(rows)
        parse_vals = [r.parse_ok for r in rows if r.parse_ok is not None]
        done_vals = [r.task_done for r in rows]
        seg_vals = [r.segment_ok for r in rows]
        event_vals = []
        for r in rows:
            ok = True
            if r.photo_ok is False or r.charge_ok is False:
                ok = False
            event_vals.append(ok)

        def pct(vals):
            return 100.0 * sum(1 for v in vals if v) / len(vals) if vals else 0.0

        summary[ttype] = {
            "count": n,
            "ipa": pct(parse_vals),
            "tcr": pct(done_vals),
            "snsr": pct(seg_vals),
            "event": pct(event_vals),
        }
    return summary


def print_summary_table(summary: Dict[str, Dict[str, Any]]):
    order = ["单目标巡检", "多目标巡检", "区域模糊巡检", "上下文相关指令", "低电量返航"]
    print("\n" + "=" * 72)
    print("  Chapter 6 experiment summary")
    print("=" * 72)
    print(f"{'类型':<14} {'N':>4} {'IPA':>8} {'TCR':>8} {'SNSR':>8} {'事件':>8}")
    print("-" * 72)
    totals = defaultdict(list)
    for ttype in order:
        if ttype not in summary:
            continue
        s = summary[ttype]
        print(f"{ttype:<14} {s['count']:>4} {s['ipa']:>7.1f}% {s['tcr']:>7.1f}% "
              f"{s['snsr']:>7.1f}% {s['event']:>7.1f}%")
        for k in ("ipa", "tcr", "snsr", "event"):
            totals[k].extend([s[k]] * s["count"])
    if totals["ipa"]:
        n_all = sum(summary[t]["count"] for t in order if t in summary)
        print("-" * 72)
        print(f"{'合计':<14} {n_all:>4} "
              f"{sum(totals['ipa'])/len(totals['ipa']):>7.1f}% "
              f"{sum(totals['tcr'])/len(totals['tcr']):>7.1f}% "
              f"{sum(totals['snsr'])/len(totals['snsr']):>7.1f}% "
              f"{sum(totals['event'])/len(totals['event']):>7.1f}%")
    print("=" * 72)


def latex_table_rows(summary: Dict[str, Dict[str, Any]]) -> str:
    order = ["单目标巡检", "多目标巡检", "区域模糊巡检", "上下文相关指令", "低电量返航"]
    lines = []
    for ttype in order:
        if ttype not in summary:
            continue
        s = summary[ttype]
        lines.append(
            f"        {ttype} & {s['count']} & {s['ipa']:.1f} & {s['tcr']:.1f} & "
            f"{s['snsr']:.1f} & {s['event']:.1f} \\\\"
        )
    if lines:
        n_all = sum(summary[t]["count"] for t in order if t in summary)
        ipa = sum(summary[t]["ipa"] * summary[t]["count"] for t in order if t in summary) / max(n_all, 1)
        tcr = sum(summary[t]["tcr"] * summary[t]["count"] for t in order if t in summary) / max(n_all, 1)
        snsr = sum(summary[t]["snsr"] * summary[t]["count"] for t in order if t in summary) / max(n_all, 1)
        ev = sum(summary[t]["event"] * summary[t]["count"] for t in order if t in summary) / max(n_all, 1)
        lines.append(
            f"        合计 & {n_all} & {ipa:.1f} & {tcr:.1f} & {snsr:.1f} & {ev:.1f} \\\\"
        )
    return "\n".join(lines)


def append_obsidian_summary(summary: Dict[str, Dict[str, Any]], obsidian_path: Path):
    if not obsidian_path.exists():
        return
    ts = datetime.now().strftime("%Y-%m-%d %H:%M")
    block = [
        "",
        f"## 自动汇总 ({ts})",
        "",
        "| 任务类型 | 任务数 | IPA (%) | TCR (%) | SNSR (%) | 事件正确率 (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    order = ["单目标巡检", "多目标巡检", "区域模糊巡检", "上下文相关指令", "低电量返航"]
    for ttype in order:
        if ttype not in summary:
            continue
        s = summary[ttype]
        block.append(
            f"| {ttype} | {s['count']} | {s['ipa']:.1f} | {s['tcr']:.1f} | "
            f"{s['snsr']:.1f} | {s['event']:.1f} |"
        )
    block.append("")
    block.append("```latex")
    block.append("% tab:ch6_results_summary")
    block.append(latex_table_rows(summary))
    block.append("```")
    block.append("")
    with obsidian_path.open("a", encoding="utf-8") as f:
        f.write("\n".join(block))


def parse_args():
    p = argparse.ArgumentParser(description="Chapter 6 end-to-end inspection task runner")
    p.add_argument("--yaml", type=str, default=str(DEFAULT_YAML))
    p.add_argument("--csv", type=str, default=str(DEFAULT_CSV))
    p.add_argument("--log-dir", type=str, default=str(DEFAULT_LOG_DIR))
    p.add_argument("--obsidian", type=str, default=str(DEFAULT_OBSIDIAN))
    p.add_argument("--task-id", type=str, default="")
    p.add_argument("--type", type=str, default="")
    p.add_argument("--all", action="store_true", help="同 --suite")
    p.add_argument("--suite", action="store_true", help="一键连续执行 YAML 中全部任务")
    p.add_argument("--fresh", action="store_true", help="覆盖写入 CSV，不追加旧结果")
    p.add_argument("--wait-stack", action="store_true", help="仅等待 ROS 栈就绪后退出")
    p.add_argument("--stack-timeout", type=float, default=180.0)
    p.add_argument("--num-runs", type=int, default=1)
    p.add_argument("--inter-task-sleep", type=float, default=5.0, help="任务间隔秒数")
    p.add_argument("--timeout", type=float, default=0.0, help="override per-task timeout")
    p.add_argument("--init-battery", type=float, default=-1.0)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--no-csv", action="store_true")
    p.add_argument("--no-obsidian", action="store_true")
    return p.parse_args()


def filter_tasks(tasks: List[Dict], args) -> List[Dict]:
    if args.suite or args.all:
        return tasks
    if args.task_id:
        tasks = [t for t in tasks if t["id"] == args.task_id]
    if args.type:
        tasks = [t for t in tasks if t["type"] == args.type]
    if not args.task_id and not args.type:
        print("请指定 --suite / --all，或 --task-id / --type")
        sys.exit(1)
    return tasks


def main():
    args = parse_args()

    if args.wait_stack:
        if rospy is None:
            print("rospy not available.")
            sys.exit(1)
        ok = wait_for_ros_stack(args.stack_timeout)
        sys.exit(0 if ok else 1)

    tasks = load_tasks(Path(args.yaml))
    tasks = filter_tasks(tasks, args)
    if not tasks:
        print("No tasks matched.")
        sys.exit(1)

    suite_mode = bool(args.suite or args.all)
    if suite_mode:
        tasks = build_suite_execution_plan(tasks)
        print(f"[chapter6] suite 执行计划共 {len(tasks)} 步（含上下文前置任务自动插入）")
        for i, t in enumerate(tasks, 1):
            print(f"  {i:02d}. {t['id']} [{t['type']}] {t['command']}")

    if args.init_battery >= 0:
        for t in tasks:
            t["init_battery"] = args.init_battery

    runner = Chapter6TaskRunner(dry_run=args.dry_run)
    all_records: List[RunRecord] = []

    if not args.dry_run:
        if rospy is None:
            print("rospy not available. Source ROS workspace first.")
            sys.exit(1)
        rospy.init_node("chapter6_task_runner", anonymous=True)
        if not wait_for_ros_stack(min(args.stack_timeout, 30.0)):
            print("ROS 栈未就绪：请先 roslaunch inspection_dashboard inspection_full.launch")
            sys.exit(1)
        runner.setup_ros()

    total_steps = len(tasks) * max(1, args.num_runs)
    step_idx = 0
    for task in tasks:
        timeout = float(args.timeout) if args.timeout > 0 else float(task.get("timeout_sec", 300))
        for run_i in range(1, args.num_runs + 1):
            step_idx += 1
            print(f"\n>>> [{step_idx}/{total_steps}] {task['id']} run {run_i}/{args.num_runs}")
            if suite_mode and not args.dry_run and step_idx > 1:
                runner.send_stop()
            rec = runner.run_task(task, run_i, timeout, suite_mode=suite_mode)
            all_records.append(rec)
            if not args.dry_run:
                save_json_log(rec, Path(args.log_dir))
            print(
                f"[{rec.task_id} run{run_i}] parse={rec.parse_ok} done={rec.task_done} "
                f"targets={rec.actual_targets} reason={rec.failure_reason or '-'}"
            )
            if not args.dry_run:
                rospy.sleep(max(1.0, args.inter_task_sleep))

    if not args.no_csv and not args.dry_run:
        save_csv(all_records, Path(args.csv), fresh=args.fresh)

    summary = aggregate_by_type(all_records)
    print_summary_table(summary)
    print("\nLaTeX rows for tab:ch6_results_summary:\n")
    print(latex_table_rows(summary))

    if not args.no_obsidian and not args.dry_run:
        append_obsidian_summary(summary, Path(args.obsidian))


if __name__ == "__main__":
    main()
