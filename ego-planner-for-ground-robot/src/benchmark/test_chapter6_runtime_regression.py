#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regression tests for Chapter 6 benchmark metrics and runtime routing."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NLP_DIR = ROOT / "src" / "nlp_commander"
BENCHMARK_DIR = Path(__file__).resolve().parent
if str(NLP_DIR) not in sys.path:
    sys.path.insert(0, str(NLP_DIR))
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

from utils.agent_runtime.runtime import TaskAgentRuntime  # noqa: E402
from utils.path_planner import PathPlanner  # noqa: E402
import chapter6_task_runner as chapter6  # noqa: E402
from substation_nlp_commander_node_v2 import SubstationNlpCommanderV2  # noqa: E402


class DummyGraph:
    def __init__(self):
        self.locations = {
            "入口点": {},
            "充电口": {},
            "低压配电室1": {},
        }

    def get_charge_point(self, name=None):
        return "充电口", 0.0, 0.0

    def validate_path(self, path):
        return bool(path) and all(node in self.locations for node in path)


class DummyPathPlanner:
    def __init__(self):
        self.graph = DummyGraph()

    def plan_path_to_single_target(self, start, target):
        if target not in self.graph.locations:
            return []
        if start == target:
            return [target]
        return [start, target]

    def plan_ordered_targets_path(self, start, targets, deduplicate=False):
        path = [start]
        for target in targets:
            if target not in self.graph.locations:
                return []
            if path[-1] != target:
                path.append(target)
        return path

    def plan_multi_target_path(self, start, targets):
        return self.plan_ordered_targets_path(start, targets)

    def get_path_distance(self, path):
        if not path:
            return 0.0
        return float(max(len(path) - 1, 0)) * 10.0

    def validate_path(self, path):
        return bool(path) and all(node in self.graph.locations for node in path)


def test_return_charge_progress_anomaly_enters_runtime():
    runtime = TaskAgentRuntime(
        path_planner=DummyPathPlanner(),
        handle_navigation_request=lambda *args, **kwargs: "ok",
        llm_callable=None,
        say=lambda *_: None,
        enabled=True,
    )
    event = {
        "event_type": "progress_anomaly",
        "message": "return_charge anomaly",
        "data": {"kind": "return_charge", "target": "充电口"},
    }

    assert runtime.should_handle_event(event) is True

    record = runtime.on_runtime_event(event, execute=False)
    assert record is not None
    assert runtime.last_run_result is not None
    assert runtime.last_run_result.success is True
    assert runtime.last_run_result.decision is not None
    assert runtime.last_run_result.decision.action == "retry_failed_target"
    assert runtime.decision_history[-1]["event"]["data"]["kind"] == "return_charge"


def test_chapter6_runtime_metrics_cover_agent_interventions():
    records = [
        chapter6.RunRecord(
            task_id="T16",
            task_type="长任务与返航恢复",
            run_index=1,
            command="完整巡检一遍，最后去充电",
            expected_targets=[],
            actual_targets=[],
            parse_ok=None,
            task_done=True,
            segment_ok=True,
            photo_ok=None,
            charge_ok=True,
            low_battery_seen=True,
            agent_intervened=True,
            agent_intervention_count=1,
            progress_anomaly_count=1,
            halt_on_anomaly=True,
            failure_reason="",
            duration_sec=12.0,
            init_battery=25.0,
        ),
        chapter6.RunRecord(
            task_id="T17",
            task_type="长任务与返航恢复",
            run_index=1,
            command="依次检查低压配电室 1、2、3 后返回充电",
            expected_targets=[],
            actual_targets=[],
            parse_ok=True,
            task_done=False,
            segment_ok=False,
            photo_ok=None,
            charge_ok=False,
            low_battery_seen=True,
            agent_intervened=False,
            agent_intervention_count=0,
            progress_anomaly_count=1,
            halt_on_anomaly=False,
            failure_reason="timeout",
            duration_sec=18.0,
            init_battery=20.0,
        ),
        chapter6.RunRecord(
            task_id="T14",
            task_type="多轮对话",
            run_index=1,
            command="返回入口点",
            expected_targets=["入口点"],
            actual_targets=["入口点"],
            parse_ok=True,
            task_done=True,
            segment_ok=True,
            photo_ok=None,
            charge_ok=None,
            low_battery_seen=False,
            agent_intervened=True,
            agent_intervention_count=1,
            progress_anomaly_count=0,
            halt_on_anomaly=False,
            failure_reason="",
            duration_sec=8.0,
            init_battery=100.0,
        ),
    ]

    summary = chapter6.aggregate_by_type(records)

    long_task = summary["长任务与返航恢复"]
    assert long_task["count"] == 2
    assert long_task["air"] == 50.0
    assert long_task["airr"] == 100.0
    assert long_task["par"] == 100.0
    assert long_task["har"] == 50.0
    assert long_task["anomaly_intervention_rate"] == 50.0
    assert long_task["agent_task_total"] == 1
    assert long_task["agent_done_total"] == 1
    assert long_task["anomaly_task_total"] == 2
    assert long_task["anomaly_halt_total"] == 1

    dialogue = summary["多轮对话"]
    assert dialogue["count"] == 1
    assert dialogue["air"] == 100.0
    assert dialogue["airr"] == 100.0
    assert dialogue["par"] == 0.0
    assert dialogue["har"] == 0.0


def test_multi_target_payload_labeled_single_target_still_runs_all_targets():
    commander = object.__new__(SubstationNlpCommanderV2)
    commander.path_planner = PathPlanner()

    analysis = commander._analyze_waypoint_sequence(
        ["低压配电室1", "变压器区2", "1SVG无功补偿区"],
        "按指定顺序依次巡检低压配电室1、变压器区2和1SVG无功补偿区。",
        "single_target",
    )

    assert analysis == {
        "type": "custom_path",
        "targets": ["低压配电室1", "变压器区2", "1SVG无功补偿区"],
    }


def test_parse_match_allows_required_charge_as_auxiliary_target():
    actual = [
        "充电口",
        "变压器区1",
        "变压器区2",
        "变压器区3",
        "高压配电区巡检点3",
        "高压配电区巡检点2",
        "高压配电区巡检点1",
    ]
    expected = [
        "高压配电区巡检点1",
        "高压配电区巡检点2",
        "高压配电区巡检点3",
        "变压器区1",
        "变压器区2",
        "变压器区3",
    ]

    assert chapter6.targets_match(actual, expected, order_required=False, auxiliary_targets=["充电口"])
    assert chapter6.targets_match(["充电口"], ["充电口"], order_required=False, auxiliary_targets=["充电口"])


def test_overall_ipa_uses_evaluable_parse_count_not_category_average():
    records = [
        chapter6.RunRecord(
            task_id="A1",
            task_type="类别A",
            run_index=1,
            command="a",
            expected_targets=["入口点"],
            actual_targets=["入口点"],
            parse_ok=True,
            task_done=True,
            segment_ok=True,
            photo_ok=None,
            charge_ok=None,
            low_battery_seen=False,
            agent_intervened=False,
            agent_intervention_count=0,
            progress_anomaly_count=0,
            halt_on_anomaly=False,
            failure_reason="",
            duration_sec=1.0,
            init_battery=100.0,
        ),
        chapter6.RunRecord(
            task_id="A2",
            task_type="类别A",
            run_index=1,
            command="full",
            expected_targets=[],
            actual_targets=[],
            parse_ok=None,
            task_done=True,
            segment_ok=True,
            photo_ok=None,
            charge_ok=None,
            low_battery_seen=False,
            agent_intervened=False,
            agent_intervention_count=0,
            progress_anomaly_count=0,
            halt_on_anomaly=False,
            failure_reason="",
            duration_sec=1.0,
            init_battery=100.0,
        ),
        chapter6.RunRecord(
            task_id="B1",
            task_type="类别B",
            run_index=1,
            command="b",
            expected_targets=["入口点"],
            actual_targets=[],
            parse_ok=False,
            task_done=True,
            segment_ok=True,
            photo_ok=None,
            charge_ok=None,
            low_battery_seen=False,
            agent_intervened=False,
            agent_intervention_count=0,
            progress_anomaly_count=0,
            halt_on_anomaly=False,
            failure_reason="",
            duration_sec=1.0,
            init_battery=100.0,
        ),
    ]

    summary = chapter6.aggregate_by_type(records)
    overall = chapter6.overall_rates(summary, ["类别A", "类别B"])

    assert overall["count"] == 3
    assert overall["ipa"] == 50.0


def main():
    test_return_charge_progress_anomaly_enters_runtime()
    test_chapter6_runtime_metrics_cover_agent_interventions()
    test_multi_target_payload_labeled_single_target_still_runs_all_targets()
    test_parse_match_allows_required_charge_as_auxiliary_target()
    test_overall_ipa_uses_evaluable_parse_count_not_category_average()
    print("Chapter 6 regression tests passed")


if __name__ == "__main__":
    main()
