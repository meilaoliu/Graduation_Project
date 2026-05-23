#!/usr/bin/env python3
"""Regression checks for semantic path planning used by chapter 6 tasks."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
NLP_DIR = PROJECT_ROOT / "src" / "nlp_commander"
sys.path.insert(0, str(NLP_DIR))

from utils.path_planner import PathPlanner  # noqa: E402


def _contains_window(path, window):
    return any(path[i:i + len(window)] == window for i in range(len(path) - len(window) + 1))


def test_full_inspection_from_mid_tour_does_not_bounce_through_entry():
    planner = PathPlanner()

    path = planner.plan_full_inspection("35kv配电箱2")

    assert path[0] == "35kv配电箱2"
    assert not _contains_window(path, ["插值点1", "入口点", "插值点1"])
    assert path.count("35kv配电箱2") == 1
    for target in [
        "低压配电室1",
        "低压配电室2",
        "低压配电室3",
        "变压器区1",
        "变压器区2",
        "变压器区3",
        "高压配电区巡检点1",
        "高压配电区巡检点2",
        "高压配电区巡检点3",
        "3SVG无功补偿区",
        "2SVG无功补偿区",
        "1SVG无功补偿区",
        "35kv配电箱1",
        "35kv配电箱3",
    ]:
        assert target in path


def test_single_target_route_to_low_voltage_room_keeps_direct_graph_route():
    planner = PathPlanner()

    path = planner.plan_path_to_single_target("35kv配电箱2", "低压配电室1")

    assert path == ["35kv配电箱2", "35kv配电箱3", "插值点1", "低压配电室1"]
