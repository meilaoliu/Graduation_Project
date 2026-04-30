# -*- coding: utf-8 -*-
"""
OSM 地图加载工具

负责从 OSM / 简化 OSM 文件中解析出变电站设备节点及其坐标或语义信息。
"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple


def _resolve_path(path: str) -> str:
    """将相对路径解析为相对于当前文件所在目录的绝对路径。"""
    if os.path.isabs(path):
        return path
    base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(base_dir, "..", path))


def load_locations_from_osm(osm_path: str) -> Dict[str, Tuple[float, float]]:
    """
    从完整 OSM 地图中加载设备节点坐标。

    要求每个设备节点至少包含：
        - name 标签：设备名称（与系统内部使用名称一致）
        - 经纬度属性：lat, lon

    Args:
        osm_path: OSM 文件路径，可以是绝对路径或相对于 nlp_commander 的相对路径。

    Returns:
        dict: {设备名称: (x, y)}，其中 (x, y) 以 OSM 的 (lon, lat) 简单映射为平面坐标。
    """
    locations, _ = load_graph_from_osm(osm_path)
    return locations


def load_graph_from_osm(osm_path: str) -> Tuple[Dict[str, Tuple[float, float]], List[Tuple[str, str]]]:
    """
    从完整 OSM 地图中加载设备节点坐标和拓扑边。

    节点通过 name 标签映射为系统内部名称；way 中相邻 nd 引用会被解释为
    可通行边。若地图暂未包含 way，返回的边列表为空，调用方可回退到内置拓扑。
    """
    full_path = _resolve_path(osm_path)
    if not os.path.exists(full_path):
        raise FileNotFoundError(f"OSM 地图文件不存在: {full_path}")

    tree = ET.parse(full_path)
    root = tree.getroot()

    locations: Dict[str, Tuple[float, float]] = {}
    node_id_to_name: Dict[str, str] = {}

    for node in root.findall("node"):
        node_id = node.get("id")
        lat = node.get("lat")
        lon = node.get("lon")
        if lat is None or lon is None:
            continue

        name = None
        for tag in node.findall("tag"):
            if tag.get("k") == "name":
                name = tag.get("v")
                break

        if not name:
            continue

        try:
            x = float(lon)
            y = float(lat)
            locations[name] = (x, y)
            if node_id is not None:
                node_id_to_name[node_id] = name
        except ValueError:
            continue

    edges: List[Tuple[str, str]] = []
    seen_edges = set()

    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd") if nd.get("ref")]
        for first_ref, second_ref in zip(refs, refs[1:]):
            first_name = node_id_to_name.get(first_ref)
            second_name = node_id_to_name.get(second_ref)
            if not first_name or not second_name or first_name == second_name:
                continue

            edge_key = tuple(sorted((first_name, second_name)))
            if edge_key in seen_edges:
                continue

            seen_edges.add(edge_key)
            edges.append((first_name, second_name))

    return locations, edges


def load_simplified_osm(osm_path: str) -> str:
    """
    读取简化后的 OSM 文件，作为 LLM 提示词中的纯文本地图上下文。

    Args:
        osm_path: 简化 OSM 文件路径（不包含经纬度与几何信息）。

    Returns:
        str: OSM XML 文本内容。
    """
    full_path = _resolve_path(osm_path)
    if not os.path.exists(full_path):
        raise FileNotFoundError(f"简化 OSM 地图文件不存在: {full_path}")

    with open(full_path, "r", encoding="utf-8") as f:
        return f.read()

