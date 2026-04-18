# -*- coding: utf-8 -*-
"""
OSM 地图加载工具

负责从 OSM / 简化 OSM 文件中解析出变电站设备节点及其坐标或语义信息。
"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, Tuple


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
    full_path = _resolve_path(osm_path)
    if not os.path.exists(full_path):
        raise FileNotFoundError(f"OSM 地图文件不存在: {full_path}")

    tree = ET.parse(full_path)
    root = tree.getroot()

    locations: Dict[str, Tuple[float, float]] = {}

    for node in root.findall("node"):
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
        except ValueError:
            # 坐标格式不正确时跳过该节点
            continue

    return locations


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

