# -*- coding: utf-8 -*-
"""
OSM 简化工具

将包含经纬度与几何连通信息的完整 OSM 地图，
简化为仅保留语义与层级关系的文本形式，以供大语言模型作为上下文输入。
"""

import os
import xml.etree.ElementTree as ET
from typing import Optional


def simplify_osm(input_path: str, output_path: Optional[str] = None) -> str:
    """
    将完整 OSM 文件简化为仅包含语义标签的版本：
        - 删除所有 node 的 lat / lon 属性
        - 删除 way 中的 nd（几何顶点引用）
        - 保留 name、area、device_type、osmAG:parent 等语义相关标签

    Args:
        input_path: 原始 OSM 文件路径（绝对路径或相对路径）
        output_path: 简化后 OSM 文件输出路径；若为 None，则只返回字符串，不落盘

    Returns:
        str: 简化后的 OSM XML 文本
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    if not os.path.isabs(input_path):
        input_path = os.path.normpath(os.path.join(base_dir, "..", input_path))

    tree = ET.parse(input_path)
    root = tree.getroot()

    # 删除 node 的几何坐标
    for node in root.findall("node"):
        for attr in ["lat", "lon"]:
            if attr in node.attrib:
                node.attrib.pop(attr)

    # 删除 way 中的 nd 引用，仅保留语义标签
    for way in root.findall("way"):
        for nd in list(way.findall("nd")):
            way.remove(nd)

    # 可选：删除与几何相关的标签，如 height / indoor
    for tag in root.findall(".//tag"):
        k = tag.get("k", "")
        if k in {"height", "indoor"}:
            parent = tag.getparent() if hasattr(tag, "getparent") else None
            # 在标准 ElementTree 中没有 getparent，这里退而求其次：不删除父节点，仅忽略这些标签
            continue

    simplified_xml = ET.tostring(root, encoding="unicode", method="xml")

    if output_path:
        if not os.path.isabs(output_path):
            output_path = os.path.normpath(os.path.join(base_dir, "..", output_path))
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(simplified_xml)

    return simplified_xml

