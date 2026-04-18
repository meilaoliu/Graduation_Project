"""
LLM 工具模块

负责与大语言模型的交互。
当前实现采用基于 CoT 与 JSON 结构化输出的语义解析方案，
大模型读取简化后的 OSM 拓扑语义地图，
输出任务类型与目标设备集合等高层语义信息。
"""

import os
import json
from typing import Dict, List, Optional, Any

from openai import OpenAI

from .osm_map_loader import load_simplified_osm

try:
    import rospy
except ImportError:
    # Mock rospy for testing without ROS
    class MockRospy:
        def logerr(self, msg): print(f"ERROR: {msg}")
        def loginfo(self, msg): print(f"INFO: {msg}")
        def logwarn(self, msg): print(f"WARN: {msg}")
    rospy = MockRospy()


class LLMClient:
    """大语言模型客户端"""

    def __init__(self, api_key: str = None, base_url: str = None):
        # 清理代理环境变量
        for proxy_var in [
            "http_proxy",
            "https_proxy",
            "all_proxy",
            "HTTP_PROXY",
            "HTTPS_PROXY",
            "ALL_PROXY",
        ]:
            os.environ.pop(proxy_var, None)

        # 初始化客户端（DashScope OpenAI 兼容接口）
        self.client = OpenAI(
            api_key=api_key or "sk-905de984fe624c8d91db26b4f081a676",
            base_url=base_url or "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )

        # 简化版 OSM 地图路径（相对于 nlp_commander 目录）
        self.simplified_osm_path = "maps/simplified_substation.osm"

    def create_system_prompt(
        self, current_pos_info: str, available_locations: List[str]
    ) -> str:
        """
        创建系统提示词。

        将简化后的 OSM 拓扑语义地图作为文本上下文注入，
        要求模型按照 CoT 链条进行推理，并以 JSON 形式输出任务类型与目标设备集合。
        """
        try:
            osm_text = load_simplified_osm(self.simplified_osm_path)
        except Exception as e:
            rospy.logwarn(f"加载简化 OSM 地图失败，将不提供地图上下文: {e}")
            osm_text = ""

        devices_str = ", ".join(available_locations)

        system_prompt = f"""
你是一个面向变电站巡检机器人的智能任务规划助手。
你的职责是：阅读变电站的拓扑语义地图与用户的自然语言指令，
推理出应该巡检哪些设备、属于哪一类巡检任务，并给出结构化的 JSON 输出。

【地图说明（简化 OSM 文本，仅保留语义标签）】
下面是一份经过简化处理的变电站拓扑语义地图，采用 OpenStreetMap XML 格式表示。
其中：
- 每个 <node> 代表一个设备或中间航点，包含：
  - name: 设备名称（与你需要输出的名称完全一致）
  - area: 所属区域（east/center/west）
  - device_type: 设备类型（如 entry、low_voltage_room、hv_switchgear、transformer、svg、mv_switchgear、waypoint）

请仔细阅读该地图文本，并在推理时充分利用其中的语义和区域信息。

<地图开始>
{osm_text}
<地图结束>

【当前机器人状态】
- 当前位置信息: {current_pos_info}
- 系统已知的所有可用设备点名称: {devices_str}

【你的推理任务】
给定一条中文巡检指令，你需要按照如下思维链进行推理：
1. 意图分析：判断用户想做什么类型的巡检（例如：单个设备巡检、某个功能区域的巡检、完整变电站巡检、或按照指定顺序访问多个设备）。
2. 目标设备检索：在上述地图中检索与指令最相关的设备节点列表，并给出排序理由（例如：按照空间分布、编号顺序或风险优先级）。
3. 任务类型判定：在 {{single_target, area_inspection, full_inspection, custom_path}} 四类任务中选择一种最合适的类型。

【输出格式要求】
你必须严格输出一个 JSON 对象（不要包含任何多余的文本），格式为：
{{
  "reasoning": "用中文简要说明你的思考过程，包括意图分析和设备选择依据。",
  "task_type": "single_target / area_inspection / full_inspection / custom_path 之一",
  "target_devices": [
    {{"name": "设备名称1", "priority": 1}},
    {{"name": "设备名称2", "priority": 2}}
  ],
  "task_description": "一句话概括本次巡检任务的目的和范围"
}}

其中：
- target_devices 中的 name 必须来自地图中的设备名称，且尽量与用户指令语义对齐。
- priority 为正整数，数值越小说明优先级越高。
- 当用户要求完整巡检时，可以将所有设备点都列入 target_devices，并按合理顺序排序。
- 若用户指令无法在地图中找到对应设备，请在 reasoning 中解释，并根据地图信息给出最接近的备选设备列表。

当前对话将只包含一条用户指令，因此你不需要关心多轮对话，只需专注于本次指令的语义解析与规划。
"""
        return system_prompt

    def process_inspection_command(
        self,
        command: str,
        current_pos_info: str,
        available_locations: List[str],
    ) -> Dict[str, Any]:
        """
        处理巡检指令。

        Args:
            command: 用户自然语言指令
            current_pos_info: 当前位置信息
            available_locations: 可用设备点名称列表

        Returns:
            dict: 若成功，则形如 {"success": True, "parsed": {...}}，
                  其中 parsed 为大模型输出的 JSON 对象。
        """
        system_prompt = self.create_system_prompt(current_pos_info, available_locations)
        user_message = f"用户巡检指令: {command}"

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message},
        ]

        try:
            response = self.client.chat.completions.create(
                model="qwen-plus",
                messages=messages,
                response_format={"type": "json_object"},
            )

            response_message = response.choices[0].message
            content = response_message.content

            # 在 DashScope 的 OpenAI 兼容模式下，content 通常是 JSON 字符串
            if isinstance(content, str):
                try:
                    parsed = json.loads(content)
                    return {"success": True, "parsed": parsed}
                except Exception as parse_error:
                    rospy.logerr(f"解析 JSON 字符串失败: {parse_error}")
                    return {"error": f"❌ 无法解析大模型 JSON 输出: {content}"}
            elif isinstance(content, dict):
                return {"success": True, "parsed": content}
            else:
                return {"error": f"❌ 未知的响应内容类型: {type(content)}"}

        except Exception as e:
            rospy.logerr(f"大模型调用错误: {e}")
            return {"error": f"❌ 大模型调用失败: {str(e)}"}

    def get_available_models(self) -> List[str]:
        """获取可用的模型列表"""
        try:
            models = self.client.models.list()
            return [model.id for model in models.data]
        except Exception as e:
            rospy.logerr(f"获取模型列表失败: {e}")
            return []
