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

try:
    from openai import OpenAI
except ImportError:
    OpenAI = None

from .osm_map_loader import load_simplified_osm

PROXY_ENV_VARS = [
    "http_proxy",
    "https_proxy",
    "all_proxy",
    "HTTP_PROXY",
    "HTTPS_PROXY",
    "ALL_PROXY",
]

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

    def __init__(self, api_key: str = None, base_url: str = None, model: str = None):
        self.api_key = api_key or os.getenv("DASHSCOPE_API_KEY") or os.getenv("OPENAI_API_KEY")
        if self._parse_bool(os.getenv("DASHSCOPE_CLEAR_PROXY"), False):
            for proxy_var in PROXY_ENV_VARS:
                os.environ.pop(proxy_var, None)

        self.base_url = base_url or os.getenv(
            "DASHSCOPE_BASE_URL",
            "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        self.model = model or os.getenv("DASHSCOPE_MODEL", "qwen3.6-plus")
        self.enable_thinking = self._parse_bool(os.getenv("DASHSCOPE_ENABLE_THINKING"), False)
        self.temperature = self._parse_float(os.getenv("DASHSCOPE_TEMPERATURE"), 0.1)
        self.max_tokens = self._parse_int(os.getenv("DASHSCOPE_MAX_TOKENS"), 2048)
        self.client = None
        self.client_error = ""

        if OpenAI is None:
            rospy.logwarn("未安装 openai Python 包，LLM 指令解析功能不可用。")
        elif not self.api_key:
            rospy.logwarn("未设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY，LLM 指令解析功能不可用。")
        else:
            try:
                self.client = OpenAI(api_key=self.api_key, base_url=self.base_url)
                rospy.loginfo(
                    "LLM配置: "
                    f"model={self.model}, base_url={self.base_url}, "
                    f"enable_thinking={self.enable_thinking}, "
                    f"temperature={self.temperature}, max_tokens={self.max_tokens}"
                )
            except Exception as e:
                self.client_error = self._format_client_init_error(e)
                rospy.logerr(self.client_error)

        # 简化版 OSM 地图路径（相对于 nlp_commander 目录）
        self.simplified_osm_path = "maps/simplified_substation.osm"

    @staticmethod
    def _parse_bool(value: str, default: bool) -> bool:
        if value is None:
            return default
        return value.strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}

    @staticmethod
    def _parse_float(value: str, default: float) -> float:
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _parse_int(value: str, default: int) -> int:
        if value is None:
            return default
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _format_client_init_error(error: Exception) -> str:
        proxy_names = [proxy_var for proxy_var in PROXY_ENV_VARS if os.getenv(proxy_var)]
        proxy_text = ", ".join(proxy_names) if proxy_names else "未设置"
        message = f"初始化 OpenAI 兼容客户端失败: {type(error).__name__}: {error}"
        if "Unknown scheme for proxy URL" in str(error):
            message += (
                "。检测到代理变量: "
                f"{proxy_text}。httpx 不接受 socks:// 写法；可改为 http://127.0.0.1:7890，"
                "或安装 SOCKS 支持后使用 socks5://127.0.0.1:7890，"
                "也可设置 DASHSCOPE_CLEAR_PROXY=1 临时忽略代理。"
            )
        return message

    def create_system_prompt(
        self,
        current_pos_info: str,
        available_locations: List[str],
        runtime_context: str = "无",
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

【当前任务执行上下文】
下面是机器人在本任务周期内的近期执行反馈，可能包含已经到达的巡检点、失败段、电量、返航充电等信息。
如果用户在任务执行中追加指令，或在任务结束后使用“刚才、继续、再按之前、同样要求”等省略表达，你需要结合任务摘要、对话历史和执行反馈理解上下文，但不要直接输出底层运动控制动作。
{runtime_context}

【你的推理任务】
给定一条中文巡检指令，你需要直接解析成机器人可执行的结构化意图：
1. 判断任务类型：single_target、area_inspection、full_inspection、custom_path、return_home、go_charge。
2. 从地图中选择目标设备，并为每个目标给出 stop_required 和 photo_required。
3. 提取执行约束：repeat_count、duration_minutes、until_time、preserve_order。
4. 给出路径策略：是否使用拓扑路径、是否允许系统优化访问顺序、低电量是否返航。

【输出格式要求】
你必须严格输出一个 JSON 对象（不要包含任何多余的文本）。
若用户只表达一个连续任务，使用单阶段格式：
{{
  "reasoning": "用中文简要说明你的思考过程，包括意图分析和设备选择依据。",
    "task_type": "single_target / area_inspection / full_inspection / custom_path / return_home / go_charge 之一",
    "targets": [
        {{"name": "设备名称1", "priority": 1, "stop_required": true, "photo_required": true}},
        {{"name": "设备名称2", "priority": 2, "stop_required": true, "photo_required": true}}
  ],
    "task_description": "一句话概括本次巡检任务的目的和范围",
    "execution": {{
    "repeat_count": 1,
    "duration_minutes": null,
        "until_time": null,
        "preserve_order": false
    }},
    "route_policy": {{
        "use_topology": true,
        "optimize_order": true,
        "return_home_on_low_battery": true
    }}
}}

若用户使用“先、然后、接着、最后”等顺序表达，或一句话中包含多个不同目标范围/不同时间约束，使用多阶段格式：
{{
    "reasoning": "说明上下文指代、阶段划分和设备选择依据。",
    "task_plan": {{
        "mode": "sequential",
        "stages": [
            {{
                "stage_id": "stage_1",
                "task_type": "single_target / area_inspection / full_inspection / custom_path / return_home / go_charge 之一",
                "targets": [
                    {{"name": "设备名称1", "priority": 1, "stop_required": true, "photo_required": true}}
                ],
                "task_description": "本阶段任务说明",
                "execution": {{
                    "repeat_count": 1,
                    "duration_minutes": null,
                    "until_time": null,
                    "preserve_order": false
                }},
                "route_policy": {{
                    "use_topology": true,
                    "optimize_order": true,
                    "return_home_on_low_battery": true
                }}
            }}
        ]
    }}
}}

其中：
- targets 中的 name 必须来自地图中的设备名称，且尽量与用户指令语义对齐。
- targets 只输出真实巡检设备，不要输出 device_type 为 waypoint 的中间航点；中间路径由 Dijkstra 自动补全。
- 只有当某一阶段的唯一目的就是回到入口、起点或返航待命时，该阶段 task_type 才输出 return_home，targets 输出入口点，并将入口点的 photo_required 设为 false。
- 当用户明确说“返回充电、去充电、回充电口充电、返航充电”时，该阶段 task_type 输出 go_charge，targets 输出入口点，并将入口点的 photo_required 设为 false；go_charge 表示到达入口点后调用充电流程，不等同于普通 return_home。
- 如果用户说“巡检A后回到起点”，不要把整个任务输出成 return_home；应输出多阶段任务，先巡检A，再增加一个 return_home 阶段。
- priority 为正整数，数值越小说明优先级越高。
- 当用户要求完整巡检时，可以将所有设备点都列入 targets，并按合理顺序排序。
- 当用户要求“反复巡检三遍”时，不要忽略“三遍”，repeat_count 必须输出 3。
- 当用户要求“巡检20分钟”时，duration_minutes 输出 20，repeat_count 保持 1；限时续巡由执行层根据剩余时间动态决定，不要把时间换算成多遍重复。
- 当用户要求“巡检到15:30”时，until_time 输出 "15:30"，repeat_count 保持 1。
- 若用户明确说“按我说的顺序/依次/先A再B”，preserve_order 输出 true，route_policy.optimize_order 输出 false。
- 若只是区域覆盖巡检，preserve_order 输出 false，route_policy.optimize_order 输出 true，系统会基于拓扑优化访问顺序。
- stop_required 表示机器人是否要在该目标停留；photo_required 表示停留后是否拍照。一般巡检目标二者都为 true，返回入口点时 photo_required 为 false。
- 若用户指令无法在地图中找到对应设备，请在 reasoning 中解释，并根据地图信息给出最接近的备选设备列表。

当前用户指令可能是新任务，也可能是在任务执行中的追加要求；若存在运行时上下文，请结合已经完成、失败、剩余时间和电量状态理解本次指令。
当用户使用“刚才、上一轮、继续、再按之前、同样要求”等指代表达时，优先参考运行时上下文里的上一任务摘要和近期反馈，复用其中的任务语义、目标范围、执行约束或拍照要求；如果新的指令给出了时间、次数或目标变更，则以新约束覆盖旧约束。
对于“沿着刚刚的路线再巡检五分钟，然后去低压区巡检一圈回到起点”这类复合指令，应输出 sequential task_plan：第一阶段复用上一任务目标并设置 duration_minutes=5，第二阶段巡检低压区，第三阶段 return_home。
对于“沿着刚刚的路线巡检五分钟，然后去低压区巡检一圈，然后返回充电”这类复合指令，第三阶段应输出 go_charge，而不是 return_home。
"""
        return system_prompt

    def process_inspection_command(
        self,
        command: str,
        current_pos_info: str,
        available_locations: List[str],
        runtime_context: str = "无",
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
        if self.client is None:
            return {
                "error": self.client_error
                or "❌ LLM客户端不可用，请先安装 openai 并设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY。"
            }

        system_prompt = self.create_system_prompt(current_pos_info, available_locations, runtime_context)
        user_message = f"用户巡检指令: {command}"

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message},
        ]

        try:
            request_kwargs = {
                "model": self.model,
                "messages": messages,
                "response_format": {"type": "json_object"},
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
            }

            if self.model.lower().startswith("qwen") or "DASHSCOPE_ENABLE_THINKING" in os.environ:
                request_kwargs["extra_body"] = {"enable_thinking": self.enable_thinking}

            response = self.client.chat.completions.create(**request_kwargs)

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
            if e.__class__.__name__ == "APIConnectionError":
                return {
                    "error": "❌ 大模型连接失败，请检查网络、DASHSCOPE_BASE_URL、DNS 或代理设置。"
                    f"当前 base_url={self.base_url}。"
                }
            return {"error": f"❌ 大模型调用失败: {str(e)}"}

    def chat_json(
        self,
        messages: List[Dict[str, Any]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Run a generic JSON-only chat request for agent runtime prompts."""
        if self.client is None:
            return {
                "error": self.client_error
                or "❌ LLM客户端不可用，请先安装 openai 并设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY。"
            }

        try:
            request_kwargs = {
                "model": self.model,
                "messages": messages,
                "response_format": {"type": "json_object"},
                "temperature": self.temperature if temperature is None else temperature,
                "max_tokens": max_tokens or min(self.max_tokens, 2048),
            }

            if self.model.lower().startswith("qwen") or "DASHSCOPE_ENABLE_THINKING" in os.environ:
                request_kwargs["extra_body"] = {"enable_thinking": self.enable_thinking}

            response = self.client.chat.completions.create(**request_kwargs)
            content = response.choices[0].message.content
            if isinstance(content, str):
                return {"success": True, "parsed": json.loads(content)}
            if isinstance(content, dict):
                return {"success": True, "parsed": content}
            return {"error": f"❌ 未知的响应内容类型: {type(content)}"}
        except Exception as e:
            rospy.logerr(f"通用 JSON 大模型调用错误: {e}")
            return {"error": f"❌ 大模型调用失败: {str(e)}"}

    def get_available_models(self) -> List[str]:
        """获取可用的模型列表"""
        try:
            models = self.client.models.list()
            return [model.id for model in models.data]
        except Exception as e:
            rospy.logerr(f"获取模型列表失败: {e}")
            return []
