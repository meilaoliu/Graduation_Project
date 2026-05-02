#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的LLM测试，直接测试API调用
"""

import os
import json

try:
    from openai import APIConnectionError, AuthenticationError, OpenAI, OpenAIError
except ImportError:
    APIConnectionError = None
    AuthenticationError = None
    OpenAI = None
    OpenAIError = Exception


PROXY_ENV_VARS = [
    "http_proxy",
    "https_proxy",
    "all_proxy",
    "HTTP_PROXY",
    "HTTPS_PROXY",
    "ALL_PROXY",
]


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}


def _print_client_init_error(error, proxy_names):
    print(f"❌ OpenAI客户端初始化失败: {type(error).__name__}: {error}")
    if "Unknown scheme for proxy URL" in str(error):
        print("   原因: 当前代理变量使用了 httpx 不接受的 socks:// 写法。")
        print("   方案1: export DASHSCOPE_CLEAR_PROXY=1 后重试，让脚本忽略代理。")
        print("   方案2: 如果 Clash 提供 HTTP 代理，把代理改成 http://127.0.0.1:7890。")
        print("   方案3: 如果必须走 SOCKS，安装 SOCKS 支持并使用 socks5://127.0.0.1:7890。")
    print(f"   代理变量: {', '.join(proxy_names) if proxy_names else '未设置'}")

def test_simple_llm_call():
    """简单测试LLM JSON结构化输出"""
    print("🧪 简单LLM JSON结构化输出测试")
    print("=" * 50)

    if _env_bool("DASHSCOPE_CLEAR_PROXY", False):
        for proxy_var in PROXY_ENV_VARS:
            os.environ.pop(proxy_var, None)

    if os.getenv("RUN_REAL_LLM_TEST") != "1":
        print("ℹ️ 未设置 RUN_REAL_LLM_TEST=1，跳过真实 LLM 调用测试。")
        return
    
    if OpenAI is None:
        print("❌ 未安装 openai Python 包，跳过真实 LLM 调用测试。")
        return

    api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("❌ 未设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY，跳过真实 LLM 调用测试。")
        return

    model = os.getenv("DASHSCOPE_MODEL", "qwen3.6-plus")
    base_url = os.getenv("DASHSCOPE_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1")
    enable_thinking = _env_bool("DASHSCOPE_ENABLE_THINKING", False)
    temperature = float(os.getenv("DASHSCOPE_TEMPERATURE", "0.1"))
    max_tokens = int(os.getenv("DASHSCOPE_MAX_TOKENS", "2048"))
    proxy_names = [proxy_var for proxy_var in PROXY_ENV_VARS if os.getenv(proxy_var)]

    print(f"🔧 model={model}")
    print(f"🔧 base_url={base_url}")
    print(f"🔧 key_present={bool(api_key)}, key_length={len(api_key) if api_key else 0}")
    print(f"🔧 enable_thinking={enable_thinking}, temperature={temperature}, max_tokens={max_tokens}")
    print(f"🔧 proxy_env={', '.join(proxy_names) if proxy_names else '未设置'}")

    # 初始化客户端
    try:
        client = OpenAI(
            api_key=api_key,
            base_url=base_url,
        )
    except Exception as e:
        _print_client_init_error(e, proxy_names)
        return

    # 系统提示词
    system_prompt = """你是一个专业的变电站巡检机器人智能助手。你的任务是理解用户的巡检指令，并输出严格 JSON。

🤖 机器人当前位置: 入口点 (9.1, 27.5)

📍 全部可用设备点:
   • 入口点
   • 插值点1
   • 低压配电室1
   • 低压配电室2
   • 低压配电室3
   • 高压配电区巡检点1
   • 高压配电区巡检点2
   • 高压配电区巡检点3
   • 变压器区1
   • 变压器区2
   • 变压器区3
   • 3SVG无功补偿区
   • 2SVG无功补偿区
   • 1SVG无功补偿区
   • 35kv配电箱1
   • 35kv配电箱2
   • 35kv配电箱3

🎯 航点选择策略:
- 单个设备: 去单个设备点，如"去低压配电室1" → waypoint_sequence: ["低压配电室1"]
- 区域巡检: 检查某区域所有设备，如"检查SVG区" → waypoint_sequence: ["1SVG无功补偿区", "2SVG无功补偿区", "3SVG无功补偿区"]
- 完整巡检: 完整巡检，如"完整巡检一遍" → waypoint_sequence: [所有设备点列表]
- 自定义路径: 多个具体设备点 → waypoint_sequence: ["设备1", "设备2"]

输出 JSON 格式：
{
    "task_type": "single_target / area_inspection / full_inspection / custom_path",
    "target_devices": [{"name": "设备名称", "priority": 1}],
    "task_description": "任务描述"
}

⚠️ 重要: 只输出 JSON，不要输出 Markdown 或额外解释。"""

    # 测试指令
    test_commands = [
        "去高压配电区1看一下",
        "完整巡检一遍",
        "检查SVG区域"
    ]
    
    for i, command in enumerate(test_commands, 1):
        print(f"\n🎯 测试 {i}: {command}")
        print("-" * 30)
        
        user_message = f"""用户巡检指令: {command}

🤖 机器人当前状态:
- 位置: 入口点 (9.1, 27.5)

请分析指令并输出 JSON。记住：
1. 只指定最终目标设备点，不要指定中间路径
2. 系统会自动使用Dijkstra算法计算最优路径
3. JSON 中的设备名称必须来自系统已知设备点"""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
        
        try:
            request_kwargs = {
                "model": model,
                "messages": messages,
                "response_format": {"type": "json_object"},
                "temperature": temperature,
                "max_tokens": max_tokens,
            }
            if model.lower().startswith("qwen") or "DASHSCOPE_ENABLE_THINKING" in os.environ:
                request_kwargs["extra_body"] = {"enable_thinking": enable_thinking}

            response = client.chat.completions.create(**request_kwargs)
            
            response_message = response.choices[0].message
            parsed = json.loads(response_message.content)
            print(f"📨 原始响应 - content: {response_message.content}")
            print(f"✅ JSON解析成功: {parsed}")
                
        except AuthenticationError as e:
            print("❌ 鉴权失败: API Key 无效、未开通模型服务，或 Key 所属地域与 base_url 不匹配。")
            print(f"   详细类型: {type(e).__name__}")
        except APIConnectionError as e:
            print("❌ 连接失败: Python/OpenAI SDK 未能连到 DashScope。")
            print(f"   当前 base_url: {base_url}")
            print(f"   代理变量: {', '.join(proxy_names) if proxy_names else '未设置'}")
            print("   建议先运行: curl -sS -o /dev/null -w '%{http_code} %{remote_ip} %{ssl_verify_result}\\n' https://dashscope.aliyuncs.com/compatible-mode/v1/models")
            print(f"   详细类型: {type(e).__name__}")
        except OpenAIError as e:
            print(f"❌ OpenAI兼容接口异常: {type(e).__name__}: {e}")
        except Exception as e:
            print(f"❌ 异常: {e}")

if __name__ == "__main__":
    test_simple_llm_call() 