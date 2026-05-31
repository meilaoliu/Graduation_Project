# Setup

## Platform

The workspace targets ROS Noetic on Ubuntu 20.04.

Install ROS and catkin tools before building:

```bash
sudo apt update
sudo apt install python3-catkin-tools python3-rosdep
```

Install project dependencies with rosdep where possible:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Some planner components use qpOASES from `src/ego-planner/qpOASES`.
If your system does not already provide it, build and install it according to that directory's upstream instructions.

## Build

```bash
catkin build
source devel/setup.bash
```

For a release-style build:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## LLM Keys

Natural-language features use Alibaba Cloud DashScope's OpenAI-compatible endpoint by default.
For normal OmniInspect usage, set a DashScope key:

```bash
export DASHSCOPE_API_KEY=...
```

Default model settings:

```bash
export DASHSCOPE_MODEL=qwen3.6-plus
export DASHSCOPE_ENABLE_THINKING=false
export DASHSCOPE_TEMPERATURE=0.1
export DASHSCOPE_MAX_TOKENS=2048
```

The default `DASHSCOPE_BASE_URL` is `https://dashscope.aliyuncs.com/compatible-mode/v1`.
`OPENAI_API_KEY` is accepted by the code only as a compatibility fallback; if you use another OpenAI-compatible provider, also override `DASHSCOPE_BASE_URL` and `DASHSCOPE_MODEL`.
