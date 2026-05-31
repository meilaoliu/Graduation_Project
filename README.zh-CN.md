# OmniInspect

中文说明 | [English](README.md)

![OmniInspect dashboard](docs/images/dashboard.png)


## 项目定位

OmniInspect 是一个基于 ROS Noetic 的变电站巡检机器人开源工作空间。
它把自然语言任务理解、拓扑语义地图、全局路径生成、局部轨迹优化、MPC 轨迹跟踪、Gazebo 仿真、拍照服务、电量约束和 Web 操作台接入同一个端到端巡检闭环。

项目面向差速地面机器人。
用户可以在网页操作台输入“前往35kV配电箱2”“检查SVG无功补偿区”“完整巡检一遍”等自然语言指令，系统会把指令解析为结构化巡检任务，映射到 OSM 语义地图中的设备或区域，生成 waypoint 序列和连续轨迹，并在仿真中完成导航、到点拍照、低电返航充电、任务恢复和异常纠偏。

## 项目亮点

- **语言交互不直接控制几何路径**：大模型负责把自然语言转成结构化任务，目标校验、拓扑搜索、返航充电和路径执行由本地确定性模块完成，降低幻觉对安全决策的影响。
- **拓扑语义地图驱动巡检任务**：使用 OSM XML 描述巡检点、设备标签、区域和通道连通关系；同一份地图同时服务于语言上下文、Dijkstra 路径搜索和网页地图编辑。
- **面向差速机器人的规控链路**：基于微分平坦性推导速度、角速度和曲率约束，将轨迹优化结果直接接到底层差速控制输入。
- **B 样条与 MINCO 双后端**：B 样条用于局部支撑、凸包约束和快速重规划；MINCO 将中间路径点和时间分配联合优化，减少额外时间重分配。
- **地面车适配的 MINCO 改进**：引入去分母式曲率约束、Huber 型惩罚、多拓扑候选轨迹择优和制动安全窗恢复策略，解决低速起停和重规划失败时的数值稳定与执行安全问题。
- **MPC 闭环跟踪**：线性化 MPC 在 30 ms 控制周期内求解 QP，通过微分平坦变换解析前馈参考速度/角速度，并用 RK4 延时补偿减小状态滞后。
- **可观测的端到端 Demo**：Web 操作台集中显示机器人状态、相机画面、自然语言对话、电量、拍照结果；另有语义地图维护界面。
- **开源友好的资产管理**：高保真 DAE 模型通过 GitHub Release 资产分发，普通 Git 历史不包含数百 MiB 大文件。

## 界面与场景

### 巡检操作台

启动完整系统后，在浏览器打开：

```text
http://localhost:5000/
```

页面由四类信息组成：

- 左侧：机器人位置、朝向、线速度、角速度、电量百分比、充电状态和剩余可行驶距离。
- 中间：Gazebo 相机实时画面，默认从 `/camera/image` 以 JPEG 形式推送。
- 右侧：自然语言指令输入框、系统回显、任务状态消息。
- 下方/侧栏：到点拍照事件和缩略图，照片默认保存到 `~/inspection_photos/`。

![Dashboard](docs/images/dashboard.png)

### 语义地图维护界面

地图编辑入口：

```text
http://localhost:5000/map
```

该页面用于维护巡检点坐标、拓扑连边和设备语义标签。
当设备名称、巡检点或通道关系变化时，可以更新 OSM 地图，而不需要在代码里硬编码巡检目标。

![Map editor](docs/images/map-editor.jpg)

### 变电站场景

轻量场景使用 Gazebo 基础几何体，适合快速联调。
高保真场景使用 Blender 导出的 DAE 模型和贴图，适合展示与论文实验。

![Substation top-down view](docs/images/substation-topdown.jpg)

语义目标标记图：

![Substation semantic targets](docs/images/substation-marked.jpg)

## 快速开始

推荐环境：

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Python 3.8
- `catkin_tools`

安装基础工具：

```bash
sudo apt update
sudo apt install python3-catkin-tools python3-rosdep
```

如果当前机器还没有初始化过 rosdep：

```bash
sudo rosdep init
rosdep update
```

安装依赖并构建：

```bash
git clone https://github.com/meilaoliu/OmniInspect.git
cd OmniInspect
rosdep install --from-paths src --ignore-src -r -y
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

自然语言功能默认使用阿里云 DashScope 的 OpenAI 兼容接口，模型默认配置为通义千问/Qwen：

```bash
export DASHSCOPE_API_KEY=...
# Optional defaults used by the code:
export DASHSCOPE_MODEL=qwen3.6-plus
export DASHSCOPE_ENABLE_THINKING=false
export DASHSCOPE_TEMPERATURE=0.1
export DASHSCOPE_MAX_TOKENS=2048
```

默认 `base_url` 为 `https://dashscope.aliyuncs.com/compatible-mode/v1`。
代码内部使用 `openai` Python 包只是因为 DashScope 提供 OpenAI-compatible API，并不表示默认走 OpenAI 官方接口。
`OPENAI_API_KEY` 只作为兼容 fallback 保留；正常运行 OmniInspect 时请设置 `DASHSCOPE_API_KEY`。
如果要切到其他 OpenAI-compatible 服务，需要同时覆盖 `DASHSCOPE_BASE_URL` 和 `DASHSCOPE_MODEL`。

不要把 API Key 写进仓库或提交到 Git。

## 运行完整巡检 Demo

### 轻量场景

轻量场景不需要额外资产，适合第一次验证：

```bash
source devel/setup.bash
roslaunch inspection_dashboard inspection_full.launch realistic:=false initial_battery:=100.0
```

启动后会同时拉起：

- Gazebo 变电站仿真环境。
- `ego_planner` 规划控制节点。
- `inspection_services` 拍照服务。
- `battery_simulator` 电量模拟。
- `inspection_dashboard` Web 操作台。
- `nlp_commander` 自然语言任务调度器。

然后打开：

```text
http://localhost:5000/
```

可以在网页右侧输入框尝试：

```text
前往35kv配电箱2
```

预期现象：

1. `nlp_commander` 将设备名称映射到 OSM 语义地图中的巡检节点。
2. SegmentScheduler 生成一段到目标点的 waypoint 序列。
3. 规划器发布连续轨迹，机器人在 Gazebo/RViz 中向目标移动。
4. 到达目标半径内后发布 `/segment_done`。
5. 调用 `/take_photo`，照片保存到 `~/inspection_photos/`，操作台出现拍照缩略图。
6. 停留约 3 秒后，任务完成，操作台显示系统回显。

更多可测试指令：

```text
检查SVG无功补偿区
巡检变压器区域
检查所有低压配电室
完整巡检一遍
沿着刚刚的路线巡检五分钟，然后返回充电
```

### 高保真场景

第一次运行前下载 Release 资产：

```bash
scripts/download_substation_assets.sh
```

启动完整系统：

```bash
source devel/setup.bash
roslaunch inspection_dashboard inspection_full.launch realistic:=true initial_battery:=35.0
```

`realistic:=true` 会加载高保真 `substation_dae` Gazebo 模型。
`system_substation_realistic.launch` 会自动设置 `GAZEBO_MODEL_PATH`，普通用户不需要修改 `~/.bashrc`。

低电返航测试可用较低初始电量：

```bash
roslaunch inspection_dashboard inspection_full.launch realistic:=false initial_battery:=35.0
```

输入：

```text
完整巡检一遍
```

预期现象：

1. 电量条随运动下降。
2. 当可达性规则判断剩余电量不足以完成下一目标并安全返回时，系统插入返航充电阶段。
3. 机器人返回充电点并调用 `/charge`。
4. 电量恢复到阈值后自动续接剩余巡检任务。

## 单模块运行

轻量变电站仿真：

```bash
source devel/setup.bash
roslaunch vehicle_simulator system_substation.launch
```

高保真变电站仿真：

```bash
scripts/download_substation_assets.sh
source devel/setup.bash
roslaunch vehicle_simulator system_substation_realistic.launch
```

规划控制链路：

```bash
source devel/setup.bash
roslaunch ego_planner run_in_substation.launch
```

只启动 Web 操作台：

```bash
source devel/setup.bash
roslaunch inspection_dashboard dashboard.launch
```

只启动自然语言调度器：

```bash
source devel/setup.bash
roslaunch nlp_commander nlp_commander.launch
```

## 模块文档导航

| 模块 | 作用 | 文档 |
| --- | --- | --- |
| `inspection_dashboard` | Web 操作台、SocketIO/ROS 桥接、地图编辑页面 | [README](src/inspection_dashboard/README.md), [DEMO](src/inspection_dashboard/DEMO.md) |
| `nlp_commander` | LLM 任务解析、OSM 地图加载、路径/阶段调度、低电返航 | [README_V2](src/nlp_commander/docs/README_V2.md), [PATH_OPTIMIZATION_SUMMARY](src/nlp_commander/docs/PATH_OPTIMIZATION_SUMMARY.md) |
| `inspection_services` | `/take_photo` 服务和 `/photo_event` 事件 | [launch](src/inspection_services/launch/photo_service.launch) |
| `battery_simulator` | 电量消耗、低电告警、充电服务 | [launch](src/battery_simulator/launch/battery_monitor.launch) |
| `substation_description` | 高保真 DAE 模型元数据与资产占位 | [README](src/substation_description/README.md) |
| `vehicle_simulator` | Gazebo worlds、机器人 URDF、传感器仿真 | [launch](src/autonomous_exploration_development_environment/src/vehicle_simulator/launch) |
| `ego-planner/planner` | 路径搜索、地图、B 样条、MINCO、MPC、FSM | [source](src/ego-planner/planner) |
| `navigation_baseline` | DWA/TEB 对比实验启动文件 | [launch](src/navigation_baseline/launch) |
| `benchmark` | 第 4/5/6 章相关实验脚本与回归测试 | [source](src/benchmark) |

## 规划与控制方法

### 语言任务到几何路径

`nlp_commander` 读取 OSM 地图中的节点、区域和连边。
大模型根据用户指令、简化地图和历史上下文输出结构化任务类型、目标集合、顺序约束、拍照/停留要求和路径策略。
随后本地模块完成目标校验、区域展开、Dijkstra 拓扑搜索、分段调度和低电返航判断。

这样做的核心原因是：自然语言理解可以交给大模型，但地图连通性、目标存在性、返航安全和几何路径不应交给大模型自由生成。

### B 样条轨迹优化

B 样条后端利用差速机器人的微分平坦特性，将轨迹规划转换到平坦输出空间中。
轨迹用三次均匀 B 样条表示，利用局部支撑和凸包性质简化碰撞、速度、加速度和曲率约束。
优化目标联合考虑：

- 避障代价。
- 加速度/跃度平滑性。
- 速度、加速度、曲率动力学可行性。
- 时间重分配与轨迹精化。

论文实验中，在 50 个固定目标点任务上，B 样条方法平均规划耗时为 **1.66 ms**，规划成功率为 **96%**。

### MINCO 时空联合优化

MINCO 后端将中间路径点和每段时间分配作为优化变量，利用带状边值问题求解框架以线性复杂度生成多项式系数。
相比固定时间间隔的 B 样条，MINCO 可以同时调整空间形状和时间分配，不需要额外的时间重分配步骤。

本项目针对差速地面机器人做了几项关键适配：

- 将速度、加速度、曲率约束施加到采样点。
- 利用 `omega = kappa * v` 的关系，将角速度可行性统一到曲率约束和速度约束中。
- 使用去分母式曲率约束，避免起步、停车和低速急转弯时的分母奇异。
- 引入 Huber 型分段惩罚，限制严重曲率违规时的梯度上界。
- 使用多拓扑候选轨迹择优，降低单一同伦类优化陷入局部极小的概率。
- 设计制动安全窗恢复策略，把重规划失败后的处理分成“安全窗继续执行、受控停车、紧急停车”三层。

论文实验中，MINCO 与其他方法在同一组 50 个固定目标点任务上的对比为：

| 方法 | 规划耗时 (ms) | 轨迹时间 (s) | 轨迹长度 (m) | 曲率平滑度 | 成功率 |
| --- | ---: | ---: | ---: | ---: | ---: |
| DWA | 3.12 | 11.28 | 9.87 | 2.15 | 82% |
| TEB | 12.87 | 8.64 | 9.12 | 1.32 | 90% |
| B 样条 | 1.66 | 7.32 | 8.85 | 0.76 | 96% |
| MINCO | 0.64 | 4.85 | 5.26 | 0.52 | 100% |

![MINCO planning](docs/images/minco-planning.png)

### MPC 轨迹跟踪

MPC 控制器接收 B 样条或 MINCO 轨迹，在每个控制周期沿参考轨迹采样预测时域，将差速机器人非线性运动学模型在参考状态处线性化，并构建 QP 问题输出线速度 `v` 和角速度 `omega`。

主要改进点：

- 从多项式轨迹的一阶、二阶导数解析计算 `v_ref` 和 `omega_ref`。
- 代价函数使用 `||U - U_ref||^2`，让 MPC 只计算偏差修正量，而不是把正确转向也当成控制量惩罚。
- 使用终端状态加权，提高短距离轨迹收敛速度。
- 使用 RK4 和历史控制输入做延时补偿，减小传感器、通信和求解延迟带来的状态滞后。

论文实验中，MPC 在 30 ms 控制周期内平均 QP 求解耗时为 **0.45 ms**。
整体平均横向误差为 **2.5 cm**，平均航向误差为 **1.4°**。
前馈参考输入使弯道平均横向误差从 **8.7 cm** 降至 **3.5 cm**；50 ms 延时条件下，RK4 延时补偿使平均横向误差从 **5.8 cm** 降至 **2.4 cm**。

![MPC tracking](docs/images/mpc-tracking-xy.jpg)

## 高保真资产

高保真 `substation_dae` 模型包含数百 MiB 的 DAE 网格和贴图，不放在普通 Git 历史中。
仓库只跟踪小型模型元数据和占位目录：

```text
src/substation_description/models/substation_dae/
  model.config
  model.sdf
  meshes/.gitkeep
  materials/textures/.gitkeep
```

用户通过 GitHub Release 资产安装：

```bash
scripts/download_substation_assets.sh
```

默认下载地址：

```text
https://github.com/meilaoliu/OmniInspect/releases/download/assets-v1/substation_dae_assets_v1.tar.gz
```

维护者打包资产：

```bash
scripts/package_substation_assets.sh
```

更多说明见 [docs/assets.md](docs/assets.md) 和 [docs/release.md](docs/release.md)。

## Baseline、测试与发布检查

运行 DWA baseline：

```bash
scripts/run_dwa_simulation.sh
```

运行 TEB baseline：

```bash
scripts/run_teb_simulation.sh
```

运行密集障碍场景：

```bash
scripts/run_dense_simulation.sh
```

运行 MPC 实验脚本：

```bash
scripts/run_mpc_experiments.sh
```

Python 回归测试：

```bash
source devel/setup.bash
python3 -m pytest src/benchmark src/nlp_commander/tests
```

发布前检查：

```bash
scripts/check_release_tree.sh
```

## 仓库结构

```text
src/
  ego-planner/
    planner/
      path_searching/       # graph search and path utilities
      plan_env/             # local map and raycasting utilities
      traj_utils/           # trajectory messages and visualization
      bspline_opt/          # B-spline trajectory optimization
      minco_opt/            # MINCO-based trajectory optimization
      mpc_controller/       # differential-drive MPC tracking
      plan_manage/          # planner node, FSM and trajectory server
    patchwork*/             # ground segmentation related packages
    qpOASES/                # bundled third-party QP solver source
  autonomous_exploration_development_environment/
    src/vehicle_simulator/  # Gazebo worlds, URDF and simulator nodes
  substation_description/   # substation model metadata and asset placeholders
  nlp_commander/            # language task parsing and waypoint scheduling
  inspection_dashboard/     # web dashboard and ROS bridge
  inspection_services/      # shared inspection messages and services
  battery_simulator/        # deterministic battery simulation
  navigation_baseline/      # DWA and TEB baseline launch files
  benchmark/                # regression tests and experiment scripts
docs/                       # setup, simulation, assets, release and design notes
scripts/                    # workspace-level helper scripts
```

## 许可证

OmniInspect 项目代码采用 BSD 3-Clause License。
第三方组件保留其原始许可证，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

