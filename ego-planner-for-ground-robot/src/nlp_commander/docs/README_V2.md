# 变电站智能巡检指挥官 V2.0

基于图论的Dijkstra算法进行智能路径规划的模块化版本。

## 🏗️ 架构设计

### 模块化结构

```
nlp_commander/
├── utils/                          # 核心工具模块
│   ├── __init__.py                 # 包初始化
│   ├── graph_utils.py              # 图论工具 (Dijkstra算法)
│   ├── path_planner.py             # 路径规划器
│   ├── llm_utils.py                # LLM客户端
│   └── waypoint_manager.py         # 航点管理器
├── substation_nlp_commander_node_v2.py  # 主节点 (V2.0)
├── substation_nlp_commander_node.py     # 原始节点 (V1.0)
├── test_dijkstra.py               # 测试脚本
└── README_V2.md                   # 本文档
```

### 核心模块说明

1. **SubstationGraph** (`graph_utils.py`)
   - 变电站拓扑图管理
   - Dijkstra最短路径算法
   - 图连通性验证

2. **PathPlanner** (`path_planner.py`)
   - 单目标路径规划
   - 多目标贪心路径优化
   - 区域巡检路径生成
   - 模糊匹配和关键字识别

3. **LLMClient** (`llm_utils.py`)
   - 多模态大模型交互
   - Function Call工具定义
   - 图像编码和处理

4. **WaypointManager** (`waypoint_manager.py`)
   - 任务队列管理
   - ROS导航控制
   - 状态监控和回调

## 🧭 算法特性

### Dijkstra算法优势

- **最优路径**: 保证找到最短距离路径
- **物理布局考虑**: 基于变电站真实拓扑结构
- **避免跨区域跳跃**: 符合实际巡检逻辑
- **高效计算**: 时间复杂度O((V+E)logV)

### 图拓扑设计

变电站设备按物理布局分为三个主要区域：

```
东侧: 低压配电室区域 (入口 → 配电室1-3)
      ↓
中间: 35kV配电箱 + 高压配电区 + 变压器区
      ↓  
西侧: SVG/无功补偿区域 (补偿区1-3)
```

连接关系考虑：
- 区域内设备的顺序连接
- 跨区域的合理连接点
- 避免不现实的直接跳跃

## 🚀 使用方法

### 启动系统

```bash
# 启动仿真环境
roslaunch vehicle_simulator system_substation_realistic.launch

# 启动规划器
roslaunch ego_planner run_in_substation.launch

# 启动V2.0智能指挥官
cd ~/Graduation_Project/ego-planner-for-ground-robot/src/nlp_commander
python3 substation_nlp_commander_node_v2.py
```

### 支持的指令

1. **单点导航**
   - `前往低压配电室1`
   - `去35kV配电箱2`

2. **区域巡检**
   - `检查SVG无功补偿区`
   - `巡检变压器区域`
   - `检查所有低压配电室`

3. **完整巡检**
   - `完整巡检一遍`
   - `全面检查设备`

4. **任务控制**
   - `stop` - 停止当前任务
   - `pause` - 暂停任务
   - `resume` - 恢复任务

5. **状态查询**
   - `status` - 显示系统状态
   - `graph` - 显示图结构
   - `help` - 显示帮助信息

## 🔧 测试和验证

### 运行测试

```bash
python3 test_dijkstra.py
```

测试内容包括：
- 图结构验证
- Dijkstra算法正确性
- 路径规划器功能
- 图连通性检查

### 预期结果

所有测试应该通过，确保：
- ✅ 17个设备点全部可达
- ✅ 没有孤立点
- ✅ 路径规划算法正确
- ✅ 模糊匹配功能正常

## 🆚 V1.0 vs V2.0 对比

| 特性 | V1.0 | V2.0 |
|------|------|------|
| 架构 | 单文件 | 模块化 |
| 路径规划 | LLM生成中间点 | Dijkstra算法 |
| 空间理解 | 依赖LLM图像理解 | 基于图拓扑结构 |
| 路径优化 | 无保证 | 数学最优 |
| 可维护性 | 较低 | 高 |
| 扩展性 | 较低 | 高 |
| 性能 | 依赖LLM响应 | 快速计算 |

## 🔮 主要改进

1. **算法可靠性**: 从依赖LLM的空间理解转为基于图论的确定性算法
2. **模块化设计**: 每个功能独立模块，便于测试和维护
3. **性能提升**: 本地计算替代LLM调用，响应更快
4. **路径质量**: 保证最短路径，避免不合理绕行
5. **易于扩展**: 添加新设备点只需修改图配置

## 📝 开发说明

### 添加新设备点

1. 在`graph_utils.py`的`locations`字典中添加坐标
2. 在`_build_adjacency_list()`方法中添加连接关系
3. 运行测试验证连通性

### 修改路径规划策略

在`path_planner.py`中修改相应的规划方法：
- `plan_path_to_single_target()` - 单目标
- `plan_multi_target_path()` - 多目标
- `plan_area_inspection()` - 区域巡检

### 自定义LLM提示词

在`llm_utils.py`的`create_system_prompt()`方法中修改系统提示词。

## 🎯 未来扩展

- [ ] A*算法支持启发式搜索
- [ ] 动态障碍物避让
- [ ] 实时地图更新
- [ ] 多机器人协调巡检
- [ ] 任务优先级管理
- [ ] 设备状态反馈集成 