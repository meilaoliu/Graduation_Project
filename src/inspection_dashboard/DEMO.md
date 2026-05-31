# 联调演示脚本 (E2E Demo)

> 本脚本用于在真机/仿真环境验证 `inspection_full.launch` 的端到端流水线。
> 需要预先按 `中文说明.md` 配置好 qpOASES、patchwork、CMU 仿真依赖以及 LLM API key。

## 0. 前置准备

```bash
cd <workspace>
catkin build -j4
source devel/setup.bash
export DASHSCOPE_API_KEY=<your_key>     # nlp_commander LLM
```

可选：调小窗口数量与 RViz 视角。`realistic:=false` 启动较快。

## Demo 1 — 单点拍照（最小流程）

```bash
roslaunch inspection_dashboard inspection_full.launch realistic:=false
```

浏览器打开 `http://localhost:5000/` → 状态面板应显示电量 100%，画面应出现摄像头 RGB。

对话框输入：

```
前往35kv配电箱2
```

预期：

1. SegmentScheduler 切出 1 段（终点 = `35kv配电箱2`）。
2. ego_planner 生成全局轨迹，机器人沿其行进。
3. 到达半径 0.5m 内 → FSM 发布 `/segment_done`（`UInt32` seq=1）。
4. 调度器调用 `/take_photo`，照片落盘 `~/inspection_photos/*.jpg`，仪表盘画廊出现缩略图。
5. dwell 3s → 任务完成，对话框输出"任务完成"。

验证命令：

```bash
rostopic echo /global_waypoints -n 1
rostopic echo /segment_done    -n 1
rostopic echo /photo_event     -n 1
ls ~/inspection_photos/
```

## Demo 2 — 多停留点巡检

```bash
检查SVG无功补偿区
```

预期：调度器把路径切成 ≥3 段，逐段执行+拍照。日志中应看到段 seq 单调递增 1,2,3,...

## Demo 3 — 低电量自动返航 + 续接

```bash
roslaunch inspection_dashboard inspection_full.launch \
    realistic:=false initial_battery:=35.0
```

浏览器输入：

```
完整巡检一遍
```

预期：

1. 任务正常推进，电量持续下降（仪表盘电量条由绿→黄→红）。
2. 电量 < 20% → 触发 `/low_battery_alert`。
3. 当前段执行完毕（不打断段内轨迹）→ 调度器自动插入"当前点 → 入口点"返航段。
4. 抵达入口点 → 调度器调用 `/charge`，电量回升至 ≥95%。
5. 仪表盘电量恢复绿色 → 自动续接剩余段，直到任务完成。

验证命令：

```bash
rostopic echo /low_battery_alert
rostopic echo /battery_state
```

## Demo 4 — 切换规划器后端

不动巡检流水线，仅改 `manager/use_minco`：

```bash
# B 样条
sed -i 's|use_minco" value="true"|use_minco" value="false"|' \
   src/ego-planner/planner/plan_manage/launch/advanced_substation_param.xml
# 重启 launch；验证 Demo 1/2/3 行为一致

# MINCO
sed -i 's|use_minco" value="false"|use_minco" value="true"|' \
   src/ego-planner/planner/plan_manage/launch/advanced_substation_param.xml
```

`/global_waypoints` → `planGlobalTrajWaypoints()` 走的是 PolynomialTraj（min-snap），
两种后端切换只影响后续局部 refine。本巡检流水线对二者透明。

## 故障排查速查

| 现象                                       | 排查方向                                   |
| ------------------------------------------ | ------------------------------------------ |
| `/segment_done` 一直不发                   | FSM `dist_to_goal_actual` 未收敛 → 检查 odom z 与 `~waypoint_z` 是否一致 (默认 0.75) |
| 调度器收到旧 segment_done 提前推进         | 检查 `/segment_done` 是 UInt32 且 seq 单调（已修复） |
| `/take_photo` 超时                         | `photo_service_node` 未启动 / `/camera/image` 未发布 |
| 浏览器画面卡顿                             | 调小 `inspection_dashboard/camera_max_hz` 或 `camera_jpeg_quality` |
| 充电后任务未续接                           | 查 `nlp_commander` 日志看是否到达 `charge_full_threshold` |
| LLM 调用失败                               | 检查 `DASHSCOPE_API_KEY` 与外网联通       |
