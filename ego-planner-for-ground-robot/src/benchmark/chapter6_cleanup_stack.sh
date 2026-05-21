#!/usr/bin/env bash
# 清理第六章实验 / inspection_full 残留的 ROS、Gazebo 与 Python 节点
# 可单独运行： ./chapter6_cleanup_stack.sh

set -u

echo "[cleanup] 正在清理巡检仿真栈残留进程..."

# 1) 实验脚本自身
pkill -TERM -f "chapter6_task_runner.py" 2>/dev/null || true
pkill -TERM -f "chapter6_task_runner" 2>/dev/null || true

# 2) roslaunch 及 launch 文件（按项目特征匹配）
pkill -TERM -f "inspection_full.launch" 2>/dev/null || true
pkill -TERM -f "inspection_dashboard.*launch" 2>/dev/null || true
pkill -TERM -f "run_in_substation.launch" 2>/dev/null || true
pkill -TERM -f "system_substation" 2>/dev/null || true
pkill -TERM -f "system_substation_realistic" 2>/dev/null || true
pkill -TERM -f "system_dense_obstacles" 2>/dev/null || true
pkill -TERM -f "roslaunch" 2>/dev/null || true

# 3) 本项目常见节点（Python/C++）
pkill -TERM -f "substation_nlp_commander" 2>/dev/null || true
pkill -TERM -f "dashboard_node.py" 2>/dev/null || true
pkill -TERM -f "battery_monitor_node.py" 2>/dev/null || true
pkill -TERM -f "photo_service" 2>/dev/null || true
pkill -TERM -f "ego_planner" 2>/dev/null || true
pkill -TERM -f "traj_server" 2>/dev/null || true
pkill -TERM -f "ego_replan_fsm" 2>/dev/null || true
pkill -TERM -f "minco_backend" 2>/dev/null || true
pkill -TERM -f "bspline_optimizer" 2>/dev/null || true
pkill -TERM -f "vehicle_simulator" 2>/dev/null || true
pkill -TERM -f "local_planner_node" 2>/dev/null || true
pkill -TERM -f "pathFollower" 2>/dev/null || true
pkill -TERM -f "sensorScanGeneration" 2>/dev/null || true

sleep 2

# 4) Gazebo / ROS 核心（先 TERM 再 KILL）
killall -TERM gzclient 2>/dev/null || true
killall -TERM gzserver 2>/dev/null || true
killall -TERM rosmaster 2>/dev/null || true
killall -TERM roscore 2>/dev/null || true
killall -TERM roslaunch 2>/dev/null || true
killall -TERM rosout 2>/dev/null || true

sleep 2

pkill -KILL -f "chapter6_task_runner.py" 2>/dev/null || true
pkill -KILL -f "inspection_full.launch" 2>/dev/null || true
pkill -KILL -f "substation_nlp_commander" 2>/dev/null || true
pkill -KILL -f "dashboard_node.py" 2>/dev/null || true
pkill -KILL -f "battery_monitor_node.py" 2>/dev/null || true
pkill -KILL -f "ego_planner" 2>/dev/null || true
pkill -KILL -f "roslaunch" 2>/dev/null || true

killall -KILL gzclient 2>/dev/null || true
killall -KILL gzserver 2>/dev/null || true
killall -KILL rosmaster 2>/dev/null || true
killall -KILL roscore 2>/dev/null || true
killall -KILL roslaunch 2>/dev/null || true

# 5) 释放默认 ROS 端口（若仍被占用）
if command -v fuser >/dev/null 2>&1; then
  fuser -k 11311/tcp 2>/dev/null || true
fi

sleep 1
echo "[cleanup] 清理完成。"

# 简要报告仍存活的可疑进程
REMAIN=$(ps aux 2>/dev/null | grep -E "gzserver|gzclient|rosmaster|roslaunch|inspection_dashboard|nlp_commander|battery_monitor|chapter6_task" | grep -v grep || true)
if [[ -n "${REMAIN}" ]]; then
  echo "[cleanup] 警告：仍检测到以下进程，请手动确认："
  echo "${REMAIN}"
  exit 1
fi

exit 0
