#!/usr/bin/env bash
# 第六章端到端实验：一键连续跑完全部 30 条任务（无需分批、无需手动改电量）
#
# 用法：
#   ./run_chapter6_experiments.sh                 # 需已 roslaunch inspection_full
#   ./run_chapter6_experiments.sh --with-launch   # 自动后台启动完整栈后跑实验
#   ./run_chapter6_experiments.sh --cleanup-only  # 仅清理残留进程后退出
#   ./run_chapter6_experiments.sh --dry-run
#   ./run_chapter6_experiments.sh --num-runs 3

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CLEANUP_SCRIPT="${SCRIPT_DIR}/chapter6_cleanup_stack.sh"

NUM_RUNS=1
WITH_LAUNCH=false
CLEANUP_ONLY=false
LAUNCH_PID=""
LAUNCH_PGID=""
PY_ARGS=()
CLEANED_ON_START=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-launch) WITH_LAUNCH=true; shift ;;
    --cleanup-only) CLEANUP_ONLY=true; shift ;;
    --num-runs) NUM_RUNS="${2:-1}"; shift 2 ;;
    --num-runs=*) NUM_RUNS="${1#*=}"; shift ;;
    *) PY_ARGS+=("$1"); shift ;;
  esac
done

source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || true
if [[ -f "${WS_ROOT}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/devel/setup.bash"
elif [[ -f "${HOME}/catkin_ws/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/catkin_ws/devel/setup.bash"
fi

kill_launch_group() {
  if [[ -n "${LAUNCH_PGID}" ]]; then
    echo "[run_chapter6] 终止 roslaunch 进程组 (pgid=${LAUNCH_PGID})"
    kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null || true
  elif [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[run_chapter6] 终止 roslaunch (pid=${LAUNCH_PID})"
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  LAUNCH_PID=""
  LAUNCH_PGID=""
}

run_stack_cleanup() {
  if [[ -x "${CLEANUP_SCRIPT}" ]]; then
    bash "${CLEANUP_SCRIPT}" || true
  else
    echo "[run_chapter6] 未找到 ${CLEANUP_SCRIPT}"
  fi
}

cleanup() {
  kill_launch_group
  # 仅在本脚本启动了 roslaunch 时做全栈清理，避免误杀用户手动开的其他 ROS 会话
  if [[ "${WITH_LAUNCH}" == "true" ]]; then
    run_stack_cleanup
  else
    pkill -TERM -f "chapter6_task_runner.py" 2>/dev/null || true
  fi
}

on_interrupt() {
  echo ""
  echo "[run_chapter6] 收到 Ctrl+C，正在清理后台任务..."
  cleanup
  exit 130
}

trap on_interrupt INT TERM
trap cleanup EXIT

if [[ "${CLEANUP_ONLY}" == "true" ]]; then
  run_stack_cleanup
  exit $?
fi

# 启动前先清理一次，避免上次 Ctrl+C 残留
echo "[run_chapter6] 启动前清理残留进程..."
run_stack_cleanup
CLEANED_ON_START=true

if [[ "${WITH_LAUNCH}" == "true" ]]; then
  echo "[run_chapter6] 后台启动 inspection_full.launch（realistic 场景，启动较慢）..."
  # setsid：便于 Ctrl+C 时按进程组整树杀掉
  setsid roslaunch inspection_dashboard inspection_full.launch initial_battery:=100.0 &
  LAUNCH_PID=$!
  sleep 0.5
  LAUNCH_PGID=$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d ' ' || true)
  echo "[run_chapter6] roslaunch pid=${LAUNCH_PID} pgid=${LAUNCH_PGID}"
  echo "[run_chapter6] 等待 ROS 栈就绪（最多 180s）..."
  if ! python3 "${SCRIPT_DIR}/chapter6_task_runner.py" --wait-stack --stack-timeout 180; then
    echo "[run_chapter6] 栈未就绪，正在清理后退出"
    exit 1
  fi
fi

echo "[run_chapter6] 开始一键 suite（每条任务 ${NUM_RUNS} 次）..."
set +e
python3 "${SCRIPT_DIR}/chapter6_task_runner.py" \
  --suite \
  --num-runs "${NUM_RUNS}" \
  --fresh \
  "${PY_ARGS[@]}"
PY_EXIT=$?
set -e

if [[ ${PY_EXIT} -ne 0 ]]; then
  echo "[run_chapter6] 实验脚本退出码 ${PY_EXIT}"
fi

echo "[run_chapter6] 完成 → ${SCRIPT_DIR}/chapter6_results.csv"
exit "${PY_EXIT}"
