#!/bin/bash
#
# MPC 第五章实验运行脚本
#
# 使用方法:
#   ./run_mpc_experiments.sh              # 显示帮助
#   ./run_mpc_experiments.sh baseline     # 实验1: 完整MPC (前馈+延时补偿)
#   ./run_mpc_experiments.sh no_ff        # 实验2: 关闭前馈 (消融)
#   ./run_mpc_experiments.sh no_delay     # 实验3: 关闭延时补偿 (消融)
#
# 每次实验结束后:
#   1. RViz 中蓝色=参考轨迹, 绿色=实际轨迹, 可截图
#   2. Ctrl+C 退出后自动保存 cmd_profile.csv
#   3. 运行 analyze_tracking.py 生成统计和图表

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCHMARK_DIR="$SCRIPT_DIR/src/benchmark"
ADVANCED_PARAM="$SCRIPT_DIR/src/ego-planner/planner/plan_manage/launch/advanced_param.xml"

usage() {
    echo "======================================"
    echo "  MPC 第五章实验脚本"
    echo "======================================"
    echo ""
    echo "用法: $0 <experiment>"
    echo ""
    echo "  baseline  - 完整MPC (前馈ON + 延时补偿ON)"
    echo "  no_ff     - 关闭前馈消融实验"
    echo "  no_delay  - 关闭延时补偿消融实验"
    echo "  analyze   - 分析最新的 cmd_profile.csv"
    echo ""
    echo "实验流程:"
    echo "  1. 先在另一个终端启动仿真环境:"
    echo "     ./run_dense_simulation.sh"
    echo "  2. 等待 Gazebo 和 RViz 完全加载"
    echo "  3. 在新终端运行本脚本启动 ego_planner + MPC:"
    echo "     ./run_mpc_experiments.sh baseline"
    echo "  4. 在 RViz 中用 2D Nav Goal 发送目标点"
    echo "  5. 轨迹跟踪完成后截图 (蓝色=参考, 绿色=实际)"
    echo "  6. Ctrl+C 退出, csv 自动保存"
    echo "  7. 运行分析: ./run_mpc_experiments.sh analyze"
    echo ""
}

set_param() {
    local param=$1
    local value=$2
    # Use rosparam to dynamically set parameters
    rosparam set "/ego_planner_node/MPC/$param" "$value"
}

run_experiment() {
    local exp_name=$1
    local use_ff=$2
    local use_delay=$3

    echo "======================================"
    echo "  实验: $exp_name"
    echo "  前馈: $use_ff"
    echo "  延时补偿: $use_delay"
    echo "======================================"

    # Modify launch params via sed (temporary, will be restored)
    local BACKUP="$ADVANCED_PARAM.bak"
    cp "$ADVANCED_PARAM" "$BACKUP"

    # Set use_feedforward
    sed -i "s|<param name=\"MPC/use_feedforward\" value=\"[^\"]*\"|<param name=\"MPC/use_feedforward\" value=\"$use_ff\"|" "$ADVANCED_PARAM"
    # Set use_delay_compensation
    sed -i "s|<param name=\"MPC/use_delay_compensation\" value=\"[^\"]*\"|<param name=\"MPC/use_delay_compensation\" value=\"$use_delay\"|" "$ADVANCED_PARAM"

    echo "参数已设置, 启动 ego_planner..."
    echo "提示: 在 RViz 中用 2D Nav Goal 发送目标点"
    echo "完成后 Ctrl+C 退出 (csv 自动保存)"
    echo ""

    source "$SCRIPT_DIR/devel/setup.bash"
    roslaunch ego_planner run_in_sim.launch

    # Restore original param file
    mv "$BACKUP" "$ADVANCED_PARAM"

    # Rename csv for this experiment
    if [ -f "$BENCHMARK_DIR/cmd_profile.csv" ]; then
        local timestamp=$(date +%Y%m%d_%H%M%S)
        local dest="$BENCHMARK_DIR/cmd_profile_${exp_name}_${timestamp}.csv"
        cp "$BENCHMARK_DIR/cmd_profile.csv" "$dest"
        echo ""
        echo "数据已保存: $dest"
        echo "运行分析: python3 $BENCHMARK_DIR/analyze_tracking.py $dest"
    fi
}

case "${1:-}" in
    baseline)
        run_experiment "baseline" "true" "true"
        ;;
    no_ff)
        run_experiment "no_feedforward" "false" "true"
        ;;
    no_delay)
        run_experiment "no_delay_comp" "true" "false"
        ;;
    analyze)
        CSV="${2:-$BENCHMARK_DIR/cmd_profile.csv}"
        echo "分析: $CSV"
        python3 "$BENCHMARK_DIR/analyze_tracking.py" "$CSV"
        ;;
    *)
        usage
        ;;
esac
