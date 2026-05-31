#!/bin/bash

# TEB baseline experiment launcher.
# Generates the dense obstacle world + occupancy grid map, then starts:
#   1. Simulation environment (Gazebo, vehicle, sensors)
#   2. move_base with TEB local planner
#   3. Trajectory recorder

# Default values (same as run_dense_simulation.sh)
COUNT=160
MIN_DIST=1.8
SEED=41

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --count) COUNT="$2"; shift ;;
        --min_dist) MIN_DIST="$2"; shift ;;
        --seed) SEED="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

echo "=========================================="
echo "TEB Baseline Experiment"
echo "  Obstacles: $COUNT, MinDist: $MIN_DIST, Seed: $SEED"
echo "=========================================="

# Define paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
GEN_SCRIPT="$WORKSPACE_ROOT/src/autonomous_exploration_development_environment/src/vehicle_simulator/scripts/gen_obstacles.py"
WORLD_FILE="$WORKSPACE_ROOT/src/autonomous_exploration_development_environment/src/vehicle_simulator/world/dense_obstacles.world"
MAP_FILE="$WORKSPACE_ROOT/src/autonomous_exploration_development_environment/src/vehicle_simulator/world/dense_obstacles_map.pgm"
BENCHMARK_DIR="$WORKSPACE_ROOT/src/benchmark"

# Generate world + map
python3 "$GEN_SCRIPT" --count "$COUNT" --min_dist "$MIN_DIST" --seed "$SEED" \
    --output "$WORLD_FILE" --map-output "$MAP_FILE"

if [ $? -ne 0 ]; then
    echo "Error: Failed to generate world/map."
    exit 1
fi

echo "World and map generated successfully."
echo "Starting TEB simulation..."
echo "=========================================="

# Ensure no zombie processes
killall -9 gzserver gzclient rosmaster > /dev/null 2>&1

# Source workspace
source "$WORKSPACE_ROOT/devel/setup.bash"

# Launch simulation + TEB + trajectory recorder
roslaunch navigation_baseline sim_env_only.launch &
SIM_PID=$!
sleep 8

roslaunch navigation_baseline move_base_teb.launch &
MB_PID=$!
sleep 3

rosrun navigation_baseline traj_recorder.py \
    _planner_type:=teb _output_dir:="$BENCHMARK_DIR" &
REC_PID=$!

echo "=========================================="
echo "All components started."
echo "  Simulation PID: $SIM_PID"
echo "  move_base PID:  $MB_PID"
echo "  Recorder PID:   $REC_PID"
echo ""
echo "Now run the benchmark in another terminal:"
echo "  cd src/benchmark"
echo "  python3 random_goal_benchmark.py --tag teb"
echo "=========================================="

wait $SIM_PID
