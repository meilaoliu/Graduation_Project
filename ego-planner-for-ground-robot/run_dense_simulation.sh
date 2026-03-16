#!/bin/bash

# Default values
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
echo "Generating Dense Obstacles World..."
echo "  Count:    $COUNT"
echo "  Min Dist: $MIN_DIST"
echo "  Seed:     $SEED"
echo "=========================================="

# Define paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEN_SCRIPT="$SCRIPT_DIR/src/autonomous_exploration_development_environment/src/vehicle_simulator/scripts/gen_obstacles.py"
WORLD_FILE="$SCRIPT_DIR/src/autonomous_exploration_development_environment/src/vehicle_simulator/world/dense_obstacles.world"

# Generate the world
python3 "$GEN_SCRIPT" --count "$COUNT" --min_dist "$MIN_DIST" --seed "$SEED" --output "$WORLD_FILE"

if [ $? -ne 0 ]; then
    echo "Error: Failed to generate world file."
    exit 1
fi

echo "World generated successfully at: $WORLD_FILE"
echo "Starting Simulation..."
echo "=========================================="

# Ensure no zombie processes
killall -9 gzserver gzclient rosmaster > /dev/null 2>&1

# Source workspace
source "$SCRIPT_DIR/devel/setup.bash"

# Launch
roslaunch vehicle_simulator system_dense_obstacles.launch
