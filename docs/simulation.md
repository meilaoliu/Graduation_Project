# Simulation

## Lightweight Substation World

The lightweight world uses primitive Gazebo geometry and is available immediately after build:

```bash
source devel/setup.bash
roslaunch vehicle_simulator system_substation.launch
```

## High-Fidelity Substation World

Install the release asset first:

```bash
scripts/download_substation_assets.sh
source devel/setup.bash
roslaunch vehicle_simulator system_substation_realistic.launch
```

The launch file sets `GAZEBO_MODEL_PATH` to `$(find substation_description)/models`.
Do not add model paths to `~/.bashrc` for normal use.

## Planner

Run the planner against the substation setup:

```bash
source devel/setup.bash
roslaunch ego_planner run_in_substation.launch
```

Use RViz `2D Nav Goal` or the inspection dashboard to send goals.
