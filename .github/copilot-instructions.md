# Copilot Instructions - OmniInspect

OmniInspect is a ROS Noetic catkin workspace for substation inspection with a ground robot.
The repository root is the workspace root; active packages live under `src/`.

## Build And Run

Use `catkin build` from the repository root:

```bash
catkin build
source devel/setup.bash
```

Common launch entry points:

```bash
roslaunch vehicle_simulator system_substation.launch
roslaunch vehicle_simulator system_substation_realistic.launch
roslaunch ego_planner run_in_substation.launch
roslaunch inspection_dashboard inspection_full.launch
```

The high-fidelity substation scene is distributed as a release asset.
Install it with:

```bash
scripts/download_substation_assets.sh
```

Do not ask users to edit `~/.bashrc` for `GAZEBO_MODEL_PATH`; the realistic launch file sets the model path through `substation_description`.

## Project Map

- `src/ego-planner/planner/`: local planner, map, trajectory optimization, and MPC controller packages.
- `src/autonomous_exploration_development_environment/src/vehicle_simulator/`: Gazebo worlds, robot model, and simulation launch files.
- `src/substation_description/`: Gazebo metadata and asset layout for the high-fidelity substation model.
- `src/nlp_commander/`: natural-language mission parsing and waypoint planning.
- `src/inspection_dashboard/`: Flask/SocketIO dashboard and ROS bridge.
- `src/inspection_services/`: shared inspection messages and services.
- `src/battery_simulator/`: simulated battery state publisher.
- `src/navigation_baseline/`: DWA and TEB baseline launch files and recorder.
- `src/benchmark/`: benchmark scripts and selected result artifacts.

## Conventions

- Keep ROS package names stable unless a task explicitly asks for a mechanical rename.
- Use package-relative paths through `$(find package)` in launch files and `rospkg` in Python.
- Keep large meshes, textures, build output, logs, caches, and local editor state out of Git.
- Prefer focused changes near the affected package; avoid broad refactors during bug fixes.
- LLM features read `DASHSCOPE_API_KEY` or `OPENAI_API_KEY` from the environment.

## Verification

Before claiming a change is complete, run the narrowest relevant checks:

```bash
scripts/check_release_tree.sh
catkin build
python3 -m pytest src/benchmark src/nlp_commander/tests
roslaunch vehicle_simulator system_substation_realistic.launch --nodes
roslaunch ego_planner run_in_substation.launch --nodes
```
