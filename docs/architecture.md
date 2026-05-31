# Architecture

OmniInspect is organized as a ROS workspace with independent catkin packages.

The main data flow is:

```text
vehicle_simulator
  -> terrain / point cloud topics
  -> plan_env
  -> path_searching
  -> bspline_opt or minco_opt
  -> plan_manage
  -> mpc_controller
```

Inspection modules build on that planning layer:

```text
nlp_commander -> inspection_services -> ego_planner goals
inspection_dashboard -> ROS topics/services -> browser UI
battery_simulator -> battery_state
```

The high-fidelity Gazebo model is isolated in `substation_description` so simulation resources can be resolved through ROS package paths instead of machine-local absolute paths.
