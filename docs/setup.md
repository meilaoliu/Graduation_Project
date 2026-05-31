# Setup

## Platform

The workspace targets ROS Noetic on Ubuntu 20.04.

Install ROS and catkin tools before building:

```bash
sudo apt update
sudo apt install python3-catkin-tools python3-rosdep
```

Install project dependencies with rosdep where possible:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Some planner components use qpOASES from `src/ego-planner/qpOASES`.
If your system does not already provide it, build and install it according to that directory's upstream instructions.

## Build

```bash
catkin build
source devel/setup.bash
```

For a release-style build:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## LLM Keys

Natural-language features read API keys from the environment:

```bash
export DASHSCOPE_API_KEY=...
# or
export OPENAI_API_KEY=...
```
