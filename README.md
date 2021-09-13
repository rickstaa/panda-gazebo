# Panda_gazebo

This package contains all the ROS packages needed for creating a Panda Emika Franka
Gazebo simulation. It is used by the [openai_ros](https://bitbucket.org/rickstaa/openai_ros/src/noetic/)
ROS RL package to create the openai_ros panda task environments.

## Clone instructions

To use this workspace, clone the repository inside your a catkin workspace folder. Since the repository contains several git submodules to use all the features, it needs to be cloned using the `--recurse-submodules` argument:

```bash
git clone --recurse-submodules https://github.com/rickstaa/bayesian-learning-control.git
```

If you already cloned the repository and forgot the `--recurse-submodule` argument you
can pull the submodules using the following git command:

```bash
git submodule update --init --recursive
```

## Installation instructions

After you cloned the repository, you have to install the system dependencies using the `rosdep install --from-path src --ignore-src -r -y` command. After these dependencies are installed, you can build the ROS packages inside the catkin workspace using the following build command:

```bash
catkin build
```

## Usage instructions

You can launch the gazebo using the `roslaunch` command:

```bash
roslaunch panda_gazebo start_simulation.launch
```
