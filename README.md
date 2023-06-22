# Panda Gazebo

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/panda-gazebo)](https://github.com/rickstaa/panda-gazebo/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](contributing.md)
[![DOI](https://zenodo.org/badge/353980386.svg)](https://zenodo.org/badge/latestdoi/353980386)

This package contains all the ROS components needed for creating a [Panda Emika Franka](https://frankaemika.github.io/docs/)
Gazebo simulation. It is used by the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym)
RL framework to create the Panda task environments. It wraps the [franka\_ros](https://github.com/frankaemika/franka_ros) and [panda\_moveit\_config](https://github.com/ros-planning/panda_moveit_config) packages to add the extra functionalities needed to train RL agents efficiently.

## Clone the repository

Since the repository contains several git submodules to use all the features, it needs to be cloned using the `--recurse-submodules` argument:

```bash
git clone --recurse-submodules https://github.com/rickstaa/stable-learning-control.git
```

If you already cloned the repository and forgot the `--recurse-submodule` argument, you can pull the submodules using the following git command:

```bash
git submodule update --init --recursive
```

## Installation and Usage

Please see the accompanying [documentation](https://rickstaa.dev/stable-gym) for information on installing and using this package.

## Contributing

<!--alex ignore husky-hooks-->

We use  [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Before contributing to this repository, please check the [contribution guidelines](CONTRIBUTING.md).
