=====================================
Welcome to panda-gazebo documentation
=====================================

.. image:: /images/panda_environment.png
   :alt: Panda Gazebo environment

Welcome to the API documentation for the :panda-gazebo:`panda_gazebo <>` package. This package
contains all the ROS packages needed for creating a `Panda Emika Franka`_ Gazebo simulation. It
is used by the :ros-gazebo-gym:`ros_gazebo_gym <>` RL framework to create the Panda task environments.
It wraps the `franka_ros`_ package to add the functionalities needed to train RL agents efficiently.

Package Overview
================

The :panda-gazebo:`panda_gazebo <>` package contains several launch files, ROS nodes and a Gazebo plugin that ease the
:ros-gazebo-gym:`ros_gazebo_gym <>` interaction with the `Panda Gazebo simulation`_. The launch files
can spawn the Panda Robot in several distinct task environments. The ROS nodes generate several ROS services that make
controlling or getting information from the robot easier. The Gazebo plugins can lock specific joints of the Panda robot.
It also contains a Gazebo world plugin that can be used to lock specific joints of the Panda robot.

Launch files
------------

- **put_robot_in_world.launch:** Spawns the Panda robot in a given task environment.
- **Start_pick_and_place_world.launch:** Loads the pick and place task environment.
- **Start_push_world.launch:** Loads the push task environment.
- **Start_reach_world.launch:** Loads the reach task environment.
- **Start_slide_world.launch:** Loads the slide task environment.
- **start_simulation.launch:** Loads the Panda robot in a given task environment.

For more information about these task environments see `the ros_gazebo_gym documentation`_.

ROS Nodes
---------

- **panda_control_server**: Creates services related to the panda_control.
   - ``get_controlled_joints:`` Returns the panda joints that are currently controlled when using a given control type.
   - ``follow_joint_trajectory:`` Sets the arm joint trajectory and the gripper width.
   - ``set_joint_commands:`` Sets an arm command (i.e. position or effort) based on the specified control type.
   - ``panda_arm/follow_joint_trajectory:`` Sets the arm joint trajectory.
   - ``panda_hand/set_gripper_width:`` Sets the gripper width.
   - ``panda_arm/set_joint_positions:`` Sets the arm positions.
   - ``panda_arm/set_joint_efforts:`` Sets the arm efforts.
- **panda_moveit_server**: Creates services to control the robot through `MoveIt!`_.
   - ``panda_arm/set_ee_pose:`` Sets the end-effector pose.
   - ``get_random_joint_positions``: Returns random valid joint positions.
   - ``get_random_ee_pose``: Returns a valid random end-effector pose.
   - ``planning_scene/add_box``: Adds a box to the MoveIt planning scene
   - ``planning_scene/add_plane``: Adds a plane to the MoveIt planning scene
   - ``panda_arm/get_ee``: Returns the currently used end-effector link name.
   - ``panda_arm/set_ee``: Sets the end effector link.
   - ``panda_arm/get_ee_pose``: Returns the current end-effector pose.
   - ``panda_arm/get_ee_pose_joint_config``: Returns a set of possible joint configurations for a given end-effector pose.
   - ``panda_arm/get_ee_rpy``: Returns the current end-effector orientation.
   - ``set_joint_positions``: Sets the arm and Hand joints positions.
   - ``get_controlled_joints``: Gets the joints that MoveIt currently controls.
   - ``panda_arm/set_joint_positions``: Sets the arm joints positions.
   - ``panda_hand/set_joint_positions``: Sets the hand joints position.

Gazebo plugins
--------------

- **panda_joint_locker**: Creates a service to lock/unlock Panda robot's specific joints.
   - ``lock_unlock_panda_joints``: Locks/unlocks the specified joints of the Panda robot.

.. _`Panda Emika Franka`: https://frankaemika.github.io/docs
.. _`gazebo`: https://gazebosim.org
.. _`ros_gazebo_gym`: https://github.com/rickstaa/ros-gazebo-gym
.. _`the ros_gazebo_gym documentation`: https://rickstaa.dev/ros-gazebo-gym/get_started/envs/panda/panda_env.html#task-environments
.. _`franka_ros`: https://github.com/frankaemika/franka_ros
.. _`panda_moveit_config`: https://github.com/ros-planning/panda_moveit_config
.. _`Panda Gazebo simulation`: https://github.com/frankaemika/franka_ros/tree/develop/franka_gazebo
.. _`MoveIt!`: https://moveit.ros.org

API Documentation
=================

More information about this package's modules and classes can be found in the :ref:`Python API documentation <python_api>`. For more
information about the message the services require, see the :ref:`ROS API documentation <ros_api>`.

Contents
========

.. toctree::
   :maxdepth: 2
   :caption: Getting started
   :glob:

   get_started/install.rst
   get_started/usage.rst
   get_started/issues.rst

.. toctree::
   :maxdepth: 2
   :caption: Development

   dev/contributing.rst
   dev/doc_dev.rst
   dev/license.rst

.. toctree::
   :maxdepth: 2
   :caption: API Documentation

   autoapi/index.rst

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
