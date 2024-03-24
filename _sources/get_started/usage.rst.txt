==========
How to use
==========

The :panda-gazebo:`panda_gazebo <>` package contains three types of launch files: **world**, **robot** and **simulation** launch files.

**World launch files**

World launch files start Gazebo and load a world where the Panda robot can be trained. *They don't spawn the robot*.

    - ``start_reach_world.launch``: Provides a clear environment for reach task training with the robot.
    - ``start_push_world.launch``: Positions the Panda robot and a box on a table for push task simulations.
    - ``start_pick_and_place_world.launch``: Includes a table and cube, creating a scenario for pick-and-place exercises.
    - ``start_slide_world.launch``: Contains a puck and a designated target area to practice precision sliding tasks.

**Robot launch file**

The ``put_robot_in_world.launch`` robot launch files spawn the Panda robot in Gazebo and load the required control services. The robot currently contains three control
modes that can be selected using the ``control_mode`` argument:

    - ``trajectory``: The robot is controlled using joint position trajectories.
    - ``position``: The robot is controlled using joint position commands.
    - ``effort``: The robot is controlled using joint effort commands.    

.. Note::

    You can test different control modes using the :mod:`joint_efforts_dynamic_reconfigure_server` and :mod:`joint_positions_dynamic_reconfigure_server` nodes. 
    These nodes allow you to send joint efforts and joint positions to the robot. To learn more about utilizing these dynamic reconfigure servers, refer to the 
    documentation of the `dynamic_reconfigure`_ and `rqt_reconfigure`_ ROS packages.

    Furthermore, you can explore trajectory control using the `MoveIt! package`_ or `rqt_joint_trajectory_controller package`_. To enable `MoveIt!`, set the
    ``use_moveit`` launch file argument to ``true``. Once enabled, you can control the robot through the `RViz Motion Planning`_ panel. For detailed instructions on how to
    use `MoveIt!`_, consult the `MoveIt! tutorials`_.

.. _dynamic_reconfigure: https://wiki.ros.org/dynamic_reconfigure
.. _rqt_reconfigure: https://wiki.ros.org/rqt_reconfigure
.. _`MoveIt! package`: https://moveit.ros.org/
.. _`rqt_joint_trajectory_controller package`: https://wiki.ros.org/rqt_joint_trajectory_controller
.. _`RViz Motion Planning`: https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html
.. _`MoveIt!`: https://ros-planning.github.io/moveit_tutorials/
.. _`MoveIt! tutorials`: https://ros-planning.github.io/moveit_tutorials/

**Simulation launch file**

The ``start_simulation.launch`` launch file combines the two other launch files to start the gazebo world and spawns the Panda robot.

Usage instructions
------------------

You can launch any launch files using the ``roslaunch`` command-line tool. For example, if you want to start a gazebo simulation
of the Panda robot, you can use the following ``roslaunch`` command:

.. code-block:: bash

    roslaunch panda_gazebo start_simulation.launch


The note you've provided is clear and informative, but you can improve it further by organizing the information more effectively and making it easier to read. Here's a revised version:
