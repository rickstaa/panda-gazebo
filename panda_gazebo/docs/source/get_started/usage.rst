=====
Usage
=====

The :panda_gazebo:`panda_gazebo <>` contains three kinds of launch files: world launch files, a robot launch file
and a simulation launch file.

**World launch files**

World launch files start Gazebo and load a world where the Panda robot can be trained. They don't spawn the robot.

    - ``start_reach_world.launch``: World that can be used for training a reaching task.
    - ``start_push_world.launch``: World that can be used for training a push task.
    - ``start_pick_and_place_world.launch``: World can be used to train a puck and place task.
    - ``start_slide_world.launch``: World that can be used for training a sliding task.

**Robot launch file**

The ``put_robot_in_world.launch`` robot launch files spawn the Panda robot in Gazebo and load the required control services.

**Simulation launch file**

The ``start_simulation.launch`` launch file combines the two other launch files to start the gazebo world and spawns the Panda robot.

Usage instructions
------------------

You can launch any launch files using the ``roslaunch`` command-line tool. For example, if you want to start a gazebo simulation
of the Panda robot, you can use the following ``roslaunch`` command:

.. code-block:: bash

    roslaunch panda_gazebo start_simulation.launch
