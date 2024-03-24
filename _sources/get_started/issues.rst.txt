============
Known Issues
============

This document details the potential issues that may arise while using the :panda-gazebo:`panda_gazebo <>` package. These problems are primarily due to bugs in the upstream `franka_ros`_ package, which is utilized by the :panda-gazebo:`panda_gazebo <>` package to simulate the Panda robot. For the most recent updates and solutions, we suggest visiting :panda-gazebo:`the issues page <issues>` on the :panda-gazebo:`GitHub repository <>`.

.. _`franka_ros`: https://frankaemika.github.io/docs/franka_ros.html

Gravity Compensation Bug
-------------------------

- **Issue**: Gravity compensation is not working properly when the robot is effort-controlled.
- **Reference**: `#39 <https://github.com/rickstaa/panda-gazebo/issues/39>`_

Due to an upstream bug in the :panda-gazebo:`franka_gazebo <>` package, the gravity compensation feature may not function as expected. 
This issue can be mitigated by switching the physics engine from `ODE`_ to `DART`_ using the ``physics`` argument in the ``simulation.launch`` file:

.. code-block:: bash

    roslaunch panda_gazebo simulation.launch physics:=Dart

Simulation Crashes with DART Physics
--------------------------------------

- **Issue**: The simulation sometimes crashes when the `DART`_ physics engine is used and the gripper is controlled.
- **Reference**: `#196 <https://github.com/rickstaa/panda-gazebo/issues/196>`_

Users may experience occasional simulation crashes when using the `DART`_ physics engine and controlling the gripper. We
recommend switching to the `ODE`_ physics engine when using the gripper.

Gripper Problems when Vertical
------------------------------

- **Issue**: The gripper is not working properly when being vertical to the ground.
- **Reference**: `#33 <https://github.com/rickstaa/panda-gazebo/issues/33>`_

Due to incorrectly tuned PID gains in the :panda-gazebo:`franka_gazebo <>` package, the gripper may not function properly when it is oriented
vertically to the ground.

.. _ODE: http://www.ode.org/
.. _DART: https://dartsim.github.io/
