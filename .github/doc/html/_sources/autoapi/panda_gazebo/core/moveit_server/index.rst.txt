:py:mod:`panda_gazebo.core.moveit_server`
=========================================

.. py:module:: panda_gazebo.core.moveit_server

.. autoapi-nested-parse::

   A ros server that creates several of MoveIt services which can be used to control
   the Panda robot or retrieve sensor data for the robot.

   Main services:
       * ``panda_arm/set_ee_pose``
       * ``get_random_joint_positions``
       * ``get_random_ee_pose``
       * ``planning_scene/add_box``
       * ``planning_scene/add_plane``

   Extra services:
       * ``panda_arm/get_ee``
       * ``panda_arm/set_ee``
       * ``panda_arm/get_ee_pose``
       * ``panda_arm/get_ee_pose_joint_config``
       * ``panda_arm/get_ee_rpy``
       * ``set_joint_positions``
       * ``get_controlled_joints``
       * ``panda_arm/set_joint_positions``
       * ``panda_hand/set_joint_positions``

   Dynamic reconfigure service:
       This node also contains a dynamic reconfigure service that allows you to change
       the control max velocity and acceleration scaling factors. You can supply the
       initial values for this dynamic reconfigure server using the
       'panda_moveit_planner_server/max_velocity_scaling_factor' and
       'panda_moveit_planner_server/max_velocity_scaling_factor' topics.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   panda_gazebo.core.moveit_server.PandaMoveItPlannerServer




Attributes
~~~~~~~~~~

.. autoapisummary::

   panda_gazebo.core.moveit_server.MAX_RANDOM_SAMPLES


.. py:data:: MAX_RANDOM_SAMPLES
   :value: 5

   

.. py:class:: PandaMoveItPlannerServer(arm_move_group='panda_arm', arm_ee_link='panda_link8', hand_move_group='hand', load_gripper=True, load_set_ee_pose_service=True, load_extra_services=False)


   Bases: :py:obj:`object`

   Used to control or request information from the Panda Robot. This is done using
   the MoveIt `moveit_commander` module.

   .. attribute:: robot

      The MoveIt robot
      commander object.

      :type: :obj:`moveit_commander.robot.RobotCommander`

   .. attribute:: scene

      The MoveIt robot scene commander object.

      :type: :obj:`moveit_commander.planning_scene_interface.PlanningSceneInterface`

   .. attribute:: move_group_arm

      The MoveIt arm move group object.

      :type: :obj:`moveit_commander.move_group.MoveGroupCommander`

   .. attribute:: move_group_hand

      The MoveIt hand move group object.

      :type: :obj:`moveit_commander.move_group.MoveGroupCommander`

   .. attribute:: ee_pose_target

      The last set ee pose.

      :type: :obj:`geometry_msgs.msg.Pose`

   .. attribute:: joint_positions_target

      Dictionary containing the last Panda arm
      and hand joint positions setpoint.

      :type: :obj:`dict`


