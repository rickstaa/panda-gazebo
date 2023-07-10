:py:mod:`panda_gazebo.core`
===========================

.. py:module:: panda_gazebo.core

.. autoapi-nested-parse::

   Contains the core components (classes and functions) that are needed for
   creating the :panda_gazebo:`panda_gazebo <>` simulation.



Submodules
----------
.. toctree::
   :titlesonly:
   :maxdepth: 1

   control_server/index.rst
   control_switcher/index.rst
   group_publisher/index.rst
   moveit_server/index.rst


Package Contents
----------------

Classes
~~~~~~~

.. autoapisummary::

   panda_gazebo.core.PandaControlServer
   panda_gazebo.core.PandaControlSwitcher
   panda_gazebo.core.GroupPublisher
   panda_gazebo.core.PandaMoveItPlannerServer




.. py:class:: PandaControlServer(autofill_traj_positions=False, load_gripper=True, load_set_joint_commands_service=True, load_arm_follow_joint_trajectory_action=False, load_extra_services=False, brute_force_grasping=False, controllers_check_rate=CONTROLLER_INFO_RATE)


   Bases: :py:obj:`object`

   Controller server used to send control commands to the simulated Panda Robot.

   .. attribute:: joint_states

      The current joint states.

      :type: :obj:`sensor_msgs.JointState`

   .. attribute:: arm_joint_positions_threshold

      The current threshold for determining
      whether the arm joint positions are within the given setpoint.

      :type: float

   .. attribute:: arm_joint_efforts_threshold

      The current threshold for determining
      whether the arm joint efforts are within the given setpoint.

      :type: float

   .. attribute:: arm_velocity_threshold

      The current threshold for determining whether
      the arm has zero velocity.

      :type: float

   .. py:property:: controlled_joints

      Returns the joints that can be controlled by a each control type.

      :param control_type: The type of control that is being executed. Options
                           are ``effort``, ``position`` and ``trajectory``.
      :type control_type: str

      :returns:

                A dictionary containing the joints that are controlled when using a
                    given control type
                    (i.e. ``control_type``>``control_group``>``controller``).
      :rtype: dict

   .. py:property:: joint_controllers

      Retrieves the controllers which are currently initialized to work with a
      given joint.

      :returns:

                Dictionary containing the controllers that can control a given panda
                    joint.
      :rtype: dict

   .. py:property:: controllers

      Retrieves info about the loaded controllers.

      :returns: Dictionary with information about the currently loaded controllers.
      :rtype: dict

   .. py:property:: gripper_width

      Returns the gripper width as calculated based on the Panda finger joints.

      :returns: The gripper width.
      :rtype: float


.. py:class:: PandaControlSwitcher(connection_timeout=10, verbose=True, robot_name_space='')


   Bases: :py:obj:`object`

   Used for switching the Panda robot controllers.

   .. attribute:: verbose

      bool
      Boolean specifying whether we want to display log messages during switching.

   .. py:property:: arm_control_type

      Returns the currently active arm control type. Returns empty string when no
      control type is enabled.

   .. py:property:: hand_control_type

      Returns the currently active hand control type. Returns empty string when no
      control type is enabled.

   .. py:method:: wait_for_control_type(control_group, control_type, timeout=None, rate=10)

      Function that can be used to wait till all the controllers used for a given
      'control_group' and 'control_type' are running. Useful 6 when you expect a
      launch file to load certain controllers.

      :param control_group: The control group of which you want the switch the
                            control type. Options are 'hand' or 'arm'.
      :type control_group: str
      :param control_type: The robot control type you want to switch to for the
                           given 'control_group'. Options are: ``trajectory``, ``position`` and
                           ``effort``.
      :type control_type: str
      :param timeout: The function timeout. Defaults to ``None`` meaning
                      the function will wait for ever.
      :type timeout: float, optional
      :param rate: The 'control_type' check rate. Defaults to `10`
                   hz.
      :type rate: int, optional

      :raises TimeoutError: Thrown when the set timeout has passed.


   .. py:method:: switch(control_group, control_type, load_controllers=True, timeout=None, verbose=None)

      Switch Panda robot control type. This function stops all currently running
      controllers and starts the required controllers for a given control type.

      :param control_group: The control group of which you want the switch the
                            control type. Options are 'hand' or 'arm'.
      :type control_group: str
      :param control_type: The robot control type you want to switch to for the
                           given 'control_group'. Options are: ``trajectory``, ``position``
                           and ``effort``.
      :type control_type: str
      :param load_controllers: Try to load the required controllers for a given
                               control_type if they are not yet loaded.
      :type load_controllers: bool
      :param timeout: The timout for switching to a given controller.
                      Defaults to :attr:`self._controller_manager_response_timeout`.
      :type timeout: int, optional
      :param verbose: Whether to display debug log messages. Defaults
                      to verbose value set during the class initiation.
      :type verbose: bool, optional

      :returns:     Contains information about whether the switch operation was successful
                    'success' and the previously used controller 'prev_control_type'.
      :rtype: :obj:`~panda_gazebo.core.control_switcher.ControllerSwitcherResponse`



.. py:class:: GroupPublisher(iterable=[])


   Bases: :py:obj:`list`

   Used for bundling ROS publishers together and publishing
   to these publishers at the same time.

   .. py:method:: publish(messages)

      Publishes a list of messages to the publishers contained on the
      GroupPublisher object. The index of the message corresponds to the
      publisher the message will be published to.

      :param messages: List containing the messages to be published.
      :type messages: list

      :raises ValueError: If something went wrong during publishing.



.. py:class:: PandaMoveItPlannerServer(arm_move_group='panda_arm', arm_ee_link='panda_link8', hand_move_group='hand', load_gripper=True, load_set_ee_pose_service=True, load_extra_services=False)


   Bases: :py:obj:`object`

   Used to control or request information from the Panda Robot. This is done using
   the MoveIt :mod:`moveit_commander` module.

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


