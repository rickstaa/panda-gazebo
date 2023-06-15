:py:mod:`panda_gazebo.core.control_server`
==========================================

.. py:module:: panda_gazebo.core.control_server

.. autoapi-nested-parse::

   This server is responsible for controlling the Panda arm. It created several
   (action) services that can be used to send control commands to the Panda Robot arm and
   hand. These services wrap around the original control topics and services created by
   the `franka_ros`_ and `ros_control`_ packages and add extra functionality. They allow
   you to `wait` for control commands to be completed, send incomplete control commands
   and incomplete trajectories (i.e. control commands/trajectories that do not contain
   all the joints).

   Main services:
       * ``get_controlled_joints``
       * ``follow_joint_trajectory``
       * ``set_joint_commands``
       * ``panda_hand/set_gripper_width``

   Main actions:
       * ``panda_arm/follow_joint_trajectory``

   Extra services:
       * ``panda_arm/set_joint_positions``
       * ``panda_arm/set_joint_efforts``

   .. _`franka_ros`: https://github.com/frankaemika/franka_ros
   .. _`ros_control`: https://github.com/ros-controls/ros_control



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   panda_gazebo.core.control_server.PandaControlServer




Attributes
~~~~~~~~~~

.. autoapisummary::

   panda_gazebo.core.control_server.GRASP_EPSILON
   panda_gazebo.core.control_server.GRASP_SPEED
   panda_gazebo.core.control_server.GRASP_FORCE
   panda_gazebo.core.control_server.WAIT_TILL_DONE_TIMEOUT
   panda_gazebo.core.control_server.DIRNAME
   panda_gazebo.core.control_server.PANDA_JOINTS
   panda_gazebo.core.control_server.ARM_POSITION_CONTROLLERS
   panda_gazebo.core.control_server.ARM_EFFORT_CONTROLLERS
   panda_gazebo.core.control_server.ARM_TRAJ_CONTROLLERS
   panda_gazebo.core.control_server.HAND_CONTROLLERS
   panda_gazebo.core.control_server.CONTROLLER_INFO_RATE
   panda_gazebo.core.control_server.CONNECTION_TIMEOUT


.. py:data:: GRASP_EPSILON
   :value: 0.003

   

.. py:data:: GRASP_SPEED
   :value: 0.1

   

.. py:data:: GRASP_FORCE
   :value: 10

   

.. py:data:: WAIT_TILL_DONE_TIMEOUT
   :value: 5

   

.. py:data:: DIRNAME

   

.. py:data:: PANDA_JOINTS

   

.. py:data:: ARM_POSITION_CONTROLLERS
   :value: ['panda_arm_joint1_position_controller', 'panda_arm_joint2_position_controller',...

   

.. py:data:: ARM_EFFORT_CONTROLLERS
   :value: ['panda_arm_joint1_effort_controller', 'panda_arm_joint2_effort_controller',...

   

.. py:data:: ARM_TRAJ_CONTROLLERS
   :value: ['panda_arm_controller']

   

.. py:data:: HAND_CONTROLLERS
   :value: ['franka_gripper']

   

.. py:data:: CONTROLLER_INFO_RATE

   

.. py:data:: CONNECTION_TIMEOUT
   :value: 10

   

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


