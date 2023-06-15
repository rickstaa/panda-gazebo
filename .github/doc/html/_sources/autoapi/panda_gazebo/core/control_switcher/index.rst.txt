:py:mod:`panda_gazebo.core.control_switcher`
============================================

.. py:module:: panda_gazebo.core.control_switcher

.. autoapi-nested-parse::

   This class is responsible for switching the control type that is used for
   controlling the Panda Robot robot ``arm``. It serves as a wrapper aroundthe services
   created by the ROS `controller_manager <https://wiki.ros.orgcontroller_manager>`_ class.

   Control types:
       * `trajectory <https://wiki.ros.org/joint_trajectory_controller/>`_
       * `position <https://wiki.ros.org/position_controllers/>`_
       * `effort <https://wiki.ros.org/effort_controllers/>`_



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   panda_gazebo.core.control_switcher.ControllerSwitcherResponse
   panda_gazebo.core.control_switcher.PandaControlSwitcher




Attributes
~~~~~~~~~~

.. autoapisummary::

   panda_gazebo.core.control_switcher.ARM_CONTROLLERS
   panda_gazebo.core.control_switcher.HAND_CONTROLLERS
   panda_gazebo.core.control_switcher.CONTROLLER_DICT


.. py:data:: ARM_CONTROLLERS

   

.. py:data:: HAND_CONTROLLERS

   

.. py:data:: CONTROLLER_DICT

   

.. py:class:: ControllerSwitcherResponse(success=True, prev_control_type='')


   Class used for returning the result of the ControllerSwitcher.switch method.

   .. attribute:: success

      Specifies whether the switch operation was successful.

      :type: bool

   .. attribute:: prev_control_type

      The previous used control type.

      :type: str


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
      'control_group' and 'control_type' are running. Usefull 6 when you expect a
      launch file to load certain controllers.

      :param control_group: The control group of which you want the switch the
                            control type. Options are 'hand' or 'arm'.
      :type control_group: str
      :param control_type: The robot control type you want to switch to for the
                           given 'control_group'. Options are: ``trajectory``, ``position`` and
                           ``effort``.
      :type control_type: str
      :param timeout: The function timeout. Defaults to `None` meaning
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
                      Defaults to :py:attr:`self._controller_manager_response_timeout`.
      :type timeout: int, optional
      :param verbose: Whether to display debug log messages. Defaults
                      to verbose value set during the class initiation.
      :type verbose: bool, optional

      :returns:     Contains information about whether the switch operation was successful
                    'success' and the previously used controller 'prev_control_type'.
      :rtype: :obj:`~panda_gazebo.core.control_switcher.ControllerSwitcherResponse`



