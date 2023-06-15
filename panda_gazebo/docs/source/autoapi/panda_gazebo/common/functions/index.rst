:py:mod:`panda_gazebo.common.functions`
=======================================

.. py:module:: panda_gazebo.common.functions

.. autoapi-nested-parse::

   Module containing some additional functions used in the
   :panda_gazebo:`panda_gazebo <>` package.



Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   panda_gazebo.common.functions.joint_state_dict_2_joint_state_msg
   panda_gazebo.common.functions.action_dict_2_joint_trajectory_msg
   panda_gazebo.common.functions.panda_action_msg_2_control_msgs_action_msg
   panda_gazebo.common.functions.controller_list_array_2_dict
   panda_gazebo.common.functions.translate_actionclient_result_error_code
   panda_gazebo.common.functions.translate_moveit_error_code
   panda_gazebo.common.functions.lower_first_char
   panda_gazebo.common.functions.wrap_space_around
   panda_gazebo.common.functions.list_2_human_text
   panda_gazebo.common.functions.dict_clean
   panda_gazebo.common.functions.get_unique_list
   panda_gazebo.common.functions.get_duplicate_list
   panda_gazebo.common.functions.flatten_list
   panda_gazebo.common.functions.action_server_exists
   panda_gazebo.common.functions.quaternion_norm
   panda_gazebo.common.functions.normalize_quaternion



.. py:function:: joint_state_dict_2_joint_state_msg(joint_state_dict, type='position')

   Converts a joint_state dictionary into a JointState msgs.

   :param joint_state_dict: Dictionary specifying joint values for each joint
                            (key).
   :type joint_state_dict: dict
   :param type: The state type. Options are ``velocity``, ``effort`` and
                ``position``. Defaults to "position".
   :type type: str, optional

   :returns: A JoinState message.
   :rtype: :obj:`sensor_msgs.msg.JointState`


.. py:function:: action_dict_2_joint_trajectory_msg(action_dict, create_time_axis=True, time_axis_step=0.01)

   Converts an action dictionary into a panda_gazebo ``FollowJointTrajectoryGoal``
   msgs.

   :param action_dict: Dictionary containing actions and joints.
   :type action_dict: dict
   :param create_time_axis: Whether you want to automatically create a joint
                            trajectory time axis if it is not yet present.
   :type create_time_axis: bool
   :param time_axis_step: The size of the time steps used for generating the time
                          axis.
   :type time_axis_step: float

   :returns:

             New FollowJointTrajectoryGoal
                 message.
   :rtype: :obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`

   :raises ValuesError: When the action_dict is invalid.


.. py:function:: panda_action_msg_2_control_msgs_action_msg(panda_action_msg)

   Converts a panda_gazebo FollowJointTrajectoryActionGoal action message
   into a :control_msgs:`control_msgs/FollowJointTrajectoryGoal
   <html/action/FollowJointTrajectory.html>` action message.

   :param panda_action_msg :obj:`control_msgs.msg.FollowJointTrajectoryGoal`: Panda_gazebo
                                                                              follow joint trajectory goal message.

   :returns:

             Control_msgs follow joint
                 trajectory goal message
   :rtype: :obj:`control_msgs.msg.FollowJointTrajectoryGoal`


.. py:function:: controller_list_array_2_dict(controller_list_msgs)

   Converts a :controller_manager_msgs:`Controller_manager/list_controllers <html/srv/ListControllers.html>`
   message into a controller information dictionary.

   :param controller_list_msgs: Controller_manager/list_controllers service response message.
   :type controller_list_msgs: :obj:`controller_manager_msgs.srv.ListControllersResponse`

   :returns: Dictionary containing information about all the available controllers.
   :rtype: dict


.. py:function:: translate_actionclient_result_error_code(actionclient_retval)

   Translates the error code returned by the SimpleActionClient.get_result()
   function into a human readable error message.

   :param actionclient_retval: The
                               result that is returned by the
                               :func:`actionlib.simple_action_client.SimpleActionClient.get_result()`
                               function.
   :type actionclient_retval: :obj:`control_msgs.msg.FollowJointTrajectoryResult`

   :returns: Error string that corresponds to the error code.
   :rtype: str


.. py:function:: translate_moveit_error_code(moveit_error_code)

   Translates a MoveIt error code object into a human readable error message.

   :param moveit_error_code: The MoveIt error code object
   :type moveit_error_code: :obj:`~moveit_msgs.msg._MoveItErrorCodes.MoveItErrorCodes`

   :returns: Error string that corresponds to the error code.
   :rtype: str


.. py:function:: lower_first_char(string)

   De-capitalize the first letter of a string.

   :param string: The input string.
   :type string: str

   :returns: The de-capitalized string.
   :rtype: str

   .. note::
       This function is not the exact opposite of the capitalize function of the
       standard library. For example, capitalize('abC') returns Abc rather than AbC.


.. py:function:: wrap_space_around(text)

   Wrap one additional space around text if it is not already present.

   :param text: Text
   :type text: str

   :returns: Text with extra spaces around it.
   :rtype: str


.. py:function:: list_2_human_text(input_list, separator=',', end_separator='&')

   Function converts a list of values into human readable sentence.

   .. rubric:: Example

   Using this function a list of 4 items ``[item1, item2, item3, item4]`` becomes
   ``item2, item3 and item4``.

   :param input_list: A input list.
   :type input_list: list

   :returns: A human readable string that can be printed.
   :rtype: str


.. py:function:: dict_clean(input_dict)

   Removes empty dictionary keys from a dictionary and returns a cleaned up
   dictionary. Empty meaning an empty list, string or dict or a None value.

   :param input_dict: The input dictionary.
   :type input_dict: dict

   :returns: The cleaned dictionary
   :rtype: dict


.. py:function:: get_unique_list(input_list, trim=True)

   Removes non-unique items from a list.

   :param input_list: The input list.
   :type input_list: list
   :param trim: Trim empty items. Defaults to ``True``.
   :type trim: list, optional

   :returns: The new list containing only unique items.
   :rtype: list


.. py:function:: get_duplicate_list(input_list)

   Returns the duplicates in a list.

   :param input_list: The input list.
   :type input_list: list

   :returns: The new list containing only the itesm that had duplicates.
   :rtype: list


.. py:function:: flatten_list(input_list)

   Function used to flatten a list containing sublists. It does this by calling
   itself recursively.

   :param input_list: A list containing strings or other lists.
   :type input_list: list

   :returns: The flattened list.
   :rtype: list


.. py:function:: action_server_exists(topic_name)

   Checks whether a topic contains an action server
   is running.

   :param topic_name: Action server topic name.
   :type topic_name: str

   :returns: Boolean specifying whether the action service exists.
   :rtype: bool


.. py:function:: quaternion_norm(quaternion)

   Calculates the norm of a quaternion.

   :param Quaternion: A quaternion.
   :type Quaternion: :obj:`geometry_msgs.msg.Quaternion`

   :returns: The norm of the quaternion.
   :rtype: float


.. py:function:: normalize_quaternion(quaternion)

   Normalizes a given quaternion.

   :param quaternion: A quaternion.
   :type quaternion: :obj:`geometry_msgs.msg.Quaternion`

   :returns: The normalized quaternion.
   :rtype: :obj:`geometry_msgs.msg.Quaternion`


