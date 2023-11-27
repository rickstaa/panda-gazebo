"""Module containing some additional helper functions used in the
:panda-gazebo:`panda_gazebo <>` package.
"""
import copy
import sys

import control_msgs.msg as control_msgs
import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from numpy import linalg, nan
from rospy.exceptions import ROSException
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from panda_gazebo.msg import FollowJointTrajectoryGoal


#################################################
# Type conversion functions #####################
#################################################
def joint_state_dict_2_joint_state_msg(joint_state_dict, type="position"):
    """Converts a joint_state dictionary into a JointState msgs.

    Args:
        joint_state_dict (dict): Dictionary specifying joint values for each joint
            (key).
        type (str, optional): The state type. Options are ``velocity``, ``effort`` and
            ``position``. Defaults to "position".

    Returns:
        :obj:`sensor_msgs.msg.JointState`: A JoinState message.
    """
    resp = JointState()
    resp.name = list(joint_state_dict.keys())
    if type.lower() == "velocity":
        resp.velocity = list(joint_state_dict.values())
    elif type.lower() == "effort":
        resp.effort = list(joint_state_dict.values())
    else:
        resp.position = list(joint_state_dict.values())
    return resp


def action_dict_2_joint_trajectory_msg(
    action_dict, create_time_axis=True, time_axis_step=0.01
):
    """Converts an action dictionary into a panda_gazebo ``FollowJointTrajectoryGoal``
    msgs.

    Args:
        action_dict (dict): Dictionary containing actions and joints.
        create_time_axis (bool): Whether you want to automatically create a joint
            trajectory time axis if it is not yet present.
        time_axis_step (float): The size of the time steps used for generating the time
            axis.

    Returns:
        :obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`: New FollowJointTrajectoryGoal
            message.

    Raises:
        ValuesError: When the action_dict is invalid.
    """
    goal_msg = FollowJointTrajectoryGoal()
    goal_msg.create_time_axis = create_time_axis
    goal_msg.time_axis_step = time_axis_step

    # Handle multiple waypoints.
    if all([np.isscalar(item) for item in list(action_dict.values())]):
        waypoints_actions = np.array(
            [
                list(action_dict.values()),
            ]
        )
    elif (
        all([isinstance(item, np.ndarray) for item in action_dict.values()])
        and all([item.ndim == 1 for item in action_dict.values()])
        and all(
            [
                item.shape == list(action_dict.values())[0].shape
                for item in action_dict.values()
            ]
        )
    ):
        waypoints_actions = np.transpose(list(action_dict.values()))
    else:
        raise ValueError(
            "Joint trajectory message could not be created since the action values "
            "in the action dict are invalid. Please make sure that each joint contains "
            "an equal amount of joint commands, one for each waypoint."
        )

    # Create and return joint trajectory message.
    for waypoint in waypoints_actions:
        wp = JointTrajectoryPoint()
        wp.positions = list(waypoint)
        goal_msg.trajectory.joint_names = list(action_dict.keys())
        goal_msg.trajectory.points.append(wp)
    return goal_msg


def panda_action_msg_2_control_msgs_action_msg(panda_action_msg):
    """Converts a panda_gazebo FollowJointTrajectoryActionGoal action message
    into a :control_msgs:`control_msgs/FollowJointTrajectoryGoal
    <html/action/FollowJointTrajectory.html>` action message.

    Args:
        panda_action_msg :obj:`control_msgs.msg.FollowJointTrajectoryGoal`: Panda_gazebo
            follow joint trajectory goal message.

    Returns:
        :obj:`control_msgs.msg.FollowJointTrajectoryGoal`: Control_msgs follow joint
            trajectory goal message
    """
    control_msgs_action_msg = control_msgs.FollowJointTrajectoryGoal()
    control_msgs_action_msg.trajectory = panda_action_msg.trajectory
    control_msgs_action_msg.goal_time_tolerance = panda_action_msg.goal_time_tolerance
    control_msgs_action_msg.goal_tolerance = panda_action_msg.goal_tolerance
    control_msgs_action_msg.path_tolerance = panda_action_msg.path_tolerance
    return control_msgs_action_msg


def controller_list_array_2_dict(controller_list_msgs):
    """Converts a :controller_manager_msgs:`Controller_manager/list_controllers <html/srv/ListControllers.html>`
    message into a controller information dictionary.

    Args:
        controller_list_msgs (:obj:`controller_manager_msgs.srv.ListControllersResponse`):
            Controller_manager/list_controllers service response message.

    Returns:
        dict: Dictionary containing information about all the available controllers.
    """  # noqa: E501
    controller_list_dict = {}
    for controller in controller_list_msgs.controller:
        controller_name = controller.name
        controller_list_dict[controller_name] = copy.deepcopy(controller)
    return controller_list_dict


def translate_actionclient_result_error_code(actionclient_retval):
    """Translates the error code returned by the SimpleActionClient.get_result()
    function into a human readable error message.

    Args:
        actionclient_retval (:obj:`control_msgs.msg.FollowJointTrajectoryResult`): The
            result that is returned by the
            :func:`actionlib.simple_action_client.SimpleActionClient.get_result()`
            function.

    Returns:
        str: Error string that corresponds to the error code.
    """
    if actionclient_retval is not None:
        error_dict = {
            value: attr
            for attr, value in actionclient_retval.__class__.__dict__.items()
            if attr[0] != "_" and all(map(str.isupper, attr.replace("_", "")))
        }
        return (
            error_dict[actionclient_retval.error_code]
            .lower()
            .capitalize()
            .replace("_", " ")
            + "."
            if error_dict[actionclient_retval.error_code] != "SUCCESSFUL"
            else ""
        )
    else:
        return "No goal set"


def translate_moveit_error_code(moveit_error_code):
    """Translates a MoveIt error code object into a human readable error message.

    Args:
        moveit_error_code (:obj:`~moveit_msgs.msg._MoveItErrorCodes.MoveItErrorCodes`):
            The MoveIt error code object

    Returns:
        str: Error string that corresponds to the error code.
    """
    error_dict = {
        value: attr
        for attr, value in moveit_error_code.__class__.__dict__.items()
        if attr[0] != "_" and all(map(str.isupper, attr.replace("_", "")))
    }
    return (
        error_dict[moveit_error_code.val].lower().capitalize().replace("_", " ") + "."
        if error_dict[moveit_error_code.val] != "SUCCESSFUL"
        else ""
    )


#################################################
# List dict and text manipulation functions #####
#################################################
def lower_first_char(string):
    """De-capitalize the first letter of a string.

    Args:
        string (str): The input string.

    Returns:
        str: The de-capitalized string.

    .. note::
        This function is not the exact opposite of the capitalize function of the
        standard library. For example, capitalize('abC') returns Abc rather than AbC.
    """
    if not string:  # Added to handle case where s == None
        return
    else:
        return string[0].lower() + string[1:]


def wrap_space_around(text):
    """Wrap one additional space around text if it is not already present.

    Args:
        text (str): Text

    Returns:
        str: Text with extra spaces around it.
    """
    if text[0] != " " and text[-1] != " ":
        return " " + text + " "
    elif text[0] != " ":
        return " " + text
    elif text[-1] != " ":
        return text + " "
    else:
        return text


def list_2_human_text(input_list, separator=",", end_separator="&"):
    """Function converts a list of values into human readable sentence.

    Example:
        Using this function a list of 4 items ``[item1, item2, item3, item4]`` becomes
        ``item2, item3 and item4``.

    Args:
        input_list (list): A input list.

    Returns:
        str: A human readable string that can be printed.
    """
    # Add spaces around separators if not present.
    separator = wrap_space_around(separator)[1:]
    end_separator = wrap_space_around(end_separator)

    # Create human readable comma deliminated text.
    if isinstance(input_list, list):
        if len(input_list) > 1:
            return (
                separator.join([str(item) for item in input_list[:-1]])
                + end_separator
                + str(input_list[-1])
            )
        if len(input_list) == 0:
            return ""
        else:
            return str(input_list[0])
    if isinstance(input_list, tuple):
        input_list = list(input_list)
        if len(input_list) > 1:
            return (
                separator.join([str(item) for item in input_list[:-1]])
                + end_separator
                + str(input_list[-1])
            )
        if len(input_list) == 0:
            return ""
        else:
            return str(input_list[0])
    else:
        return input_list


def dict_clean(input_dict):
    """Removes empty dictionary keys from a dictionary and returns a cleaned up
    dictionary. Empty meaning an empty list, string or dict or a None value.

    Args:
        input_dict (dict):The input dictionary.

    Returns:
        dict: The cleaned dictionary
    """
    stripped_dict = {}
    for k, v in input_dict.items():
        if isinstance(v, dict):
            v = dict_clean(v)
        if v not in ("", None, {}, []):
            stripped_dict[k] = v
    return stripped_dict


def get_unique_list(input_list, trim=True):
    """Removes non-unique items from a list.

    Args:
        input_list (list): The input list.
        trim (list, optional): Trim empty items. Defaults to ``True``.

    Returns:
        list: The new list containing only unique items.
    """
    if trim:
        return list({item for item in input_list if item != ""})
    else:
        return list({item for item in input_list})


def get_duplicate_list(input_list):
    """Returns the duplicates in a list.

    Args:
        input_list (list): The input list.

    Returns:
        list: The new list containing only the itesm that had duplicates.
    """
    return list(set([x for x in input_list if input_list.count(x) > 1]))


def flatten_list(input_list):
    """Function used to flatten a list containing sublists. It does this by calling
    itself recursively.

    Args:
        input_list (list): A list containing strings or other lists.

    Returns:
        list: The flattened list.
    """
    flattened_list = []
    for list_item in input_list:
        if type(list_item) is list:
            flattened_list.extend(
                flatten_list(list_item)
            )  # NOTE: Calls itself recursively
        else:
            flattened_list.append(list_item)
    return flattened_list


#################################################
# Other functions ###############################
#################################################
def action_server_exists(topic_name):
    """Checks whether a topic contains an action server
    is running.

    Args:
        topic_name (str): Action server topic name.

    Returns:
        bool: Boolean specifying whether the action service exists.
    """
    # Strip action server specific topics from topic name.
    if topic_name.split("/")[-1] in ["cancel", "feedback", "goal", "result", "status"]:
        topic_name = "/".join(topic_name.split("/")[:-1])
    if topic_name[-1] == "/":
        topic_name = topic_name[:-1]

    # Validate if action server topic exists.
    try:
        rospy.wait_for_message("%s/status" % topic_name, GoalStatusArray, timeout=5)
    except ROSException:
        return False

    # Check if topic contains action client.
    exists = False
    for item in rospy.get_published_topics():
        if "%s/status" % topic_name in item[0]:
            if "actionlib_msgs" in item[1]:
                exists = True
            else:
                exists = False
    return exists


def quaternion_norm(quaternion):
    """Calculates the norm of a quaternion.

    Args:
        Quaternion (:obj:`geometry_msgs.msg.Quaternion`): A quaternion.

    Returns:
        float: The norm of the quaternion.
    """
    return linalg.norm([quaternion.x, quaternion.y, quaternion.z, quaternion.w])


def normalize_quaternion(quaternion):
    """Normalizes a given quaternion.

    Args:
        quaternion (:obj:`geometry_msgs.msg.Quaternion`): A quaternion.

    Returns:
        :obj:`geometry_msgs.msg.Quaternion`: The normalized quaternion.
    """
    quaternion = copy.deepcopy(
        quaternion
    )  # Make sure the original object is not changed.
    norm = quaternion_norm(quaternion)

    # Normalize quaternion.
    if norm == nan:
        # test.
        rospy.logwarn(
            "Quaternion could not be normalized since the norm could not be "
            "calculated."
        )
    elif norm == 0.0:  # Transform to identity.
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
    else:
        quaternion.x = quaternion.x / norm
        quaternion.y = quaternion.y / norm
        quaternion.z = quaternion.z / norm
        quaternion.w = quaternion.w / norm
    return quaternion


def vector_norm(vector):
    """Calculates the norm of a vector.

    Args:
        vector (union[list, tuple, np.array]): A vector.

    Returns:
        float: The norm of the vector.

    Raises:
        TypeError: If vector is not of type list or tuple.
    """
    if not isinstance(vector, (list, tuple, np.ndarray)):
        raise TypeError("vector must be of type list, tuple or numpy array")

    vector = np.array(vector)
    return np.linalg.norm(vector)


def normalize_vector(vector, force=True):
    """Normalizes a given vector.

    Args:
        vector (union[list, tuple]): A vector.
        force (bool): Whether to force the vector to have a unit length if the norm is
            zero. Defaults to ``True``.

    Returns:
        list: The normalized vector.

    Raises:
        TypeError: If vector is not of type list or tuple.
    """
    if not isinstance(vector, (list, tuple, np.ndarray)):
        raise TypeError("vector must be of type list or tuple or numpy array")

    vector = np.array(vector)
    norm = vector_norm(vector)
    if norm == 0:
        if force:
            return [0.0, 0.0, 1.0]
        rospy.logwarn("Vector could not be normalized since the norm is zero.")
        return list(vector)
    return list(vector / norm)


def ros_exit_gracefully(shutdown_msg=None, exit_code=0):
    """Shuts down the ROS node wait until it is shutdown and exits the script.

    Args:
        shutdown_msg (str, optional): The shutdown message. Defaults to ``None``.
        exit_code (int, optional): The exit code. Defaults to ``0``.
    """
    if exit_code == 0:
        rospy.loginfo(shutdown_msg)
    else:
        rospy.logerr(shutdown_msg)
    rospy.signal_shutdown(shutdown_msg)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
    sys.exit(exit_code)
