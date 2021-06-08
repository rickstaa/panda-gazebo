"""Module containing some additional functions used in the
:panda_gazebo:`panda_gazebo <>` package.
"""

import copy

import control_msgs.msg as control_msgs
import rospy
from actionlib_msgs.msg import GoalStatusArray
from panda_gazebo.msg import FollowJointTrajectoryGoal
from panda_gazebo.srv import SetJointPositionsRequest
from rospy.exceptions import ROSException
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


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


def action_dict_2_joint_trajectory_msg(action_dict):
    """Converts an action dictionary into a FollowJointTrajectoryGoal
    msgs.

    Args:
        action_dict (dict): Dictionary containing actions and joints.

    Returns:
        :obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`: New FollowJointTrajectoryGoal
            message.
    """
    # Initiate waypoints and new trajectory message
    goal_msg = FollowJointTrajectoryGoal()
    goal_msg.create_time_axis = True
    goal_msg.time_axis_step = 0.01
    waypoint = JointTrajectoryPoint()

    # creates waypoint from joint_positions
    waypoint.positions = list(action_dict.values())
    goal_msg.trajectory.joint_names = list(action_dict.keys())

    # Add waypoint to trajectory message and return goal msgs
    goal_msg.trajectory.points.append(waypoint)
    return goal_msg


def panda_action_msg_2_control_msgs_action_msg(panda_action_msg):
    """Converts a panda_gazebo FollowJointTrajectoryActionGoal action message
    into a
    :control_msgs:`control_msgs/FollowJointTrajectoryGoal
    <html/action/FollowJointTrajectory.html>` action message.

    Args:
        panda_action_msg :obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`: Panda_gazebo
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


def joint_positions_2_follow_joint_trajectory_goal(joint_positions, time_from_start=1):
    """Converts a dictionary of joint_positions into a FollowJointTrajectoryGoal
    msgs.

    Args:
        joint_positions (union[dict, :obj:`panda_gazebo.msg.SetJointPositionsRequest`]):
            Dictionary or message containing the joint positions of each of the robot
            joints.
        time_from_start (dict, optional): The time from the start at which the joint
            position has to be achieved. Defaults to  1 sec.

    Returns:
        :obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`):New FollowJointTrajectoryGoal
            message.
    """
    # Initiate waypoints and new trajectory message
    goal_msg = FollowJointTrajectoryGoal()
    waypoint = JointTrajectoryPoint()
    waypoint.time_from_start.secs = time_from_start

    # creates waypoint from joint_posisitions
    if isinstance(joint_positions, SetJointPositionsRequest):
        waypoint.positions = joint_positions.joint_positions
        goal_msg.trajectory.joint_names = joint_positions.joint_names
    elif isinstance(joint_positions, dict):
        waypoint.positions = list(joint_positions.values())
        goal_msg.trajectory.joint_names = list(joint_positions.keys())
    else:
        TypeError(
            "FollowJointTrajectory message could not be created since the "
            "joint_positions argument has the %s type while the "
            "'joint_positions_2_follow_joint_trajectory_goal' function only accepts "
            "a dictionary or a SetJointPositions message." % type(joint_positions)
        )

    # Add waypoint to trajectory message and return goal msgs
    goal_msg.trajectory.points.append(waypoint)
    return goal_msg


def controller_list_array_2_dict(controller_list_msgs):
    """Converts a :controller_manager_msgs:`Controller_manager/list_controllers
    <html/srv/ListControllers.html>` message into a controller information dictionary.

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


def translate_gripper_width_2_finger_joint_commands(input_dict):
    """Translate any ``gripper_width`` keys that are present in the action dictionary
    into the corresponding finger joint control commands which are used by the
    controllers.

    Args:
        input_dict (dict): Dictionary containing the desired actions.

    Returns:
        dict: Action dictionary in which the gripper_width is translated to finger joint
            commands.
    """
    input_dict = input_dict.copy()  # Ensure that changes stay within this scope

    # Translate any 'gripper_width' keys into Panda finger joint command keys
    if isinstance(input_dict, dict):
        if "gripper_width" in input_dict.keys():  # If dictionary contains commands
            finger_position = input_dict["gripper_width"] / 2.0
            del input_dict["gripper_width"]
            input_dict["panda_finger_joint1"] = finger_position
            input_dict["panda_finger_joint2"] = finger_position
        elif (
            "gripper_width_min" in input_dict.keys()
            or "gripper_width_max" in input_dict.keys()
        ):  # If dictionary contains bounds commands
            if "gripper_width_min" in input_dict.keys():  # Translate min
                finger_position_min = input_dict["gripper_width_min"] / 2.0
                del input_dict["gripper_width_min"]
                input_dict["panda_finger_joint1_min"] = finger_position_min
                input_dict["panda_finger_joint2_min"] = finger_position_min
            if "gripper_width_max" in input_dict.keys():  # Translate max
                finger_position_max = input_dict["gripper_width_max"] / 2.0
                del input_dict["gripper_width_max"]
                input_dict["panda_finger_joint1_max"] = finger_position_max
                input_dict["panda_finger_joint2_max"] = finger_position_max
    else:
        raise TypeError(
            "Input argument has the wrong type the"
            "'translate_gripper_width_2_finger_joint_commands'"
            "function only takes a dictionary."
        )
    return input_dict


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


def list_2_human_text(input_list, seperator=",", end_seperator="&"):
    """Function converts a list of values into human readable sentence.

    Example:
        Using this function a list of 4 items ``[item1, item2, item3, item4]`` becomes
        ``item2, item3 and item4``.

    Args:
        input_list (list): A input list.

    Returns:
        str: A human readable string that can be printed.
    """
    # Add spaces around separators if not present
    seperator = wrap_space_around(seperator)[1:]
    end_seperator = wrap_space_around(end_seperator)

    # Create human readable comma deliminated text
    if isinstance(input_list, list):
        if len(input_list) > 1:
            return (
                seperator.join([str(item) for item in input_list[:-1]])
                + end_seperator
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
                seperator.join([str(item) for item in input_list[:-1]])
                + end_seperator
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
        if v not in (u"", None, {}, []):
            stripped_dict[k] = v
    return stripped_dict


def get_unique_list(input_list):
    """Removes non-unique items from a list.

    Args:
        input_list (list): The input list.

    Returns:
        list: The new list containing only unique items.
    """
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
    # Strip action server specific topics from topic name
    if topic_name.split("/")[-1] in ["cancel", "feedback", "goal", "result", "status"]:
        topic_name = "/".join(topic_name.split("/")[:-1])
    if topic_name[-1] == "/":
        topic_name = topic_name[:-1]

    # Validate if action server topic exists
    try:
        rospy.wait_for_message("%s/status" % topic_name, GoalStatusArray, timeout=5)
    except ROSException:
        return False

    # Check if topic contains action client
    exists = False
    for item in rospy.get_published_topics():
        if "%s/status" % topic_name in item[0]:
            if "actionlib_msgs" in item[1]:
                exists = True
            else:
                exists = False
    return exists
