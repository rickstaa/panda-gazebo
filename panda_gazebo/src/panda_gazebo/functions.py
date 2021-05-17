"""Module containing some additional functions used in the
:panda_openai_sim:`panda_openai_sim <>` package.
"""

# Main python imports
import copy
import os
import glob

from panda_openai_sim.extras import EulerAngles

# Import ROS python packages
import rospy
from rospy.exceptions import ROSException
from tf.transformations import euler_from_quaternion

# ROS msgs and srvs
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import JointTrajectoryPoint
import control_msgs.msg as control_msgs

from panda_openai_sim.msg import FollowJointTrajectoryGoal
from panda_openai_sim.srv import SetJointPositionsRequest


#################################################
# Type conversion functions #####################
#################################################
def action_dict_2_joint_trajectory_msg(action_dict):
    """Converts an action dictionary into a FollowJointTrajectoryGoal
    msgs.

    Parameters
    ----------
    action_dict : dict
        Dictionary containing actions and joints.

    Returns
    -------
    panda_openai_sim.msg.FollowJointTrajectoryGoal
        New FollowJointTrajectoryGoal message.
    """

    # Initiate waypoints and new trajectory message
    goal_msg = FollowJointTrajectoryGoal()
    goal_msg.create_time_axis = True
    goal_msg.time_axis_step = 0.01
    waypoint = JointTrajectoryPoint()

    # creates waypoint from joint_positions
    waypoint.positions = list(action_dict.values())
    goal_msg.trajectory.joint_names = list(action_dict.keys())

    # Add waypoint to trajectory message
    goal_msg.trajectory.points.append(waypoint)

    # Return goal msgs
    return goal_msg


def panda_action_msg_2_control_msgs_action_msg(panda_action_msg):
    """Converts a panda_openai_sim FollowJointTrajectoryActionGoal action message
    into a
    :control_msgs:`control_msgs/FollowJointTrajectoryGoal
    <html/action/FollowJointTrajectory.html>` action message.

    Parameters
    ----------
    panda_action_msg : panda_openai_sim.msg.FollowJointTrajectoryGoal
        panda_openai_sim follow joint trajectory goal message.

    Returns
    -------
    control_msgs.msg.FollowJointTrajectoryGoal
        Control_msgs follow joint trajectory goal message
    """

    # Fill new control_msgs.msg.FollowJointTrajectoryGoal message
    control_msgs_action_msg = control_msgs.FollowJointTrajectoryGoal()
    control_msgs_action_msg.trajectory = panda_action_msg.trajectory
    control_msgs_action_msg.goal_time_tolerance = panda_action_msg.goal_time_tolerance
    control_msgs_action_msg.goal_tolerance = panda_action_msg.goal_tolerance
    control_msgs_action_msg.path_tolerance = panda_action_msg.path_tolerance
    return control_msgs_action_msg


def joint_positions_2_follow_joint_trajectory_goal(joint_positions, time_from_start=1):
    """Converts a dictionary of joint_positions into a FollowJointTrajectoryGoal
    msgs.

    Parameters
    ----------
    joint_positions : dict or panda_openai_sim.msg.SetJointPositionsRequest
        Dictionary or message containing the joint positions of each of the robot
        joints.
    time_from_start : dict, optional
        The time from the start at which the joint position has to be achieved, by
        default 1 sec.

    Returns
    -------
    panda_openai_sim.msg.FollowJointTrajectoryGoal
        New FollowJointTrajectoryGoal message.
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

    # Add waypoint to trajectory message
    goal_msg.trajectory.points.append(waypoint)

    # Return goal msgs
    return goal_msg


def model_state_msg_2_link_state_dict(link_state_msgs):
    """Converts the a :gazebo_msgs:`gazebo_msgs/ModelState <html/msg/ModelState.html>`
    message into a panda_state dictionary. Contrary to the original ModelState message,
    in the model_state dictionary the poses and twists are grouped per link/model.

    Parameters
    ----------
    link_state_msgs : gazebo_msgs.msg.ModelState
        A ModelState message.

    Returns
    -------
    dict
        A panda_openai_sim model_state dictionary.
    """

    # Create controller_list dictionary
    model_state_dict = {}
    for (joint_name, position, twist) in zip(
        link_state_msgs.name, link_state_msgs.pose, link_state_msgs.twist
    ):
        model_state_dict[joint_name] = {}
        model_state_dict[joint_name]["pose"] = copy.deepcopy(position)
        model_state_dict[joint_name]["twist"] = copy.deepcopy(twist)

    # Return dictionary
    return model_state_dict


def controller_list_array_2_dict(controller_list_msgs):
    """Converts a :controller_manager_msgs:`Controller_manager/list_controllers
    <html/srv/ListControllers.html>` message into a controller information dictionary.

    Parameters
    ----------
    controller_list_msgs : controller_manager_msgs.srv.ListControllersResponse
        Controller_manager/list_controllers service response message.

    Returns
    -------
    dict
        Dictionary containing information about all the available controllers.
    """

    # Create controller_list dictionary
    controller_list_dict = {}
    for controller in controller_list_msgs.controller:
        controller_name = controller.name
        controller_list_dict[controller_name] = copy.deepcopy(controller)

    # Return dictionary
    return controller_list_dict


def pose_dict_2_pose_msg(pose_dict):
    """Create a :geometry_msgs:`geometry_msgs.msg.Pose<html/msg/Pose.html>` message out
    of a panda_openai_sim pose dictionary ``{x, y, z, rx, ry, rz, rw}``.

    Parameters
    ----------
    pose_dict : dict
        Dict containing the object position ``{x, y, z}`` and orientation
        ``{rx, ry, rz, rw}``
    """

    # Create pose message out of a panda_openai_sim pose dict
    pose_msg = Pose()
    pose_msg.position.x = pose_dict["x"]
    pose_msg.position.y = pose_dict["y"]
    pose_msg.position.z = pose_dict["z"]
    pose_msg.orientation.x = pose_dict["rx"]
    pose_msg.orientation.y = pose_dict["ry"]
    pose_msg.orientation.z = pose_dict["rz"]
    pose_msg.orientation.w = pose_dict["rw"]

    # Return pose_msg
    return pose_msg


def pose_msg_2_pose_dict(pose_msg):
    """Create a panda_openai_sim pose dictionary ``{x, y, z, rx, ry, rz, rw}`` out of a
    :geometry_msgs:`geometry_msgs.msg.Pose<html/msg/Pose.html>` message.

    Parameters
    ----------
    pose_msg : geometry_msgs.msg.Pose
        A pose message
    """
    # Create dictionary out of pose message
    pose_dict = {
        "x": pose_msg.position.x,
        "y": pose_msg.position.y,
        "z": pose_msg.position.z,
        "rx": pose_msg.orientation.x,
        "ry": pose_msg.orientation.y,
        "rz": pose_msg.orientation.z,
        "rw": pose_msg.orientation.w,
    }
    return pose_dict


def translate_actionclient_result_error_code(actionclient_retval):
    """Translates the error code returned by the SimpleActionClient.get_result()
    function into a human readable error message.

    Parameters
    ----------
    actionclient_retval : control_msgs.msg.FollowJointTrajectoryResult
        The result that is returned by the
        actionlib.simple_action_client.SimpleActionClient.get_result() function.

    Returns
    -------
    str
        Error string that corresponds to the error code.
    """

    # Create error dictionary
    error_dict = {
        value: attr
        for attr, value in actionclient_retval.__class__.__dict__.items()
        if attr[0] != "_" and all(map(str.isupper, attr.replace("_", "")))
    }

    # Create error code message
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

    Parameters
    ----------
    input_dict : dict
        Dictionary containing the desired actions.

    Returns
    -------
    dict
        Action dictionary in which the gripper_width is translated to finger joint
        commands.
    """

    # ensure that we don't change the action outside of this scope
    input_dict = input_dict.copy()

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

        # Throw invalid type error
        raise TypeError(
            "Input argument has the wrong type the"
            "'translate_gripper_width_2_finger_joint_commands'"
            "function only takes a dictionary."
        )

    # Return translated list/dic
    return input_dict


#################################################
# List dict and text manipulation functions #####
#################################################
def lower_first_char(string):
    """De-capitalize the first letter of a string.

    Parameters
    ----------
    string : str
        The input string.

    Returns
    -------
    string
        The de-capitalized string.

    Note
    ---------
        This function is not the exact opposite of the capitalize function of the
        standard library. For example, capitalize('abC') returns Abc rather than AbC.
    """

    if not string:  # Added to handle case where s == None
        return
    else:
        return string[0].lower() + string[1:]


def wrap_space_around(text):
    """Wrap one additional space around text if it is not already present.

    Parameters
    ----------
    text : str
        Text

    Returns
    -------
    str
        Text with extra spaces around it.
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

    Parameters
    ----------
    input_list : list
        A input list.

    Returns
    -------
    str
        A human readable string that can be printed.
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


def log_pose_dict(qpose, header="q_pose", level="DEBUG"):
    """Prints the qpose in a more human readably format.

    Parameters
    ----------
    qpose_dict : dict
        A dictionary containing the generalized robot pose.
    level : str
        The log level you want to use (info, warn or debug).
    header : str
        The header you want to display above the dictionary elements.
    """

    # Validate qpose message
    if not isinstance(qpose, dict):
        rospy.logwarn(
            "Qpose could not be printed in a human readable format as it is of type %s"
            "while the log_qpose function expects type 'dict'. As a result the qpose "
            "is printed in its original format."
        ) % type(qpose)
        qpose_msg = qpose
    else:

        # Convert qpose dictionary to human readable format
        qpose_msg = (
            header
            + ":\n  "
            + "  ".join(
                [str(key) + ": " + str(val) + "\n" for key, val in qpose.items()]
            )[:-1]
        )

    # Check log level and log qpose
    if level.lower() == "warn":
        rospy.logwarn(qpose_msg)
    elif level.lower() == "info":
        rospy.loginfo(qpose_msg)
    else:
        rospy.logdebug(qpose_msg)


def split_dict(input_dict, *args):
    """Split a dictionary into smaller dictionaries based on the keys.

    Example
    -----------

    .. code-block:: python

        split_dict_list = split_dict(input_dict,["first_dict_key1","first_dict_key2"],
        ["second_dict_key1", "second_dict_key2"])

    Parameters
    ----------
    input_dict : dict
        Input dictionary.
    *args : list
        Lists containing the keys you want to have in the successive dictionaries.

    Returns
    -------
    list
        A list containing the splitted dictionaries.
    """

    split_dict_list = []
    for split_list_item in args:
        split_dict_list.append(
            {key: val for key, val in input_dict.items() if key in split_list_item}
        )
    return split_dict_list


def split_bounds_dict(bounds_dict):
    """Splits the bounding region dictionary into two separate bounding dictionaries,
    one for the ``ee_pose`` and one fore the ``joint_pose``.

    Parameters
    ----------
    bounds_dict : dict
        Original bounds dictionary.

    Returns
    -------
    dict, dict
        One ee_pose bounding region dictionary and one joint_pose bounding region
        dictionary.
    """

    # Split bounds dict into two separate dictionaries one for the ee_pose bounds and
    # one for the joint_pose bounds.
    split_dict_list = split_dict(
        bounds_dict,
        ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"],
        [
            "panda_joint1_min",
            "panda_joint2_min",
            "panda_joint3_min",
            "panda_joint4_min",
            "panda_joint5_min",
            "panda_joint6_min",
            "panda_joint7_min",
            "panda_finger_joint1_min",
            "panda_finger_joint1_max",
            "panda_finger_joint2_min",
            "panda_finger_joint2_max",
            "gripper_width_min",
            "gripper_width_max",
        ],
    )
    return split_dict_list[0], split_dict_list[1]


def split_pose_dict(pose_dict):
    """Splits a pose dictionary into two separate pose dictionaries, one for the
    ``ee_pose`` and one fore the ``joint_pose``.

    Parameters
    ----------
    bounding_region : dict
        Original bounds dictionary.

    Returns
    -------
    dict, dict
        One ee_pose dictionary and one joint_pose dictionary.
    """

    # Split bounds dict into two separate dictionaries one for the ee_pose and one for
    # the joint_pose.
    split_dict_list = split_dict(
        pose_dict,
        ["x", "y", "z", "rx", "ry", "rz", "rw"],
        ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"],
    )
    return split_dict_list[0], split_dict_list[1]


def merge_two_dicts(x, y):
    """Given two dicts, merge them into a new dict as a shallow copy.

    Parameters
    ----------
    x : dict
        The first dictionary.
    y : dict
        The second dictionary.

    Returns
    -------
    dict
        The new merged dictionary.
    """
    z = x.copy()
    z.update(y)
    return z


def dict_clean(input_dict):
    """Removes empty dictionary keys from a dictionary and returns a cleaned up
    dictionary. Empty meaning an empty list, string or dict or a None value.

    Parameters
    ----------
    input_dict : dict
        The input dictionary.

    Returns
    -------
    dict
        The cleaned dictionary
    """

    # Strip dictionary from empty keys
    stripped_dict = {}
    for k, v in input_dict.items():
        if isinstance(v, dict):
            v = dict_clean(v)
        if v not in (u"", None, {}, []):
            stripped_dict[k] = v
    return stripped_dict


def get_unique_list(input_list):
    """Removes non-unique items from a list.

    Parameters
    ----------
    list : list
        The input list.

    Returns
    -------
    list
        The new list containing only unique items.
    """

    return list({item for item in input_list})


def get_duplicate_list(input_list):
    """Returns the duplicates in a list.

    Parameters
    ----------
    list : list
        The input list.

    Returns
    -------
    list
        The new list containing only the itesm that had duplicates.
    """

    return list(set([x for x in input_list if input_list.count(x) > 1]))


def flatten_list(input_list):
    """Function used to flatten a list containing sublists. It does this by calling
    itself recursively.

    Parameters
    ----------
    input_list : list of lists
        A list containing strings or other lists.

    Returns
    -------
    list
        The flattened list.
    """

    # Convert list of list to flattened lists
    flattened_list = []
    for list_item in input_list:
        if type(list_item) is list:
            flattened_list.extend(
                flatten_list(list_item)
            )  # NOTE: Calls itself recursively
        else:
            flattened_list.append(list_item)

    # Return flattened list
    return flattened_list


#################################################
# Argument validation functions #################
#################################################
def has_invalid_type(variable, variable_types, items_types=None, depth=0):
    """Validates whether a variable or its attributes has an invalid type.

    Parameters
    ----------
    variable :
        The variable you want to check.
    variable_types : tuple
        The type the variable can have.
    items_types : tuple
        The types the dictionary or list values can have.

    Returns
    --------
    bool, depth, invalid_type
        A tuple containing whether the variable has an invalid type, the maximum depth
        at which a type was invalid, and the types that were invalid.
    """

    # If one type was given make tuple
    if isinstance(variable_types, type):
        variable_types = (variable_types,)
    if isinstance(items_types, type):
        items_types = (items_types,)

    # Check if variable type is valid
    if type(variable) in variable_types:

        # Check list or dictionary value types are valid
        if items_types:  # If items_types == None we are at the deepest level
            if isinstance(variable, dict):

                # Check if the dictionary values are of the right type
                depth += 1
                invalid_types = []
                for key, val in variable.items():
                    if type(val) in [dict, list]:
                        retval, depth, invalid_type = has_invalid_type(
                            val,
                            variable_types=items_types,
                            items_types=items_types,
                            depth=depth,
                        )
                    else:
                        retval, depth, invalid_type = has_invalid_type(
                            val, variable_types=items_types, depth=depth
                        )
                    if retval:  # If invalid type was found
                        if invalid_type not in invalid_types:  # If not already in list
                            invalid_types.append(invalid_type)
                return retval, depth, flatten_list(invalid_types)
            elif isinstance(variable, list):

                # Check if the list values are of the right type
                depth += 1
                invalid_types = []
                for val in variable:
                    if type(val) in [dict, list]:
                        retval, depth, invalid_type = has_invalid_type(
                            val,
                            variable_types=items_types,
                            items_types=items_types,
                            depth=depth,
                        )
                    else:
                        retval, depth, invalid_type = has_invalid_type(
                            val, variable_types=items_types, depth=depth
                        )
                    if retval:  # If invalid type was found
                        if invalid_type not in invalid_types:  # If not already in list
                            invalid_types.append(invalid_type)
                return retval, depth, flatten_list(invalid_types)
        else:

            # Return type not invalid bool, depth and type
            return False, depth, []
    else:
        # Return type invalid bool, depth and type
        return True, depth, type(variable)


def contains_keys(input_dict, required_keys, exclusive=True):
    """Function used to check if a dictionary contains the required keys. If the
    ``required_keys`` argument contains a nested list it checks whether at least one of
    the nested_list elements is present.

    Parameters
    ----------
    input_dict : dict
        The input dictionary.
    required_keys : list
        List containing the keys you want to check.
    exclusive : bool, optional
        Whether the dictionary can contain other keys than those in the 'required_keys'
        argument, by default False.

    Returns
    -------
    bool, list, list
        A bool specifying whether invalid keys are, a list containing the keys that
        were missing and a list containing the keys that were found in addition to the
        required keys.
    """

    # Check if dictionary contains missing keys
    missing_keys = []
    for key in required_keys:

        # If nested list check if one of the keys is present
        if isinstance(key, list):
            found_keys = [
                nested_key
                for nested_key in key
                if nested_key in list(input_dict.keys())
            ]
            if len(found_keys) == 0:
                missing_keys.extend(key)
        else:
            if key not in list(input_dict.keys()):
                missing_keys.append(key)

    # Check if the dictionary contains extra keys
    extra_keys = [
        key for key in input_dict.keys() if key not in flatten_list(required_keys)
    ]

    # Return result
    if len(missing_keys) > 0:
        return False, missing_keys, extra_keys
    elif exclusive and len(extra_keys) > 0:
        return False, missing_keys, extra_keys
    else:
        return True, missing_keys, extra_keys


def has_invalid_value(variable, valid_values):
    """Checks whether a string or list contains invalid values.

    Parameters
    ----------
    variable : list, str
        Input variable
    valid_values : list
        The values that are correct.

    Returns
    -------
    bool, list
        A bool specifying whether invalid values were found and a list that contains
        the invalid values if they were found.
    """

    # Check if list/string contains/has (a) valid value.
    if isinstance(variable, str):
        if variable not in valid_values:
            return True, variable
    elif isinstance(variable, list):
        invalid_values = [item for item in variable if item not in valid_values]
        if len(invalid_values) > 0:
            return True, invalid_values
    else:
        rospy.logwarn(
            "Variable could not be checked for invalid values as type '%s' "
            "is not supported." % type(variable)
        )
        return False, variable

    # Return success bool
    return False, []


#################################################
# Other functions ###############################
#################################################
def find_gazebo_model_path(model_name, model_folder_path, extension=""):
    """Finds the path of the ``sdf`` or ``urdf`` file that belongs to a given
    ``model_name``. This is done by searching in the ``model_folder_path`` folder.

    Parameters
    ----------
    model_name : srt
        The name of the model for which you want to find the path.
    model_folder_path : str
        The path of the folder that contains the gazebo models.
    extension : str, optional
        The model path extension, by default ``""``.

    Returns
    -------
    str, str
        The path where the ``sdf`` or ``urdf`` model file can be found and the
        extension of the model file. If not file was found the model file path is
        returned empty.
    """

    # Add dot to extension if needed
    if extension != "" and extension[0] != ".":
        extension = "." + extension

    # Try to find the model path for a given model_name
    model_folder = os.path.join(model_folder_path, model_name)
    if os.path.isdir(model_folder):  # Check if model_name folder exists
        if extension != "":  # Find based on extension

            # Check if sdf or urdf exists with the model_name name
            model_path = glob.glob(os.path.join(model_folder, "model" + extension))
            if model_path:  # If found
                return model_path[0]
            else:
                rospy.logwarn(
                    "Model path for '%s' could not be found. Please check if the"
                    "'model.sdf' or 'model.urdf' file exist in the model directory "
                    "'%s'." % (model_name, model_folder)
                )
                return "", extension[1:]
        else:  # no extension given

            # Look for sdf or urdf files
            model_path_sdf = glob.glob(os.path.join(model_folder, "model" + ".sdf"))
            model_path_urdf = glob.glob(os.path.join(model_folder, "model" + ".urdf"))

            # Check which extension was found
            if model_path_sdf and model_path_urdf:
                rospy.logwarn(
                    "Model path for '%s' could not be retrieved as both an 'model.sdf' "
                    "and a 'model.urdf' file are present. Please make sure that only "
                    "one is model file is present in the '%s' folder or use the "
                    "extension argument." % (model_name, model_folder)
                )
                return "", ""
            elif model_path_sdf:
                return model_path_sdf[0], "sdf"
            elif model_path_urdf:
                return model_path_urdf[0], "urdf"
            else:
                rospy.logwarn(
                    "Model path for '%s' could not be found. Please check if the"
                    "'model.sdf' or 'model.urdf' file exist in the model directory "
                    "'%s'." % (model_name, model_folder)
                )
                return "", ""
    else:  # Display warning
        rospy.logwarn(
            "Model path for '%s' could not be found. Please check if the 'model.sdf' "
            "or 'model.urdf' file exist in the model directory '%s'."
            % (model_name, model_folder)
        )
        return "", ""


def action_server_exists(topic_name):
    """Checks whether a topic contains an action server
    is running.

    Parameters
    ----------
    topic_name : str
        Action server topic name.

    Returns
    -------
    bool
        Boolean specifying whether the action service exists.
    """

    # Strip action server specific topics from topic name
    if topic_name.split("/")[-1] in ["cancel", "feedback", "goal", "result", "status"]:
        topic_name = "/".join(topic_name.split("/")[:-1])

    # Remove trailing /
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


def get_orientation_euler(quaternion):
    """Converts pose (position, orientation) to euler angles.

    Parameters
    ----------
    quaternion : geometry_msgs.Pose
        Input quaternion

    Returns
    -------
    panda_openai_sim.EulerAngles
        Object containing the yaw (z), pitch (y) and roll (z) euler angles.
    """

    # Convert quaternion to euler
    orientation_list = [
        quaternion.orientation.x,
        quaternion.orientation.y,
        quaternion.orientation.z,
        quaternion.orientation.w,
    ]
    euler_resp = euler_from_quaternion(orientation_list, "rzyx")

    # Convert list to euler object
    euler = EulerAngles()
    euler.y = euler_resp[0]
    euler.p = euler_resp[1]
    euler.r = euler_resp[2]
    return euler
