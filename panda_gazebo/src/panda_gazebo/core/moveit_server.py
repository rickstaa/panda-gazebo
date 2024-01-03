#! /usr/bin/env python
"""A ros server that creates several of MoveIt services which can be used to control
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
"""
import re
import sys

import moveit_commander
import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander.exception import MoveItCommanderException
from moveit_msgs.msg import DisplayTrajectory
from rospy.exceptions import ROSException
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from panda_gazebo.cfg import MoveitServerConfig
from panda_gazebo.common.helpers import (
    flatten_list,
    get_duplicate_list,
    get_unique_list,
    joint_state_dict_2_joint_state_msg,
    load_panda_joint_limits,
    lower_first_char,
    normalize_quaternion,
    normalize_vector,
    quaternion_norm,
    ros_exit_gracefully,
    translate_moveit_error_code,
)
from panda_gazebo.exceptions import InputMessageInvalidError, JointLimitsInvalidError
from panda_gazebo.srv import (
    AddBox,
    AddBoxResponse,
    AddPlane,
    AddPlaneResponse,
    GetEe,
    GetEePose,
    GetEePoseJointConfig,
    GetEePoseJointConfigResponse,
    GetEePoseResponse,
    GetEeResponse,
    GetEeRpy,
    GetEeRpyResponse,
    GetMoveItControlledJoints,
    GetMoveItControlledJointsResponse,
    GetRandomEePose,
    GetRandomEePoseResponse,
    GetRandomJointPositions,
    GetRandomJointPositionsResponse,
    SetEe,
    SetEePose,
    SetEePoseResponse,
    SetEeResponse,
    SetJointPositions,
    SetJointPositionsResponse,
)

# The maximum number times the get_random_ee_pose service tries to sample from the
# bounding region before ignoring it.
MAX_RANDOM_SAMPLES = 5


class PandaMoveItPlannerServer(object):
    """Used to control or request information from the Panda Robot. This is done using
    the MoveIt :mod:`moveit_commander` module.

    Attributes:
        robot (:obj:`moveit_commander.robot.RobotCommander`): The MoveIt robot
            commander object.
        scene (:obj:`moveit_commander.planning_scene_interface.PlanningSceneInterface`):
            The MoveIt robot scene commander object.
        move_group_arm (:obj:`moveit_commander.move_group.MoveGroupCommander`):
            The MoveIt arm move group object.
        move_group_hand (:obj:`moveit_commander.move_group.MoveGroupCommander`):
            The MoveIt hand move group object.
        ee_pose_target (:obj:`geometry_msgs.msg.Pose`): The last set ee pose.
        joint_positions_target (:obj:`dict`): Dictionary containing the last Panda arm
            and hand joint positions setpoint.
    """

    def __init__(  # noqa: C901
        self,
        arm_move_group="panda_arm",
        arm_ee_link="panda_link8",
        hand_move_group="panda_hand",
        load_gripper=True,
        load_set_ee_pose_service=True,
        load_extra_services=False,
    ):
        """Initialise PandaMoveItPlannerServer object.

        Args:
            arm_move_group (str, optional): The name of the move group you want to use
                for controlling the Panda arm. Defaults to ``panda_arm``.
            arm_ee_link (str, optional): The end effector you want MoveIt to use when
                controlling the Panda arm. Defaults to ``panda_link8``.
            hand_move_group (str, optional): The name of the move group you want to use
                for controlling the Panda hand. Defaults to ``panda_hand``.
            load_gripper (boolean, optional): Whether we also want to load the gripper
                control services. Defaults to ``True``.
            load_set_ee_pose_service (boolean, optional): Whether the set ee pose
                service should be loaded. This service is used by the
                :ros-gazebo-gym:`ros_gazebo_gym <>` package when the control type is
                set to ``trajectory``. Defaults, to ``True``.
            load_extra_services (bool, optional): Whether to load extra services that
                are not used by the
                :ros-gazebo-gym:`ros_gazebo_gym <>` package. Defaults to ``False``.
        """
        self._load_gripper = load_gripper

        # Initialise MoveIt/Robot/Scene commanders
        rospy.logdebug("Initialise MoveIt Robot/Scene commanders.")
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
        except Exception as e:
            if "invalid robot mode" in e.args[0]:
                err_msg = (
                    "Shutting down '%s' because robot_description was not found."
                    % rospy.get_name()
                )
            else:
                err_msg = "Shutting down '%s' because %s" % (
                    rospy.get_name(),
                    e.args[0],
                )
            ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

        # Initialise group commanders.
        try:
            rospy.logdebug("Initialise MoveIt Panda arm commander.")
            self.move_group_arm = moveit_commander.MoveGroupCommander(arm_move_group)
        except Exception as e:
            if len(re.findall("Group '(.*)' was not found", e.args[0])) >= 1:
                err_msg = (
                    "Shutting down '%s' because Panda arm move group '%s' was not "
                    "found."
                    % (
                        rospy.get_name(),
                        arm_move_group,
                    )
                )
            else:
                err_msg = "Shutting down '%s' because %s" % (
                    rospy.get_name(),
                    e.args[0],
                )
            ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
        if self._load_gripper:
            try:
                rospy.logdebug("Initialise MoveIt Panda hand commander.")
                self.move_group_hand = moveit_commander.MoveGroupCommander(
                    hand_move_group
                )
            except Exception as e:
                if len(re.findall("Group '(.*)' was not found", e.args[0])) >= 1:
                    rospy.logwarn(
                        "Gripper services could not be loaded since the "
                        f"'{hand_move_group}' was not found."
                    )
                    self._load_gripper = False
                else:
                    err_msg = "Shutting down '%s' because %s" % (
                        rospy.get_name(),
                        e.args[0],
                    )
                    ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
        self.move_group_arm.set_end_effector_link(arm_ee_link)

        # Create display trajectory publisher.
        self._display_trajectory_publisher = rospy.Publisher(
            "move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=10,
        )

        # Retrieve parameters and create dynamic reconfigure server.
        # NOTE: Used to change the max velocity and acceleration scaling that is used
        # by the MoveIt control server.
        self._get_params()
        self._dyn_reconfigure_srv = Server(MoveitServerConfig, self._dyn_reconfigure_cb)

        ########################################
        # Create node services services ########
        ########################################

        # Create main PandaMoveItPlannerServer services.
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        if load_set_ee_pose_service:
            rospy.logdebug(
                "Creating '%s/panda_arm/set_ee_pose' service." % rospy.get_name()
            )
            self._arm_set_ee_pose_srv = rospy.Service(
                "%s/panda_arm/set_ee_pose" % rospy.get_name().split("/")[-1],
                SetEePose,
                self._arm_set_ee_pose_callback,
            )
        rospy.logdebug(
            "Creating '%s/panda_arm/get_ee_pose_joint_config' service."
            % rospy.get_name()
        )
        self._arm_get_ee_pose_joint_config_srv = rospy.Service(
            "%s/panda_arm/get_ee_pose_joint_config" % rospy.get_name().split("/")[-1],
            GetEePoseJointConfig,
            self._arm_get_ee_pose_joint_config,
        )
        rospy.logdebug(
            "Creating '%s/get_random_joint_positions' service." % rospy.get_name()
        )
        self._get_random_joints_positions_srv = rospy.Service(
            "%s/get_random_joint_positions" % rospy.get_name().split("/")[-1],
            GetRandomJointPositions,
            self._get_random_joint_positions_callback,
        )
        rospy.logdebug("Creating '%s/get_random_ee_pose' service." % rospy.get_name())
        self._get_random_ee_pose_srv = rospy.Service(
            "%s/get_random_ee_pose" % rospy.get_name().split("/")[-1],
            GetRandomEePose,
            self._get_random_ee_pose_callback,
        )
        rospy.logdebug("Creating '%s/set_joint_positions' service." % rospy.get_name())
        self._set_joint_positions_srv = rospy.Service(
            "%s/set_joint_positions" % rospy.get_name().split("/")[-1],
            SetJointPositions,
            self._set_joint_positions_callback,
        )
        if load_extra_services:
            rospy.logdebug("Creating '%s/panda_arm/get_ee' service." % rospy.get_name())
            self._arm_get_ee = rospy.Service(
                "%s/panda_arm/get_ee" % rospy.get_name().split("/")[-1],
                GetEe,
                self._arm_get_ee_callback,
            )
            rospy.logdebug("Creating '%s/panda_arm/set_ee' service." % rospy.get_name())
            self._arm_set_ee = rospy.Service(
                "%s/panda_arm/set_ee" % rospy.get_name().split("/")[-1],
                SetEe,
                self._arm_set_ee_callback,
            )
            rospy.logdebug(
                "Creating '%s/panda_arm/get_ee_pose' service." % rospy.get_name()
            )
            self._arm_get_ee_pose_srv = rospy.Service(
                "%s/panda_arm/get_ee_pose" % rospy.get_name().split("/")[-1],
                GetEePose,
                self._arm_get_ee_pose_callback,
            )
            rospy.logdebug(
                "Creating '%s/panda_arm/get_ee_rpy' service." % rospy.get_name()
            )
            self._arm_get_ee_rpy_srv = rospy.Service(
                "%s/panda_arm/get_ee_rpy" % rospy.get_name().split("/")[-1],
                GetEeRpy,
                self._arm_get_ee_rpy_callback,
            )
            rospy.logdebug(
                "Creating '%s/get_controlled_joints' service." % rospy.get_name()
            )
            self._get_controlled_joints_srv = rospy.Service(
                "%s/get_controlled_joints" % rospy.get_name().split("/")[-1],
                GetMoveItControlledJoints,
                self._get_controlled_joints_cb,
            )
            rospy.logdebug(
                "Creating '%s/panda_arm/set_joint_positions' service."
                % rospy.get_name()
            )
            self._arm_set_joint_positions_srv = rospy.Service(
                "%s/panda_arm/set_joint_positions" % rospy.get_name().split("/")[-1],
                SetJointPositions,
                self._arm_set_joint_positions_callback,
            )
            if self._load_gripper:
                rospy.logdebug(
                    "Creating '%s/panda_hand/set_joint_positions' service."
                    % rospy.get_name()
                )
                self._hand_set_joint_positions_srv = rospy.Service(
                    "%s/panda_hand/set_joint_positions"
                    % rospy.get_name().split("/")[-1],
                    SetJointPositions,
                    self._hand_set_joint_positions_callback,
                )

        # Planning scene services.
        rospy.logdebug(
            "Creating '%s/planning_scene/add_box' service." % rospy.get_name()
        )
        self._scene_add_box_srv = rospy.Service(
            "%s/planning_scene/add_box" % rospy.get_name().split("/")[-1],
            AddBox,
            self._scene_add_box_callback,
        )
        rospy.logdebug(
            "Creating '%s/planning_scene/add_plane' service." % rospy.get_name()
        )
        self._scene_add_plane_srv = rospy.Service(
            "%s/planning_scene/add_plane" % rospy.get_name().split("/")[-1],
            AddPlane,
            self._scene_add_plane_callback,
        )

        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        # Initialise service msgs.
        self.ee_pose_target = Pose()
        self.joint_positions_target = {}

        ########################################
        # Retrieve controlled joints and joint #
        # state masks. #########################
        ########################################
        self._joint_states = None
        while self._joint_states is None and not rospy.is_shutdown():
            try:
                self._joint_states = rospy.wait_for_message(
                    "joint_states", JointState, timeout=1.0
                )
            except ROSException:
                rospy.logwarn(
                    "Current joint_states not ready yet, retrying for getting "
                    "'joint_states'."
                )
        arm_joints = self.move_group_arm.get_active_joints()
        hand_joints = (
            self.move_group_hand.get_active_joints() if self._load_gripper else []
        )
        controlled_joints = (
            [arm_joints, hand_joints]
            if self._joint_states.name[0] in arm_joints
            else [hand_joints, arm_joints]
        )
        self._controlled_joints_dict = {
            "arm": arm_joints,
            "hand": hand_joints,
            "both": flatten_list(controlled_joints),
        }

        # Retrieve joint state mask.
        # NOTE: Used to split joint_states into arm and hand
        self._arm_states_mask = [
            joint in self._controlled_joints_dict["arm"]
            for joint in self._joint_states.name
        ]
        if self._load_gripper:
            self._hand_states_mask = [
                joint in self._controlled_joints_dict["hand"]
                for joint in self._joint_states.name
            ]

        # Retrieve panda joint limits.
        self._joint_limits = load_panda_joint_limits()
        if not self._joint_limits:
            rospy.logerr(
                "Unable to load Panda joint limits. Ensure 'joint_limits.yaml' from "
                "'franka_description' is loaded in 'put_robot_in_world.launch'."
            )
            ros_exit_gracefully(shutdown_msg="Shutting down.", exit_code=1)

    ################################################
    # Helper functions #############################
    ################################################
    def _get_params(self):
        """Retrieve optional 'moveit_server' parameters from the parameter server.

        .. note::
            Can be used to specify parameters of the 'moveit_server'. You can for
            example set the 'max_velocity_scaling_factor' used by the
            :class:`~moveit_commander.MoveGroupCommander` by setting the
            'panda_moveit_planner_server/max_velocity_scaling_factor' and
            'panda_moveit_planner_server/max_acceleration_scaling_factor' topics.
        """
        self._max_velocity_scaling = rospy.get_param(
            "~max_velocity_scaling_factor", None
        )
        self._max_acceleration_scaling = rospy.get_param(
            "~max_acceleration_scaling_factor", None
        )

    def _link_exists(self, link_name):
        """Function checks whether a given link exists in the robot_description.

        Args:
            link_name (str): Name of link you want to check.

        Returns:
            bool: Boolean specifying whether the link exists.
        """
        return link_name in self.robot.get_link_names()

    def _split_positions_moveit_setpoint(self, positions_moveit_setpoint):
        """Splits a combined MoveIt arm and hand joint position setpoint dictionary into
        separate dictionaries for each control group.

        Args:
            positions_moveit_setpoint (dict): Dictionary that contains the combined arm
                and hand joint positions setpoint.

        Returns:
            dict: Dictionary containing dictionaries for arm and hand joint positions
                setpoint respectively.
        """
        return {
            "arm": {
                joint_name: joint_position
                for joint_name, joint_position in positions_moveit_setpoint.items()
                if joint_name in self._controlled_joints_dict["arm"]
            },
            "hand": {
                joint_name: joint_position
                for joint_name, joint_position in positions_moveit_setpoint.items()
                if joint_name in self._controlled_joints_dict["hand"]
            },
        }

    def _set_joint_value_target(self, group, positions):
        """Sets the joint value target for a control group.

        Args:
            group (str): The control group ("arm" or "hand").
            positions (dict): The joint positions.

        Returns:
            bool: True if successful, False otherwise.
            str: The error message, or an empty string if successful.
        """
        try:
            rospy.logdebug(
                f"{group.capitalize()} joint positions setpoint: {positions}"
            )
            getattr(self, f"move_group_{group}").set_joint_value_target(positions)
            return True, ""
        except MoveItCommanderException as e:
            return False, e.args[0]

    def _execute(self, control_group="both", wait=True):  # noqa: C901
        """Plan and execute a trajectory/pose or orientation setpoints.

        Args:
            control_group (str, optional): The robot control group for which you want
                to execute the control. Options are ``arm`` or ``hand`` or ``both``.
                Defaults to ``both``.
            wait (boolean, optional): Whether to wait on the control to be executed.

        Returns:
        (tuple): tuple containing:

            - list: List specifying whether the arm and/or hand execution was successful.
                If ``control_group == "both"`` then ``["arm_success", "hand_success"]``.
            - str: The error message, or an empty string if successful.
        """
        control_group = control_group.lower()
        if control_group not in ["arm", "hand", "both"]:
            rospy.logwarn(
                f"Control group '{control_group}' does not exist. Please specify a "
                "valid control group. Valid values are 'arm', 'hand' or 'both'."
            )
            return [False]

        # Plan and execute trajectory.
        retval, groups, err_msg = [], [], None
        if control_group in ["arm", "both"]:
            groups.append(self.move_group_arm)
        if control_group in ["hand", "both"]:
            groups.append(self.move_group_hand if self._load_gripper else None)
        for group in groups:
            if group is None:
                rospy.logwarn("Hand commands not executed since gripper is not loaded.")
                retval.append(False)
                continue

            (plan_retval, _, _, error_code) = group.plan()
            if plan_retval:
                group_retval = group.go(wait=wait)
                if wait:
                    group.stop()
            else:
                err_msg = translate_moveit_error_code(error_code)
                rospy.logwarn(
                    f"No plan found for the current {group.get_name()} setpoints "
                    f"since '{err_msg}'."
                )
                group_retval = False
            retval.append(group_retval)

        return retval, err_msg

    def _create_positions_moveit_setpoints(  # noqa: C901
        self, set_joint_positions_req, control_group
    ):
        """Converts a :class:`~panda_gazebo.srv.SetJointPositionsRequest` message into
        a :class:`~moveit_commander.move_group.MoveGroupCommander` hand and/or arm joint
        positions setpoint dictionary.

        Args:
            set_joint_positions_req (:class:`~panda_gazebo.srv.SetJointPositionsRequest`):  # noqa: E501
                The service request message we want to convert.
            control_group (str): The control group for which we want to create the joint
                positions setpoint. Options are ``arm`` or ``hand`` or ``both``.

        Returns:
            dict: Dictionary containing the arm and/or hand joint positions setpoint.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when no
                joint position setpoint dictionary could be created from the given input
                message.
        """
        joint_names = set_joint_positions_req.joint_names
        joint_positions = list(set_joint_positions_req.joint_positions)
        controlled_joints = self._controlled_joints_dict[control_group]
        control_group_str = "arm and hand" if control_group == "both" else control_group

        # Raise error if no joint positions were given.
        if len(joint_positions) == 0:
            raise InputMessageInvalidError(
                message=("No joint positions were given."),
                log_message=(
                    "No joint positions were given. Please specify at least one joint "
                    "position."
                ),
            )

        # Handle joint positions request without joint names.
        controlled_joints_size = len(controlled_joints)
        if len(joint_names) == 0:
            # Display warning if to many or to few joint positions were given.
            joint_positions_count = len(joint_positions)
            if joint_positions_count != controlled_joints_size:
                action = (
                    "extra joint positions were ignored"
                    if joint_positions_count > controlled_joints_size
                    else (
                        "missing joint positions were set to the current joint "
                        "positions"
                    )
                )
                rospy.logwarn(
                    f"You specified {joint_positions_count} joint positions while the "
                    f"Panda robot {control_group_str} contains "
                    f"{controlled_joints_size} active joints. As a result, the "
                    f"{action}."
                )

            # Split joint positions into arm and hand joint positions and return.
            joint_positions_setpoint = dict(zip(controlled_joints, joint_positions))
            if control_group != "both":
                return {control_group: joint_positions_setpoint}
            return self._split_positions_moveit_setpoint(joint_positions_setpoint)

        # Check if duplicate joint names were given.
        duplicate_joint_names = get_duplicate_list(joint_names)
        if duplicate_joint_names:
            rospy.logwarn(
                f"You specified duplicate joint names ({duplicate_joint_names}). "
                "As a result, the duplicate joint names are ignored."
            )
            joint_names = get_unique_list(joint_names)

        # Apply gripper width to first hand joint.
        # NOTE: Only the first joint is controlled directly by MoveIT.
        if "gripper_width" in joint_names:
            gripper_width_index = joint_names.index("gripper_width")
            joint_positions[gripper_width_index] /= 2
            joint_names[gripper_width_index] = self._controlled_joints_dict["hand"][0]

        # Validate joint_names.
        invalid_joint_names = [
            joint_name
            for joint_name in joint_names
            if joint_name not in controlled_joints
        ]
        if invalid_joint_names:
            joint_names = [
                joint_name
                for joint_name in joint_names
                if joint_name not in invalid_joint_names
            ]
            joint_positions = [
                joint_position
                for joint_position, joint_name in zip(joint_positions, joint_names)
                if joint_name not in invalid_joint_names
            ]

            # Raise error if no valid joint names were given and log warning if invalid
            # joint names were given.
            word_forms = (
                "Joint" if len(invalid_joint_names) == 1 else "Joints",
                "was" if len(invalid_joint_names) == 1 else "were",
                "it is" if len(invalid_joint_names) == 1 else "they are",
            )
            logwarn_msg = (
                f"{word_forms[0]} {invalid_joint_names} {word_forms[1]} ignored since "
                f"{word_forms[2]} invalid. Valid joint names for controlling the panda "
                f"{control_group_str} are {controlled_joints}."
            )
            if len(joint_positions) == 0:
                raise InputMessageInvalidError(
                    message=(
                        "No valid joint names were given. Please specify at least one "
                        "valid joint name."
                    ),
                    log_message=f"No valid joint names were given. {logwarn_msg}",
                )
            rospy.logwarn(logwarn_msg)

        # Check if each joint name has a corresponding joint position.
        if len(joint_names) != len(joint_positions):
            joint_positions_count = len(joint_positions)
            joint_names_count = len(joint_names)
            joint_positions_str = (
                "joint position" if joint_positions_count == 1 else "joint positions"
            )
            joint_names_str = "joint" if joint_names_count == 1 else "joints"
            logwarn_msg = (
                f"You specified {joint_positions_count} {joint_positions_str} while "
                "the 'joint_names' field of the 'panda_gazebo/SetJointPositions' "
                f"message contains {joint_names_count} {joint_names_str}. Please make "
                "sure you supply a joint position for each joint contained in the "
                "'joint_names' field."
            )
            raise InputMessageInvalidError(
                message=(
                    "Joint_names and joint_positions fields of the input message are "
                    "of different lengths."
                ),
                log_message=logwarn_msg,
                joint_positions_command_length=joint_positions_count,
                joint_names_length=joint_names_count,
            )

        # Split joint positions into arm and hand joint positions and return.
        joint_positions_setpoint = dict(zip(joint_names, joint_positions))
        if control_group != "both":
            return {control_group: joint_positions_setpoint}
        return self._split_positions_moveit_setpoint(joint_positions_setpoint)

    def _get_valid_joint_limits(self, joint_limits_names, joint_limits_values):
        """Returns a dictionary containing the valid joint limits.

        Args:
            joint_limits_names (list): List containing the joint limit names.
            joint_limits_values (list): List containing the joint limit values.

        Returns:
            dict: Dictionary containing the valid joint limits.
        """
        # Check if limit names and limit values are of equal length.
        joint_positions_command_length = len(joint_limits_values)
        joint_names_length = len(joint_limits_names)
        if joint_positions_command_length != joint_names_length:
            rospy.logwarn(
                "Joint limits ignored as the number of joints ({}) is "
                "unequal to the number of limit values ({}).".format(
                    joint_names_length, joint_positions_command_length
                )
            )
            return {}

        # Check if joint names are valid.
        valid_joint_names = flatten_list(
            [
                [joint + "_min", joint + "_max"]
                for joint in self._controlled_joints_dict["both"]
            ]
        )
        invalid_joint_names = [
            name for name in joint_limits_names if name not in valid_joint_names
        ]
        if len(invalid_joint_names) != 0:
            singular = len(invalid_joint_names) == 1
            warn_strings = (
                (
                    "limit",
                    invalid_joint_names[0],
                    "was",
                    "it is not a valid joint limit",
                )
                if singular
                else (
                    "limits",
                    ", ".join(invalid_joint_names),
                    "were",
                    "they are not valid joint limits",
                )
            )
            rospy.logwarn(
                f"Joint {warn_strings[0]} '{warn_strings[1]}' "
                f"{warn_strings[2]} ignored since {warn_strings[3]}. Valid "
                f"values are '{valid_joint_names}'."
            )
            joint_limits_names = [
                joint_name
                for joint_name in joint_limits_names
                if joint_name not in invalid_joint_names
            ]
            joint_limits_values = [
                joint_limit
                for joint_limit_name, joint_limit in zip(
                    joint_limits_names,
                    joint_limits_values,
                )
                if joint_limit_name not in invalid_joint_names
            ]

        # Ensure that joint limits have a min and max value.
        limited_joints = get_unique_list(
            [
                name.replace("_min", "").replace("_max", "")
                for name in joint_limits_names
            ]
        )
        required_joint_limits = flatten_list(
            [[joint + "_min", joint + "_max"] for joint in limited_joints]
        )
        missing_joint_limits = [
            required_joint_limit
            for required_joint_limit in required_joint_limits
            if required_joint_limit not in joint_limits_names
        ]
        if missing_joint_limits:
            ignored_joint_limit_joint = get_unique_list(
                [
                    missing_joint_limit.replace("_min", "").replace("_max", "")
                    for missing_joint_limit in missing_joint_limits
                ]
            )
            singular = len(ignored_joint_limit_joint) == 1
            joint_or_joints = "joint" if singular else "joints"
            ignored_joint_limit_joint_str = (
                f"'{ignored_joint_limit_joint[0]}'"
                if singular
                else ", ".join(ignored_joint_limit_joint)
            )
            rospy.logwarn(
                f"Joint limits specified on {joint_or_joints} "
                f"{ignored_joint_limit_joint_str} were ignored as both a min and max "
                "limit need to be specified."
            )
            joint_limits_names = [
                name
                for name in joint_limits_names
                if name.replace("_min", "").replace("_max", "")
                not in ignored_joint_limit_joint
            ]
            joint_limits_values = [
                joint_limit
                for joint_limit_name, joint_limit in zip(
                    joint_limits_names,
                    joint_limits_values,
                )
                if joint_limit_name.replace("_min", "").replace("_max", "")
                not in ignored_joint_limit_joint
            ]

        return {
            name: value
            for name, value in zip(
                joint_limits_names,
                joint_limits_values,
            )
        }

    def _get_random_joint_values(self, group):
        """Returns a dictionary containing random joint values for a given control
        group.

        Args:
            group (str): The control group for which you want to retrieve random joint
                values. Options are ``arm`` or ``hand``.

        Returns:
            dict: Dictionary containing the random joint values.
        """
        group = group.lower()
        if group not in ["arm", "hand"]:
            raise ValueError(
                f"Invalid group '{group}'. Valid values are 'arm' or 'hand'."
            )

        # Retrieve random joint values.
        try:
            return dict(
                zip(
                    self._controlled_joints_dict[group],
                    getattr(self, f"move_group_{group}").get_random_joint_values(),
                )
            )
        except MoveItCommanderException:
            return None

    def _get_bounded_random_joint_values(  # noqa: C901
        self, joint_limits, max_attempts
    ):
        """Returns a dictionary containing bounded random joint values for given joint
        limits.

        Args:
            joint_limits (dict): Dictionary containing the joint limits.
            max_attempts (int): The maximum number of attempts to sample valid random
                joint values.

        Returns:
            (tuple): tuple containing:
                - **random_arm_joint_values** (:obj:`dict`): Dictionary containing the
                    (bounded) random arm joint values. Is ``None`` if no valid random
                    arm joint values could be sampled.
                - **random_hand_joint_values** (:obj:`dict`): Dictionary containing the
                    (bounded) random hand joint values. Is ``None`` if the gripper is not
                    loaded or no valid random hand joint values could be sampled.
        """
        # Retrieve unbounded random joint values.
        rospy.logdebug("Retrieving unbounded random joint values.")
        random_arm_joint_values = self._get_random_joint_values("arm")
        random_hand_joint_values = (
            self._get_random_joint_values("hand") if self._load_gripper else None
        )
        if not joint_limits:
            return random_arm_joint_values, random_hand_joint_values

        # Notify the user if joint limits are not valid with the panda joint limits.
        rospy.logdebug("Checking if joint limits are valid.")
        invalid_joint_limits = [
            joint_name
            for joint_name in joint_limits
            if (
                joint_name.endswith("_min")
                and joint_limits[joint_name] < self._joint_limits[joint_name]
            )
            or (
                joint_name.endswith("_max")
                and joint_limits[joint_name] > self._joint_limits[joint_name]
            )
        ]
        if invalid_joint_limits:
            verb = "is" if len(invalid_joint_limits) == 1 else "are"
            log_msg = (
                f"Joint limits '{invalid_joint_limits}' {verb} not valid since they are "
                "outside the Panda joint limits specified in 'joint_limits.yaml'. "
                f"Valid joint limits are '{self._joint_limits}'."
            )
            raise JointLimitsInvalidError(
                message=("Invalid joint limits were given."),
                log_message=log_msg,
            )

        # If joint limits were given, retrieve bounded random joint values.
        joint_commands_valid = {
            "arm": False,
            "hand": not self._load_gripper,
        }
        move_groups = {
            "arm": self.move_group_arm,
        }
        random_joint_values = {
            "arm": random_arm_joint_values,
        }
        if self._load_gripper:
            move_groups["hand"] = self.move_group_hand
            random_joint_values["hand"] = random_hand_joint_values
        unique_joints = get_unique_list(
            [
                joint_limit_names.replace("_min", "").replace("_max", "")
                for joint_limit_names in joint_limits.keys()
            ]
        )
        for attempt in range(max_attempts):
            # Retrieve valid random joint values.
            rospy.logdebug(
                f"Retrieving valid random joint values. Attempt {attempt + 1} of "
                f"{max_attempts}."
            )
            for joint in unique_joints:
                limb = (
                    "hand" if joint in self._controlled_joints_dict["hand"] else "arm"
                )
                if joint_commands_valid[limb] or not random_joint_values[limb]:
                    continue
                if joint_limits[joint + "_min"] != joint_limits[joint + "_max"]:
                    random_joint_values[limb][joint] = np.random.uniform(
                        joint_limits[joint + "_min"],
                        joint_limits[joint + "_max"],
                    )
                else:
                    joint_commands_valid[limb] = True

            # Check if joint values are valid.
            rospy.logdebug("Checking if joint values are valid.")
            for limb in ["arm", "hand"]:
                if joint_commands_valid[limb] or not random_joint_values[limb]:
                    continue
                try:
                    plan = move_groups[limb].plan(
                        joint_state_dict_2_joint_state_msg(random_joint_values[limb])
                    )
                    joint_commands_valid[limb] = (
                        len(plan[1].joint_trajectory.points) != 0
                    )
                except MoveItCommanderException:
                    joint_commands_valid[limb] = False
            if all(joint_commands_valid.values()):
                break
            elif attempt == max_attempts - 1:
                rospy.logwarn(
                    "Failed to sample valid random joint positions within the maximum "
                    f"number of attempts ({max_attempts})."
                )
                random_arm_joint_values, random_hand_joint_values = None, None
                break

            rospy.logwarn(
                "Failed to sample valid random joint positions from the bounding "
                "region. Trying again."
            )

        return random_arm_joint_values, random_hand_joint_values

    ###############################################
    # Service callback functions ##################
    ###############################################
    def _dyn_reconfigure_cb(self, config, level):  # noqa: C901
        """Dynamic reconfigure callback function. Can be used to update the
        ``max_velocity_scaling_factor`` and ``max_acceleration_scaling_factor`` used by
        the MoveIt control server.

        Args:
            config (:obj:`dynamic_reconfigure.encoding.Config`): The current dynamic
                reconfigure configuration object.
            level (int): Bitmask that gives information about which parameter has been
                changed.

        Returns:
            :obj:`~dynamic_reconfigure.encoding.Config`: Modified dynamic reconfigure
                configuration object.
        """
        rospy.logdebug(
            (
                "Reconfigure Request: max_vel_scaling - {max_velocity_scaling_factor} "
                "max_acc_scaling - {max_acceleration_scaling_factor}"
            ).format(**config)
        )
        if level == -1:
            # Update initial values to user supplied parameters.
            if self._max_velocity_scaling:
                if self._max_velocity_scaling < 0.0 or self._max_velocity_scaling > 1.0:
                    rospy.logwarn(
                        "Max velocity scaling factor was clipped since it was not "
                        "between 0.01 and 1.0."
                    )
                    self._max_velocity_scaling = np.clip(0.0, 1e-2, 1.0)
                config["max_velocity_scaling_factor"] = self._max_velocity_scaling
            if self._max_acceleration_scaling:
                if (
                    self._max_acceleration_scaling < 0.0
                    or self._max_acceleration_scaling > 1.0
                ):
                    rospy.logwarn(
                        "Max acceleration scaling factor was clipped since it was not "
                        "between 0.01 and 1.0."
                    )
                    self._max_acceleration_scaling = np.clip(0.0, 1e-2, 1.0)
                config[
                    "max_acceleration_scaling_factor"
                ] = self._max_acceleration_scaling

            # Set initial scaling factors.
            self.move_group_arm.set_max_velocity_scaling_factor(
                config["max_velocity_scaling_factor"]
            )
            self.move_group_arm.set_max_acceleration_scaling_factor(
                config["max_acceleration_scaling_factor"]
            )
            if self._load_gripper:
                self.move_group_hand.set_max_velocity_scaling_factor(
                    config["max_velocity_scaling_factor"]
                )
                self.move_group_hand.set_max_acceleration_scaling_factor(
                    config["max_acceleration_scaling_factor"]
                )
        elif level == 0:
            # Update move group velocity settings.
            self.move_group_arm.set_max_velocity_scaling_factor(
                config["max_velocity_scaling_factor"]
            )
            if self._load_gripper:
                self.move_group_hand.set_max_velocity_scaling_factor(
                    config["max_velocity_scaling_factor"]
                )
        elif level == 1:
            # Update move group acceleration settings.
            self.move_group_arm.set_max_acceleration_scaling_factor(
                config["max_acceleration_scaling_factor"]
            )
            if self._load_gripper:
                self.move_group_hand.set_max_acceleration_scaling_factor(
                    config["max_acceleration_scaling_factor"]
                )
        return config

    def _arm_get_ee_pose_joint_config(self, get_ee_pose_joint_configuration):
        """Request a set of joint configurations that lead to a given end-effector
        (EE) pose.

        Args:
            set_ee_pose_req :obj:`geometry_msgs.msg.Pose`: The trajectory you want the
                EE to follow.

        Returns:
            :obj:`panda_gazebo.srv.SetEePoseResponse`: Response message containing (
                success bool, message).
        """
        max_attempts = (
            get_ee_pose_joint_configuration.attempts
            if get_ee_pose_joint_configuration.attempts != 0.0
            else MAX_RANDOM_SAMPLES
        )

        # Make sure quaternion is normalized.
        if quaternion_norm(get_ee_pose_joint_configuration.pose.orientation) != 1.0:
            rospy.logwarn(
                "The quaternion in the set ee pose was normalized since MoveIt expects "
                "normalized quaternions."
            )
            get_ee_pose_joint_configuration.pose.orientation = normalize_quaternion(
                get_ee_pose_joint_configuration.pose.orientation
            )

        # Retrieve joint configurations.
        pose_target = get_ee_pose_joint_configuration.pose
        resp = GetEePoseJointConfigResponse()
        n_sample = 0
        while (
            n_sample < max_attempts
        ):  # Continue till joint positions are valid or max samples size.
            rospy.logdebug("Retrieving joint configuration for given EE pose.")
            try:
                retval, plan, _, _ = self.move_group_arm.plan(pose_target)
            except MoveItCommanderException:
                retval = False

            # Check if joint config was retrieved.
            if retval:
                resp.joint_names = plan.joint_trajectory.joint_names
                resp.joint_positions = list(plan.joint_trajectory.points[-1].positions)
                resp.success = True
                resp.message = "Everything went OK"
                break
            else:
                rospy.logwarn(
                    "Failed to retrieve joint configuration for given EE pose. Trying "
                    f"again ({n_sample})."
                )
                n_sample += 1

        # Return response.
        if n_sample >= max_attempts:
            rospy.logwarn(
                "Joint configuration could not be retrieved for given EE pose within "
                f"the maximum number of attempts ({max_attempts})."
            )
            resp.success = False
            resp.message = "Joint configuration could not be retrieved for Ee pose"
        return resp

    def _arm_set_ee_pose_callback(self, set_ee_pose_req):
        """Request the Panda arm to control to a given end effector (EE) pose.

        Args:
            set_ee_pose_req :obj:`geometry_msgs.msg.Pose`: The trajectory you want the
                EE to follow.

        Returns:
            :obj:`panda_gazebo.srv.SetEePoseResponse`: Response message containing (
                success bool, message).
        """
        # Make sure quaternion is normalized.
        if quaternion_norm(set_ee_pose_req.pose.orientation) != 1.0:
            rospy.logwarn(
                "The quaternion in the set ee pose was normalized since MoveIt expects "
                "normalized quaternions."
            )
            set_ee_pose_req.pose.orientation = normalize_quaternion(
                set_ee_pose_req.pose.orientation
            )

        # Fill trajectory message.
        rospy.logdebug("Setting ee pose.")
        resp = SetEePoseResponse()
        self.ee_pose_target = set_ee_pose_req.pose

        # Send trajectory message and return response.
        try:
            self.move_group_arm.set_pose_target(self.ee_pose_target)
            retval, err_msg = self._execute(control_group="arm")
            self.move_group_arm.clear_pose_targets()

            # Check if setpoint execution was successful.
            if not all(retval):
                resp.success = False
                resp.message = (
                    "Ee pose could not be set since MoveIt "
                    f"{lower_first_char(err_msg)}."
                )
            else:
                resp.success = True
                resp.message = "Everything went OK"
        except MoveItCommanderException as e:
            rospy.logwarn(e.args[0])
            resp.success = False
            resp.message = e.args[0]
        return resp

    def _set_joint_positions_callback(self, set_joint_positions_req):  # noqa: C901
        """Request the Panda arm and hand to go to a given joint angle and gripper
        position.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionRequest`):
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionResponse`: Response message
                containing (success bool, message).
        """
        rospy.logdebug("Setting joint position targets.")

        # Create moveit_commander command.
        resp = SetJointPositionsResponse()
        try:
            moveit_commander_commands = self._create_positions_moveit_setpoints(
                set_joint_positions_req, control_group="both"
            )
        except InputMessageInvalidError as e:
            logwarn_msg = "Panda robot joint positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.args[0]
            return resp

        # Set joint positions setpoint.
        self.joint_positions_target = moveit_commander_commands
        success, failed_groups = True, []
        for group in ["arm", "hand"]:
            if group == "hand" and not self._load_gripper:
                continue
            group_success, group_error_msg = self._set_joint_value_target(
                group, moveit_commander_commands[group]
            )
            if not group_success:
                success = False
                failed_groups.append(group)
                rospy.logwarn(
                    f"Setting {group} joint position targets failed since there was an "
                    f"{lower_first_char(group_error_msg)}"
                )

        # Print error message if an error occurred and return.
        if not success:
            failed_groups_str = (
                " and ".join(failed_groups)
                if len(failed_groups) > 1
                else failed_groups[0]
            )
            resp.success = False
            resp.message = "Failed to set joint positions for %s." % failed_groups_str
            return resp

        # Execute setpoints.
        rospy.logdebug("Executing joint positions setpoint.")
        try:
            retval, err_msg = self._execute()
            if not all(retval):
                resp.success = False
                resp.message = (
                    f"Joint position setpoint could not executed since {err_msg}."
                )
            else:
                resp.success = True
                resp.message = "Everything went OK"
        except MoveItCommanderException as e:
            rospy.logwarn(e.args[0])
            resp.success = False
            resp.message = e.args[0]
        return resp

    def _arm_set_joint_positions_callback(self, set_joint_positions_req):
        """Request the Panda arm to go to a given joint angle.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionRequest`):
                The joint positions request message.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionResponse`: Response message
                containing (success bool, message).
        """
        rospy.logdebug("Setting arm joint position targets.")

        # Create moveit_commander command.
        resp = SetJointPositionsResponse()
        try:
            moveit_commander_arm_commands = self._create_positions_moveit_setpoints(
                set_joint_positions_req, control_group="arm"
            )["arm"]
        except InputMessageInvalidError as e:
            logwarn_msg = "Arm joint Positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.args[0]
            return resp

        # Set joint positions setpoint.
        arm_joint_states = self.move_group_arm.get_current_joint_values()
        rospy.logdebug(f"Current arm joint positions: {arm_joint_states}")
        self.joint_positions_target = moveit_commander_arm_commands
        try:
            rospy.logdebug(
                "Arm joint positions setpoint: %s" % moveit_commander_arm_commands
            )
            self.move_group_arm.set_joint_value_target(moveit_commander_arm_commands)
        except MoveItCommanderException as e:
            rospy.logwarn(
                "Setting arm joint position targets failed since there was an %s"
                % (lower_first_char(e.args[0]))
            )
            resp.success = False
            resp.message = "Failed to set arm setpoints."
            return resp

        # Execute setpoint.
        rospy.logdebug("Executing joint positions setpoint.")
        try:
            retval, err_msg = self._execute(control_group="arm")
            if not all(retval):
                resp.success = False
                resp.message = (
                    f"Arm joint position setpoint could not executed since {err_msg}."
                )
            else:
                resp.success = True
                resp.message = "Everything went OK"
        except MoveItCommanderException as e:
            rospy.logwarn(e.args[0])
            resp.success = False
            resp.message = e.args[0]
        return resp

    def _hand_set_joint_positions_callback(self, set_joint_positions_req):
        """Set panda hand to a given joint position or gripper width.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionRequest`):
                The joint positions request message.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionResponse`: Response message
                containing (success bool, message).
        """
        rospy.logdebug("Setting hand joint position targets.")

        # Create moveit_commander command.
        resp = SetJointPositionsResponse()
        try:
            hand_moveit_set_point = self._create_positions_moveit_setpoints(
                set_joint_positions_req, control_group="hand"
            )["hand"]
        except InputMessageInvalidError as e:
            logwarn_msg = "Hand joint Positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.args[0]
            return resp

        # Set joint positions setpoint.
        hand_joint_states = self.move_group_hand.get_current_joint_values()
        rospy.logdebug(f"Current hand joint positions: {hand_joint_states}")
        self.joint_positions_target = hand_moveit_set_point
        try:
            rospy.logdebug("Hand joint positions setpoint: %s" % hand_moveit_set_point)
            self.move_group_hand.set_joint_value_target(hand_moveit_set_point)
        except MoveItCommanderException as e:
            rospy.logwarn(
                "Setting hand joint position targets failed since there was an %s"
                % (lower_first_char(e.args[0]),)
            )
            resp.success = False
            resp.message = "Failed to set arm setpoints."
            return resp

        # Execute setpoints.
        rospy.logdebug("Executing joint positions setpoint.")
        try:
            retval, err_msg = self._execute(control_group="hand")
            if not all(retval):
                resp.success = False
                resp.message = (
                    "Hand joint position setpoint could not be executed since "
                    f"{err_msg}."
                )
            else:
                resp.success = True
                resp.message = "Everything went OK"
        except MoveItCommanderException as e:
            rospy.logwarn(e.args[0])
            resp.success = False
            resp.message = e.args[0]
        return resp

    def _arm_get_ee_pose_callback(self, _):
        """Request end effector pose.

        Args:
            get_ee_pose_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The current end effector pose.
        """
        rospy.logdebug("Retrieving ee pose.")
        ee_pose = self.move_group_arm.get_current_pose()
        resp = GetEePoseResponse()
        resp.pose = ee_pose.pose
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_get_ee_rpy_callback(self, _):
        """Request current end effector (EE) orientation.

        Args:
            get_ee_rpy_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`panda_gazebo.srv.GetEeResponse`: Response message containing
                containing the roll (x), yaw (z), pitch (y) euler angles.
        """
        rospy.logdebug("Retrieving ee orientation.")
        ee_rpy = self.move_group_arm.get_current_rpy()
        resp = GetEeRpyResponse()
        resp.r = ee_rpy[0]
        resp.y = ee_rpy[1]
        resp.p = ee_rpy[2]
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_get_ee_callback(self, _):
        """Request end effector (EE) name.

        Args:
            get_ee_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`panda_gazebo.srv.GetEeResponse`: Response message containing the name
                of the current EE.
        """
        rospy.logdebug("Retrieving ee name.")
        resp = GetEeResponse()
        resp.ee_name = self.move_group_arm.get_end_effector_link()
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_ee_callback(self, set_ee_req):
        """Request end effector (EE) change.

        Args:
            set_ee_req (:obj:`panda_gazebo.srv.SetEeRequest`):  Request message
                containing the name of the end effector you want to be set.

        Returns:
            :obj:`panda_gazebo.srv.SetEeResponse`: Response message containing (success
                bool, message).
        """
        rospy.logdebug(f"Setting ee to '{set_ee_req.ee_name}'.")
        resp = SetEeResponse()
        if self._link_exists(set_ee_req.ee_name):  # Check if valid.
            try:
                self.move_group_arm.set_end_effector_link(set_ee_req.ee_name)
            except MoveItCommanderException as e:
                rospy.logwarn("Ee could not be set.")
                resp.success = False
                resp.message = e.args[0]
            resp.success = True
            resp.message = "Everything went OK"
        else:
            rospy.logwarn(
                f"EE could not be as '{set_ee_req.ee_name}' is not a valid ee link."
            )
            resp.success = False
            resp.message = f"'{set_ee_req.ee_name}' is not a valid ee link."
        return resp

    def _get_random_joint_positions_callback(  # noqa: C901
        self, get_random_position_req
    ):
        """Returns valid joint position commands for the Panda arm and hand.

        Args:
            get_random_position_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`panda_gazebo.srv.GetRandomPositionsResponse`: Response message
                containing the random joints positions.

        .. important::
            Please be aware that when the ``min`` and ``max`` boundaries of a joint are
            set to be equal the joint is assumed to be unbounded.
        """
        resp = GetRandomJointPositionsResponse(
            success=True, message="Everything went OK"
        )
        max_attempts = (
            get_random_position_req.attempts
            if get_random_position_req.attempts != 0.0
            else MAX_RANDOM_SAMPLES
        )

        # Retrieve valid joint limits.
        joint_limits_values = get_random_position_req.joint_limits.values
        joint_limits_names = [
            joint_name.lower()
            for joint_name in get_random_position_req.joint_limits.names
        ]
        if joint_limits_names and joint_limits_values:
            joint_limits = self._get_valid_joint_limits(
                joint_limits_names, get_random_position_req.joint_limits.values
            )
        else:
            rospy.logwarn(
                "Joint limits ignored as either the joint names or the joint values "
                "were not specified."
            )
            joint_limits = {}

        # Retrieve random joint values.
        try:
            (
                random_arm_joint_values,
                random_hand_joint_values,
            ) = self._get_bounded_random_joint_values(joint_limits, max_attempts)
        except JointLimitsInvalidError as e:
            rospy.logwarn(e.log_message)
            resp.success = False
            resp.message = e.args[0]
            return resp

        # Return failure if no random joint positions could be retrieved.
        if random_arm_joint_values is None or (
            self._load_gripper and random_hand_joint_values is None
        ):
            joint_type = "arm"
            if self._load_gripper:
                if random_arm_joint_values is None and random_hand_joint_values is None:
                    joint_type = "hand and arm"
                elif random_hand_joint_values is None:
                    joint_type = "hand"
            limit_string = " with the set joint limits" if joint_limits else "."
            resp.success = False
            resp.message = (
                f"Random {joint_type} joint positions could not be retrieved"
                f"{limit_string}"
            )
            return resp

        # Fill and return successful response.
        joint_order = (
            [random_arm_joint_values, random_hand_joint_values]
            if self._arm_states_mask[0]
            else [random_hand_joint_values, random_arm_joint_values]
        )
        joint_order = [
            joint_values for joint_values in joint_order if joint_values is not None
        ]  # Filter out None values
        resp.joint_names = flatten_list(
            [list(joint_values.keys()) for joint_values in joint_order]
        )
        resp.joint_positions = flatten_list(
            [list(joint_values.values()) for joint_values in joint_order]
        )
        return resp

    def _get_random_ee_pose_callback(self, get_random_ee_pose_req):
        """Returns a valid random end effector (EE) pose for the Panda arm, along with
        the joint positions corresponding to this EE pose. This function ensures that
        the EE pose is within a specified bounding region, if one is provided.

        Args:
            get_random_ee_pose_req :obj:`std_srvs.srv.Empty`: Empty request.

        Returns:
            :obj:`panda_gazebo.srv.GetRandomEePoseResponse`: Response message containing
                the random ee pose and accompanying joint positions.

        .. important::
            Please be aware that when the ``min`` and ``max`` boundary of a EE
            coordinate are equal the dimension is assumed to be be unbounded.
        """
        resp = GetRandomEePoseResponse()
        max_attempts = (
            get_random_ee_pose_req.attempts
            if get_random_ee_pose_req.attempts != 0.0
            else MAX_RANDOM_SAMPLES
        )

        # Get a random ee pose.
        rospy.logdebug("Retrieving a valid random end effector pose.")
        try:
            random_ee_pose_unbounded = self.move_group_arm.get_random_pose()
        except MoveItCommanderException as e:
            rospy.logwarn("No random ee pose could be retrieved: %s" % e.args[0])
            resp.success = False
            resp.message = "Random ee pose could not be retrieved."
            return resp

        # Retrieve corresponding joint positions.
        retval, plan, _, _ = self.move_group_arm.plan(random_ee_pose_unbounded)
        if not retval:
            rospy.logwarn(
                "Corresponding joint configuration could not be retrieved for the"
                "random ee pose."
            )
            resp.success = False
            resp.message = (
                "Joint configuration could not be retrieved for random ee pose"
            )
            return resp

        # Return response if no bounding region is set.
        bounding_region = get_random_ee_pose_req.bounding_region
        if (
            sum(
                [
                    bounding_region.x_max - bounding_region.x_min,
                    bounding_region.y_max - bounding_region.y_min,
                    bounding_region.z_max - bounding_region.z_min,
                ]
            )
            == 0.0
        ):
            resp.success = True
            resp.message = "Everything went OK"
            resp.ee_pose = random_ee_pose_unbounded.pose
            resp.joint_names = plan.joint_trajectory.joint_names
            resp.joint_positions = list(plan.joint_trajectory.points[-1].positions)
            return resp
        x_bound_set = bounding_region.x_min != bounding_region.x_max
        y_bound_set = bounding_region.y_min != bounding_region.y_max
        z_bound_set = bounding_region.z_min != bounding_region.z_max

        # Return response if ee pose is within the bounding region.
        if (
            (
                not x_bound_set
                or bounding_region.x_min
                <= random_ee_pose_unbounded.pose.position.x
                <= bounding_region.x_max
            )
            and (
                not y_bound_set
                or bounding_region.y_min
                <= random_ee_pose_unbounded.pose.position.y
                <= bounding_region.y_max
            )
            and (
                not z_bound_set
                or bounding_region.z_min
                <= random_ee_pose_unbounded.pose.position.z
                <= bounding_region.z_max
            )
        ):
            resp.success = True
            resp.message = "Everything went OK"
            resp.ee_pose = random_ee_pose_unbounded.pose
            resp.joint_names = plan.joint_trajectory.joint_names
            resp.joint_positions = list(plan.joint_trajectory.points[-1].positions)
            return resp

        # Try to find a valid ee pose if it is not within the bounding region.
        for n_sample in range(max_attempts):
            sampled_ee_position = np.random.uniform(
                [
                    get_random_ee_pose_req.bounding_region.x_min,
                    get_random_ee_pose_req.bounding_region.y_min,
                    get_random_ee_pose_req.bounding_region.z_min,
                ],
                [
                    get_random_ee_pose_req.bounding_region.x_max,
                    get_random_ee_pose_req.bounding_region.y_max,
                    get_random_ee_pose_req.bounding_region.z_max,
                ],
                size=3,
            )

            # Check if ee pose is valid.
            # Use unbounded ee pose if the bounding region is unbounded (np.inf).
            random_ee_pose = Pose()
            random_ee_pose.position.x = (
                sampled_ee_position[0]
                if x_bound_set
                else random_ee_pose_unbounded.pose.position.x
            )
            random_ee_pose.position.y = (
                sampled_ee_position[1]
                if y_bound_set
                else random_ee_pose_unbounded.pose.position.y
            )
            random_ee_pose.position.z = (
                sampled_ee_position[2]
                if z_bound_set
                else random_ee_pose_unbounded.pose.position.z
            )
            random_ee_pose.orientation = random_ee_pose_unbounded.pose.orientation
            try:
                retval, plan, _, _ = self.move_group_arm.plan(random_ee_pose)
            except MoveItCommanderException:
                retval = False

            # Return response if valid ee pose was found.
            if retval:
                resp.success = True
                resp.message = "Everything went OK"
                resp.ee_pose = random_ee_pose
                resp.joint_names = plan.joint_trajectory.joint_names
                resp.joint_positions = list(plan.joint_trajectory.points[-1].positions)
                return resp

            rospy.logwarn(
                "Random end effector pose not within the set boundary region. "
                f"Trying again ({n_sample+1})."
            )

        # Return failure if no valid ee pose was found.
        rospy.logwarn(
            "No valid random end effector pose could be found within the set "
            "boundary region within the set number of attempts (%s). " % max_attempts
        )
        resp.success = False
        resp.message = "Valid random end effector pose could not be retrieved."
        return resp

    def _get_controlled_joints_cb(self, _):
        """Returns the joints that are currently being controlled by MoveIt.

        Args:
            get_controlled_joints_req (:obj:`panda_gazebo.srv.GetControlledJointsRequest`):
                The service request message specifying the control_type.

        Returns:
            :obj:`panda_gazebo.srv.GetControlledJointsResponse`: The response message
                that contains the ``controlled_joints`` list that specifies the joints
                that are currently controlled by MoveIt.
        """  # noqa: E501
        resp = GetMoveItControlledJointsResponse()
        resp.success = True
        resp.message = "Everything went OK"
        try:
            resp.controlled_joints = self._controlled_joints_dict["both"]
            resp.controlled_joints_arm = self._controlled_joints_dict["arm"]
            resp.controlled_joints_hand = self._controlled_joints_dict["hand"]
        except InputMessageInvalidError:
            resp.success = False
            resp.message = "Controlled joints could not be retrieved."
        return resp

    def _scene_add_box_callback(self, add_box_req):
        """Add box to planning scene

        Args:
            add_box_req (:obj:`panda_gazebo.srv.AddBoxRequest`): The add box request.

        Returns:
            bool: Whether the box was successfully added.
        """
        box_name = add_box_req.name or "box"
        pose_header = Header(frame_id=add_box_req.frame_id or "world")
        box_pose = Pose(
            position=add_box_req.pose.position,
            orientation=normalize_quaternion(add_box_req.pose.orientation),
        )
        pose_stamped = PoseStamped(header=pose_header, pose=box_pose)

        # Warn users if box is not visible in RViz.
        if add_box_req.size == (0.0, 0.0, 0.0):
            rospy.logwarn(
                f"The size of box '{box_name}' was set to (0, 0, 0). The box will not "
                "be visible in RViz."
            )

        # Send request.
        resp = AddBoxResponse()
        try:
            self.scene.add_box(
                box_name,
                pose_stamped,
                size=add_box_req.size,
            )
        except Exception:
            resp.success = False
            resp.message = "Box could not be added"
            return resp

        # Return response.
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _scene_add_plane_callback(self, add_plane_req):
        """Add plane to planning scene

        Args:
            add_plane_req (:obj:`panda_gazebo.srv.AddBoxRequest`): The add plane
                request.

        Returns:
            bool: Whether the plane was successfully added.
        """
        plane_name = add_plane_req.name or "plane"
        pose_header = Header(frame_id=add_plane_req.frame_id or "world")
        plane_pose = Pose(
            position=add_plane_req.pose.position,
            orientation=normalize_quaternion(add_plane_req.pose.orientation),
        )
        pose_stamped = PoseStamped(header=pose_header, pose=plane_pose)

        # Send request.
        resp = AddPlaneResponse()
        try:
            self.scene.add_plane(
                plane_name,
                pose_stamped,
                normal=normalize_vector(add_plane_req.normal, force=True),
                offset=add_plane_req.offset,
            )
        except Exception as e:
            resp.success = False
            resp.message = "Plane could not be added because: %s" % e
            return resp

        # Return response.
        resp.success = True
        resp.message = "Everything went OK"
        return resp
