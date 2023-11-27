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
import copy
import re
import sys
from itertools import compress

import moveit_commander
import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
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
    lower_first_char,
    normalize_quaternion,
    quaternion_norm,
    ros_exit_gracefully,
    translate_moveit_error_code,
    normalize_vector,
)
from panda_gazebo.exceptions import InputMessageInvalidError
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
            arm_ee_link (str, optional): The end effector you want moveit to use when
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
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
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
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
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
        self._controlled_joints_dict = {
            "arm": flatten_list(self.move_group_arm.get_active_joints()),
            "hand": flatten_list(self.move_group_hand.get_active_joints())
            if self._load_gripper
            else [],
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
        try:
            self._max_velocity_scaling = rospy.get_param("~max_velocity_scaling_factor")
        except KeyError:
            self._max_velocity_scaling = None
        try:
            self._max_acceleration_scaling = rospy.get_param(
                "~max_acceleration_scaling_factor"
            )
        except KeyError:
            self._max_acceleration_scaling = None

    def _link_exists(self, link_name):
        """Function checks whether a given link exists in the robot_description.

        Args:
            link_name (str): Name of link you want to check.

        Returns:
            bool: Boolean specifying whether the link exists.
        """
        return link_name in self.robot.get_link_names()

    def _execute(self, control_group="both", wait=True):  # noqa: C901
        """Plan and execute a trajectory/pose or orientation setpoints

        Args:
            control_group (str, optional): The robot control group for which you want
                to execute the control. Options are ``arm`` or ``hand`` or ``both``.
                Defaults to ``both``.
            wait (boolean, optional): Whether to wait on the control to be executed.

        Returns:
            list: List specifying whether the arm and/or hand execution was successful.
                If ``control_group == "both"`` then ``["arm_success", "hand_success"]``.
        """
        control_group = control_group.lower()

        # Execute.
        retval = []
        if control_group == "arm":
            (arm_plan_retval, _, _, error_code) = self.move_group_arm.plan()
            if arm_plan_retval:
                arm_retval = self.move_group_arm.go(wait=wait)
                if wait:
                    self.move_group_arm.stop()
            else:
                rospy.logwarn(
                    "No plan found for the current arm setpoints since '%s'"
                    % translate_moveit_error_code(error_code)
                )
                arm_retval = False
            retval.append(arm_retval)
        elif control_group == "hand":
            if self._load_gripper:
                (hand_plan_retval, _, _, error_code) = self.move_group_hand.plan()
                if hand_plan_retval:
                    hand_retval = self.move_group_hand.go(wait=wait)
                    if wait:
                        self.move_group_hand.stop()
                else:
                    rospy.logwarn(
                        "No plan found for the current hand setpoints since '%s'"
                        % translate_moveit_error_code(error_code)
                    )
                    hand_retval = False
                retval.append(hand_retval)
            else:
                rospy.logwarn("Hand commands not executed since gripper is not loaded.")
                retval.append(False)
        elif control_group == "both":
            # Get hand plan.
            if not self._load_gripper:
                rospy.logwarn_once(
                    "Hand commands not executed since gripper is not loaded."
                )
            else:
                (hand_plan_retval, _, _, error_code) = self.move_group_hand.plan()

            # Get arm plan.
            (arm_plan_retval, _, _, error_code) = self.move_group_arm.plan()

            # Execute arm/hand plans
            if self._load_gripper:
                if hand_plan_retval:
                    hand_retval = self.move_group_hand.go(wait=False)
                else:
                    rospy.logwarn(
                        "No plan found for the current hand setpoints since '%s'"
                        % translate_moveit_error_code(error_code)
                    )
                    hand_retval = False
                retval.append(hand_retval)
            if arm_plan_retval:
                arm_retval = self.move_group_arm.go(wait=wait)
                if wait:
                    if self._load_gripper:
                        self.move_group_hand.stop()
                    self.move_group_arm.stop()
            else:
                rospy.logwarn(
                    "No plan found for the current hand setpoints since '%s'"
                    % translate_moveit_error_code(error_code)
                )
                arm_retval = False
            retval.append(arm_retval)
        else:
            logwarn_msg = (
                f"Control group '{control_group}' does not exist. Please specify a "
                "valid control group. Valid values are 'arm', 'hand' or 'both'."
            )
            rospy.logwarn(logwarn_msg)
            retval = [False]
        return retval

    def _create_joint_positions_commands(  # noqa: C901
        self, input_msg, control_group="both"
    ):
        """Converts the service input message in :mod:`moveit_commander` compatible joint
        position setpoint commands. While doing this it also verifies whether the given
        input message is valid.

        Args:
            input_msg (str): The service input message we want to validate.
            control_group (str, optional): The robot control group for which you want to
                execute the control. Options are ``arm`` or ``hand`` or ``both``.
                Defaults to ``both``.

        Returns:
            dict: Dictionary that contains the 'moveit_commander' arm and hand joint
                position commands. Grouped by control group.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                input_msg could not be converted into 'moveit_commander' arm hand joint
                position commands.
        """
        joint_names = input_msg.joint_names
        joint_positions = list(input_msg.joint_positions)
        control_group = control_group.lower()

        # Validate control_group.
        if control_group not in ["arm", "hand", "both"]:
            logwarn_msg = (
                f"The '{control_group}' control group does not exist. Please specify "
                "a valid control group (options: 'arm', 'hand', 'both')."
            )
            rospy.logerr(logwarn_msg)
            raise InputMessageInvalidError(
                message=f"Control group '{control_group}' does not exist.",
                log_message=logwarn_msg,
            )
        elif control_group == "hand" and not self._load_gripper:
            logwarn_msg = (
                "The 'hand' control group can not be used when the 'load_gripper' "
                "variable is set to False. Please change 'load_gripper' to True or "
                "specify a valid control group (options: ['arm'])."
            )
            rospy.logerr(logwarn_msg)
            raise InputMessageInvalidError(
                message=f"Control group '{control_group}' does not exist.",
                log_message=logwarn_msg,
            )
        elif control_group == "both" and not self._load_gripper:
            # Change to arm if the hand is not loaded.
            control_group = "arm"

        # Get controlled joints, throw warning if control group is incorrect.
        if control_group == "arm":
            controlled_joints = self._controlled_joints_dict["arm"]
        elif control_group == "hand":
            controlled_joints = self._controlled_joints_dict["hand"]
        else:
            controlled_joints = flatten_list(
                [
                    self._controlled_joints_dict["arm"],
                    self._controlled_joints_dict["hand"],
                ]
            )

        # Get the current state of the arm and hand.
        arm_state_dict = dict(
            zip(
                self.move_group_arm.get_active_joints(),
                self.move_group_arm.get_current_joint_values(),
            )
        )
        if control_group in ["hand", "both"]:
            hand_state_dict = dict(
                zip(
                    self.move_group_hand.get_active_joints(),
                    self.move_group_hand.get_current_joint_values(),
                )
            )

        # Generate 'moveit_commander' control command
        controlled_joints_size = len(controlled_joints)
        if len(joint_names) == 0:
            # Check if enough joint position commands were given.
            if len(joint_positions) > controlled_joints_size:
                logwarn_msg = (
                    "You specified %s while the Panda Robot %s only %s %s. Only the "
                    "first %s are used in the control."
                    % (
                        "%s %s"
                        % (
                            len(joint_positions),
                            "joint position"
                            if len(joint_positions) == 1
                            else "joint positions",
                        ),
                        "arm and hand" if control_group == "both" else control_group,
                        "contain" if control_group == "both" else "contains",
                        "%s %s"
                        % (
                            controlled_joints_size,
                            "active joint"
                            if controlled_joints_size == 1
                            else "active joints",
                        ),
                        "%s %s"
                        % (
                            controlled_joints_size,
                            "joint position"
                            if controlled_joints_size == 1
                            else "joint positions",
                        ),
                    )
                )
                rospy.logwarn(logwarn_msg)

                # Remove the extra joint commands.
                joint_positions = joint_positions[:controlled_joints_size]
            elif len(joint_positions) < controlled_joints_size:
                logwarn_msg = (
                    "You specified %s while the Panda Robot %s %s %s. As a result "
                    "only the first %s joints are controlled while for the other "
                    "joints the current robot state is used."
                    % (
                        "%s %s"
                        % (
                            len(joint_positions),
                            "joint position"
                            if len(joint_positions) == 1
                            else "joint positions",
                        ),
                        "arm and hand" if control_group == "both" else control_group,
                        "contain" if control_group == "both" else "contains",
                        "%s %s"
                        % (
                            controlled_joints_size,
                            "active joint"
                            if controlled_joints_size == 1
                            else "active joints",
                        ),
                        len(joint_positions),
                    )
                )
                rospy.logwarn(logwarn_msg)

                # Fill the missing joint commands.
                if control_group == "both":
                    joint_states = (
                        list(arm_state_dict.values()) + list(hand_state_dict.values())
                        if self._arm_states_mask[0]
                        else list(hand_state_dict.values())
                        + list(arm_state_dict.values())
                    )
                else:
                    joint_states = (
                        list(arm_state_dict.values())
                        if control_group == "arm"
                        else list(hand_state_dict.values())
                    )
                joint_positions = (
                    joint_positions
                    + joint_states[len(joint_positions) :]  # noqa: E203, E501
                )

            # Return moveit_commander_control command dictionary.
            if control_group == "arm":
                control_commands = {"arm": joint_positions}
            elif control_group == "hand":
                control_commands = {"hand": joint_positions}
            else:
                control_commands = {
                    "arm": list(compress(joint_positions, self._arm_states_mask)),
                    "hand": list(compress(joint_positions, self._hand_states_mask)),
                }
            return control_commands
        else:
            # Check if enough control values were given.
            if len(joint_names) != len(joint_positions):
                logwarn_msg = (
                    "You specified %s while the 'joint_names' field of the "
                    "'%s' message contains %s. Please make sure you supply "
                    "a joint position for each joint contained in the 'joint_names' "
                    "field."
                    % (
                        "%s %s"
                        % (
                            len(joint_positions),
                            "joint position"
                            if len(joint_positions) == 1
                            else "joint positions",
                        ),
                        "panda_gazebo/SetJointPositions",
                        "%s %s"
                        % (
                            len(joint_names),
                            "joint" if len(joint_names) == 1 else "joints",
                        ),
                    )
                )
                rospy.logerr(logwarn_msg)
                raise InputMessageInvalidError(
                    message=(
                        "Joint_names and joint_positions fields of the input "
                        "message are of different lengths."
                    ),
                    log_message=logwarn_msg,
                    details={
                        "joint_positions_command_length": len(joint_positions),
                        "joint_names_length": len(joint_names),
                    },
                )

            # Validate joint_names.
            invalid_joint_names = [
                joint_name
                for joint_name in joint_names
                if joint_name not in controlled_joints
            ]
            input_command_dict = dict(zip(joint_names, joint_positions))
            if len(invalid_joint_names) != 0:
                # Remove invalid keys and throw warning.
                [input_command_dict.pop(joint, None) for joint in invalid_joint_names]
                logwarn_msg = (
                    "Ignoring %s %s since %s invalid. Valid joint names for "
                    "controlling the panda %s are %s."
                    % (
                        invalid_joint_names,
                        "joint" if len(invalid_joint_names) == 1 else "joints",
                        "it is" if len(invalid_joint_names) == 1 else "they are",
                        "arm and hand" if control_group == "both" else control_group,
                        controlled_joints,
                    )
                )
                rospy.logwarn(logwarn_msg)

            # Update current state dictionary with given joint_position setpoints.
            arm_output_command_dict = copy.deepcopy(arm_state_dict)
            for joint, position in input_command_dict.items():  # Update arm.
                if joint in arm_state_dict:
                    arm_output_command_dict[joint] = position
            if control_group in ["hand", "both"]:
                hand_output_command_dict = copy.deepcopy(hand_state_dict)
                for joint, position in input_command_dict.items():  # Update hand.
                    if joint in hand_state_dict:
                        hand_output_command_dict[joint] = position

            # Return moveit_commander_control command dictionary.
            if control_group == "arm":
                control_commands = {"arm": list(arm_output_command_dict.values())}
            elif control_group == "hand":
                control_commands = {"hand": list(hand_output_command_dict.values())}
            else:
                control_commands = {
                    "arm": list(arm_output_command_dict.values()),
                    "hand": list(hand_output_command_dict.values()),
                }
            return control_commands

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
                "The quaternion in the set ee pose was normalized since moveit expects "
                "normalized quaternions."
            )
            get_ee_pose_joint_configuration.pose.orientation = normalize_quaternion(
                get_ee_pose_joint_configuration.pose.orientation
            )

        pose_target = Pose()
        pose_target.orientation.x = get_ee_pose_joint_configuration.pose.orientation.x
        pose_target.orientation.y = get_ee_pose_joint_configuration.pose.orientation.y
        pose_target.orientation.z = get_ee_pose_joint_configuration.pose.orientation.z
        pose_target.orientation.w = get_ee_pose_joint_configuration.pose.orientation.w
        pose_target.position.x = get_ee_pose_joint_configuration.pose.position.x
        pose_target.position.y = get_ee_pose_joint_configuration.pose.position.y
        pose_target.position.z = get_ee_pose_joint_configuration.pose.position.z

        # Retrieve joint configurations.
        resp = GetEePoseJointConfigResponse()
        n_sample = 0
        while True:  # Continue till joint positions are valid or max samples size.
            rospy.logdebug("Retrieving joint configuration for given EE pose.")
            try:
                retval, plan, _, _ = self.move_group_arm.plan(pose_target)
            except MoveItCommanderException:
                retval = False

            # Check if joint config was retrieved.
            if retval:
                resp.joint_names = list(plan.joint_trajectory.joint_names)
                resp.joint_positions = list(plan.joint_trajectory.points[-1].positions)
                resp.success = True
                resp.message = "Everything went OK"
                break
            elif n_sample >= max_attempts:
                resp.success = False
                resp.message = "Joint configuration could not be retrieved for Ee pose"
                break
            else:
                rospy.logwarn(
                    "Failed to retrieve joint configuration for given EE pose Trying "
                    f"again ({n_sample})."
                )
                n_sample += 1
        return resp

    def _arm_set_ee_pose_callback(self, set_ee_pose_req):
        """Request the Panda arm to control to a given end effector
        (EE) pose.

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
                "The quaternion in the set ee pose was normalized since moveit expects "
                "normalized quaternions."
            )
            set_ee_pose_req.pose.orientation = normalize_quaternion(
                set_ee_pose_req.pose.orientation
            )

        # Fill trajectory message.
        rospy.logdebug("Setting ee pose.")
        resp = SetEePoseResponse()
        self.ee_pose_target.orientation.x = set_ee_pose_req.pose.orientation.x
        self.ee_pose_target.orientation.y = set_ee_pose_req.pose.orientation.y
        self.ee_pose_target.orientation.z = set_ee_pose_req.pose.orientation.z
        self.ee_pose_target.orientation.w = set_ee_pose_req.pose.orientation.w
        self.ee_pose_target.position.x = set_ee_pose_req.pose.position.x
        self.ee_pose_target.position.y = set_ee_pose_req.pose.position.y
        self.ee_pose_target.position.z = set_ee_pose_req.pose.position.z

        # Send trajectory message and return response.
        try:
            self.move_group_arm.set_pose_target(self.ee_pose_target)
            retval = self._execute(control_group="arm")
            self.move_group_arm.clear_pose_targets()

            # Check if setpoint execution was successful.
            if not all(retval):
                resp.success = False
                resp.message = "Ee pose could not be set"
            else:
                resp.success = True
                resp.message = "Everything went OK"
        except MoveItCommanderException as e:
            rospy.logwarn(e.args[0])
            resp.success = False
            resp.message = e.args[0]
        return resp

    def _set_joint_positions_callback(self, set_joint_positions_req):  # noqa: C901
        """Request the Panda arm and hand to go to a given joint angle.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionRequest`):
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionResponse`: Response message
                 containing (success bool, message).
        """
        rospy.logdebug("Setting joint position targets.")

        # Check if set_joint_positions_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "positions."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/set_joint_positions" % rospy.get_name(),
                )
            )

        # Create moveit_commander command.
        resp = SetJointPositionsResponse()
        try:
            moveit_commander_commands = self._create_joint_positions_commands(
                set_joint_positions_req
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
        arm_joint_states = self.move_group_arm.get_current_joint_values()
        rospy.logdebug(f"Current arm joint positions: {arm_joint_states}")
        if self._load_gripper:
            hand_joint_states = self.move_group_hand.get_current_joint_values()
            rospy.logdebug(f"Current hand joint positions: {hand_joint_states}")
        self.joint_positions_target = moveit_commander_commands
        set_joint_value_target_success_bool = []
        set_joint_value_target_error_msg = []
        try:
            rospy.logdebug(
                "Arm joint positions setpoint: %s" % moveit_commander_commands["arm"]
            )
            self.move_group_arm.set_joint_value_target(moveit_commander_commands["arm"])
            set_joint_value_target_success_bool.append(True)
        except MoveItCommanderException as e:
            set_joint_value_target_success_bool.append(False)
            set_joint_value_target_error_msg.append(e.args[0])
        if self._load_gripper:
            try:
                rospy.logdebug(
                    "Hand joint positions setpoint: %s"
                    % moveit_commander_commands["hand"]
                )
                self.move_group_hand.set_joint_value_target(
                    moveit_commander_commands["hand"]
                )
                set_joint_value_target_success_bool.append(True)
            except MoveItCommanderException as e:
                set_joint_value_target_success_bool.append(False)
                set_joint_value_target_error_msg.append(e.args[0])

        # Print error message if an error occurred and return.
        if set_joint_value_target_error_msg:
            if len(set_joint_value_target_error_msg) > 1:
                log_warn_string = "arm and hand"
                rospy.logwarn(
                    "Setting arm joint position targets failed since there was an %s"
                    % (lower_first_char(set_joint_value_target_error_msg[0]))
                )
                rospy.logwarn(
                    "Setting hand joint position targets failed since there was an %s"
                    % (lower_first_char(set_joint_value_target_error_msg[1]))
                )
            else:
                log_warn_string = (
                    "arm"
                    if not self._load_gripper
                    else ("arm" if set_joint_value_target_success_bool[0] else "hand")
                )
                rospy.logwarn(
                    "Setting %s joint position targets failed since there was an %s"
                    % (
                        log_warn_string,
                        lower_first_char(set_joint_value_target_error_msg[0]),
                    )
                )
            resp.success = False
            resp.message = f"Failed to set {log_warn_string} setpoints."
            return resp

        # Execute setpoints.
        rospy.logdebug("Executing joint positions setpoint.")
        try:
            retval = self._execute()
            if not all(retval):
                resp.success = False
                resp.message = "Joint position setpoint could not be set"
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
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionResponse`: Response message
                containing (success bool, message).
        """
        rospy.logdebug("Setting arm joint position targets.")

        # Check if set_joint_positions_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "positions."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_arm/set_joint_positions" % rospy.get_name(),
                )
            )

        # Create moveit_commander command.
        resp = SetJointPositionsResponse()
        try:
            moveit_commander_commands = self._create_joint_positions_commands(
                set_joint_positions_req, control_group="arm"
            )
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
        self.joint_positions_target = moveit_commander_commands
        try:
            rospy.logdebug(
                "Arm joint positions setpoint: %s" % moveit_commander_commands["arm"]
            )
            self.move_group_arm.set_joint_value_target(moveit_commander_commands["arm"])
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
            retval = self._execute(control_group="arm")
            if not all(retval):
                resp.success = False
                resp.message = "Arm joint position setpoint could not be set"
            else:
                resp.success = True
                resp.message = "Everything went OK"
        except MoveItCommanderException as e:
            rospy.logwarn(e.args[0])
            resp.success = False
            resp.message = e.args[0]
        return resp

    def _hand_set_joint_positions_callback(self, set_joint_positions_req):
        """Request the Panda arm to go to a given joint angle.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionRequest`):
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionResponse`: Response message
                containing (success bool, message).
        """
        rospy.logdebug("Setting hand joint position targets.")

        # Check if set_joint_positions_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "positions."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_hand/set_joint_positions" % rospy.get_name(),
                )
            )

        # Create moveit_commander command.
        resp = SetJointPositionsResponse()
        try:
            moveit_commander_commands = self._create_joint_positions_commands(
                set_joint_positions_req, control_group="hand"
            )
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
        self.joint_positions_target = moveit_commander_commands
        try:
            rospy.logdebug(
                "Hand joint positions setpoint: %s" % moveit_commander_commands["hand"]
            )
            self.move_group_hand.set_joint_value_target(
                moveit_commander_commands["hand"]
            )
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
            retval = self._execute(control_group="hand")
            if not all(retval):
                resp.success = False
                resp.message = "Hand joint position setpoint could not be set"
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
        max_attempts = (
            get_random_position_req.attempts
            if get_random_position_req.attempts != 0.0
            else MAX_RANDOM_SAMPLES
        )
        joint_limits = {}
        limited_joints = []
        arm_joints = self.move_group_arm.get_active_joints()
        hand_joints = (
            self.move_group_hand.get_active_joints() if self._load_gripper else []
        )

        # Validate joint limits if supplied (remove them if invalid)
        if (
            get_random_position_req.joint_limits.names
            and get_random_position_req.joint_limits.values
        ):
            get_random_position_req.joint_limits.names = [
                joint_limit_name.lower()
                for joint_limit_name in get_random_position_req.joint_limits.names
            ]

            # Check if limit names and limit values are of equal length.
            if len(get_random_position_req.joint_limits.names) != len(
                get_random_position_req.joint_limits.values
            ):
                rospy.logwarn(
                    "Joint limits ignored as the number of joints ({}) is "
                    "unequal to the number of limit values ({}).".format(
                        len(get_random_position_req.joint_limits.names),
                        len(get_random_position_req.joint_limits.values),
                    )
                )
                get_random_position_req.joint_limits.names = []
                get_random_position_req.joint_limits.values = []
            else:  # Check if the names in the joint_limits message are valid.
                valid_joint_names = flatten_list(
                    [
                        [joint + "_min", joint + "_max"]
                        for joint in flatten_list([arm_joints, hand_joints])
                    ]
                )
                invalid_joint_names = []
                for name in get_random_position_req.joint_limits.names:
                    if name not in valid_joint_names:
                        invalid_joint_names.append(name)

                # Throw warning and remove invalid joint limits.
                if len(invalid_joint_names) != 0:
                    warn_strings = (
                        [
                            "limit",
                            invalid_joint_names[0],
                            "was",
                            "it is not a valid joint limit",
                        ]
                        if len(invalid_joint_names) == 1
                        else [
                            "limits",
                            invalid_joint_names,
                            "were",
                            "they are not valid joint limits",
                        ]
                    )
                    rospy.logwarn(
                        "Joint {} '{}' {} ignored since {}. Valid values are "
                        "'{}'.".format(
                            warn_strings[0],
                            warn_strings[1],
                            warn_strings[2],
                            warn_strings[3],
                            valid_joint_names,
                        )
                    )
                    get_random_position_req.joint_limits.names = [
                        joint_name
                        for joint_name in get_random_position_req.joint_limits.names
                        if joint_name not in invalid_joint_names
                    ]
                    get_random_position_req.joint_limits.values = [
                        joint_limit
                        for joint_limit_name, joint_limit in zip(
                            get_random_position_req.joint_limits.names,
                            get_random_position_req.joint_limits.values,
                        )
                        if joint_limit_name not in invalid_joint_names
                    ]

                # Check if a joint limit both has a min and max specified.
                limited_joints = get_unique_list(
                    [
                        name.replace("_min", "").replace("_max", "")
                        for name in get_random_position_req.joint_limits.names
                    ]
                )
                required_joint_limits = flatten_list(
                    [[joint + "_min", joint + "_max"] for joint in limited_joints]
                )
                missing_joint_limits = [
                    required_joint_limit
                    for required_joint_limit in required_joint_limits
                    if required_joint_limit
                    not in get_random_position_req.joint_limits.names
                ]
                if missing_joint_limits:
                    ignored_joint_limit_joint = get_unique_list(
                        [
                            missing_joint_limit.replace("_min", "").replace("_max", "")
                            for missing_joint_limit in missing_joint_limits
                        ]
                    )
                    warn_strings = (
                        ["joint", f"'{ignored_joint_limit_joint}'"]
                        if len(ignored_joint_limit_joint) == 1
                        else ["joints", ignored_joint_limit_joint]
                    )
                    rospy.logwarn(
                        "Joint limits specified on {} {} were ignored as both a min "
                        "and max limit need to be specified.".format(*warn_strings)
                    )
                    get_random_position_req.joint_limits.names = [
                        name
                        for name in get_random_position_req.joint_limits.names
                        if name.replace("_min", "").replace("_max", "")
                        not in ignored_joint_limit_joint
                    ]
                    get_random_position_req.joint_limits.values = [
                        joint_limit
                        for joint_limit_name, joint_limit in zip(
                            get_random_position_req.joint_limits.names,
                            get_random_position_req.joint_limits.values,
                        )
                        if joint_limit_name.replace("_min", "").replace("_max", "")
                        not in ignored_joint_limit_joint
                    ]
            joint_limits = dict(
                zip(
                    get_random_position_req.joint_limits.names,
                    get_random_position_req.joint_limits.values,
                )
            )

        # Retrieve random joint position values.
        get_random_arm_joint_positions_srvs_exception = False
        get_random_hand_joint_positions_srvs_exception = False
        try:
            random_arm_joint_values_unbounded = dict(
                zip(arm_joints, self.move_group_arm.get_random_joint_values())
            )
        except MoveItCommanderException:
            get_random_arm_joint_positions_srvs_exception = True
        if self._load_gripper:
            try:
                random_hand_joint_values_unbounded = dict(
                    zip(hand_joints, self.move_group_hand.get_random_joint_values())
                )
            except MoveItCommanderException:
                get_random_hand_joint_positions_srvs_exception = True

        # Get random joint positions (while taking into possible joint limits)
        resp = GetRandomJointPositionsResponse()
        random_arm_joint_values = random_arm_joint_values_unbounded.copy()
        random_hand_joint_values = (
            random_hand_joint_values_unbounded.copy() if self._load_gripper else {}
        )
        if not joint_limits:  # If no joint limits were given.
            if (
                not get_random_arm_joint_positions_srvs_exception
                and not get_random_hand_joint_positions_srvs_exception
            ):
                resp.success = True
                resp.message = "Everything went OK"
            else:
                resp.success = False
                resp.message = "Random joint position could not be retrieved."
        else:  # Joint limits were set.
            # Try to find random joint values within the joint limits.
            n_sample = 0
            arm_joint_commands_valid = False
            hand_joint_commands_valid = False if self._load_gripper else True
            while True:  # Continue till joint positions are valid or max samples size.
                rospy.logdebug("Retrieving valid random joint positions.")
                for joint in get_unique_list(
                    [
                        joint_limit_names.replace("_min", "").replace("_max", "")
                        for joint_limit_names in joint_limits.keys()
                    ]
                ):
                    # Sample random value for the given joint within the joint limits.
                    if (
                        joint in random_arm_joint_values.keys()
                        and not arm_joint_commands_valid
                    ):
                        if (
                            joint_limits[joint + "_min"] != joint_limits[joint + "_max"]
                        ):  # Only sample if min and max are unequal.
                            random_arm_joint_values[joint] = np.random.uniform(
                                joint_limits[joint + "_min"],
                                joint_limits[joint + "_max"],
                            )
                        else:
                            arm_joint_commands_valid = True
                    if (
                        joint in random_hand_joint_values.keys()
                        and not hand_joint_commands_valid
                    ):
                        if joint_limits[joint + "_min"] != joint_limits[joint + "_max"]:
                            random_hand_joint_values[joint] = np.random.uniform(
                                joint_limits[joint + "_min"],
                                joint_limits[joint + "_max"],
                            )
                        else:
                            hand_joint_commands_valid = True

                # Check if joint positions are valid (Plan is not empty)
                if not arm_joint_commands_valid:
                    try:
                        arm_plan = self.move_group_arm.plan(
                            joint_state_dict_2_joint_state_msg(random_arm_joint_values)
                        )
                        if len(arm_plan[1].joint_trajectory.points) != 0:
                            arm_joint_commands_valid = True
                        else:
                            arm_joint_commands_valid = False
                    except MoveItCommanderException:
                        arm_joint_commands_valid = False
                if not hand_joint_commands_valid:
                    try:
                        hand_plan = self.move_group_hand.plan(
                            joint_state_dict_2_joint_state_msg(random_hand_joint_values)
                        )
                        if len(hand_plan[1].joint_trajectory.points) != 0:
                            hand_joint_commands_valid = True
                        else:
                            hand_joint_commands_valid = False
                    except MoveItCommanderException:
                        hand_joint_commands_valid = False

                # Set success boolean and break out of loop if joint positions are valid
                # otherwise keep sampling till the max sample limit has been reached.
                if arm_joint_commands_valid and hand_joint_commands_valid:  # If valid.
                    resp.success = True
                    resp.message = "Everything went OK"
                    break
                elif n_sample >= max_attempts:
                    if (
                        not get_random_arm_joint_positions_srvs_exception
                        and not get_random_hand_joint_positions_srvs_exception
                    ):
                        warn_string = (
                            "hand and arm"
                            if not (
                                hand_joint_commands_valid and arm_joint_commands_valid
                            )
                            else ("hand" if hand_joint_commands_valid else "arm")
                        )
                        rospy.logwarn(
                            (
                                "No valid random {} positions could be found within "
                                "the set boundary region within the set number of "
                                "attempts ({}). Please make sure that the robot joints "
                                "can reach the set joint_limits. Unbounded random "
                                "{} joint positions returned instead."
                            ).format(warn_string, max_attempts, warn_string)
                        )
                        resp.success = False
                        resp.message = (
                            "Random {} joint positions could not be retrieved in the "
                            "boundary region within the set attempts. Unbounded {} "
                            "random joint positions returned instead."
                        ).format(warn_string, warn_string)
                        random_arm_joint_values = random_arm_joint_values_unbounded
                        if self._load_gripper:
                            random_hand_joint_values = (
                                random_hand_joint_values_unbounded
                            )
                    else:
                        resp.success = False
                        resp.message = "Random joint position could not be retrieved."
                    break
                else:
                    rospy.logwarn(
                        "Failed to sample valid random joint positions from the "
                        "bounding region. Trying again."
                    )
                    n_sample += 1  # Increase sampling counter.

        # Fill and resturn response message.
        resp.joint_names = (
            flatten_list(
                [
                    list(random_arm_joint_values.keys()),
                    list(random_hand_joint_values.keys()),
                ]
            )
            if self._arm_states_mask[0]
            else flatten_list(
                [
                    list(random_hand_joint_values.keys()),
                    list(random_arm_joint_values.keys()),
                ]
            )
        )
        resp.joint_positions = (
            flatten_list(
                [
                    list(random_arm_joint_values.values()),
                    list(random_hand_joint_values.values()),
                ]
            )
            if self._arm_states_mask[0]
            else flatten_list(
                [
                    list(random_hand_joint_values.values()),
                    list(random_arm_joint_values.values()),
                ]
            )
        )
        return resp

    def _get_random_ee_pose_callback(self, get_random_ee_pose_req):  # noqa: C901
        """Returns valid ee pose for the Panda arm. This function also makes sure that
        the ee pose is within a bounding region, if one is supplied.

        Args:
            get_random_ee_pose_req :obj:`std_srvs.srv.Empty`: Empty request.

        Returns:
            :obj:`panda_gazebo.srv.GetRandomEePoseResponse`: Response message containing
                the random joints positions.

        .. note::
            Additionally also returns the joint_positions that relate to the random EE
            pose.

        .. important::
            Please be aware that when the ``min`` and ``max`` boundary of a EE coordinate
            are equal the dimension is assumed to be be unbounded.
        """
        max_attempts = (
            get_random_ee_pose_req.attempts
            if get_random_ee_pose_req.attempts != 0.0
            else MAX_RANDOM_SAMPLES
        )

        # Get a random ee pose.
        try:
            random_ee_pose_unbounded = self.move_group_arm.get_random_pose()
            get_random_pose_srvs_exception = False
        except MoveItCommanderException:
            get_random_pose_srvs_exception = True

        # Return random ee pose (while taking into account a possible bounding region)
        resp = GetRandomEePoseResponse()
        if (
            sum(
                [
                    get_random_ee_pose_req.bounding_region.x_max
                    - get_random_ee_pose_req.bounding_region.x_min,
                    get_random_ee_pose_req.bounding_region.y_max
                    - get_random_ee_pose_req.bounding_region.y_min,
                    get_random_ee_pose_req.bounding_region.z_max
                    - get_random_ee_pose_req.bounding_region.z_min,
                ]
            )
            == 0.0
        ):  # No bounding region was set.
            if not get_random_pose_srvs_exception:
                resp.success = True
                resp.message = "Everything went OK"
                resp.ee_pose = random_ee_pose_unbounded.pose
            else:
                resp.success = False
                resp.message = "Random ee pose could not be retrieved."
        else:  # A bounding region was set.
            # Try to find a valid ee_pose within the bounding region.
            n_sample = 0
            ee_pose_valid = False
            while True:  # Continue till ee pose is valid or max samples size.
                rospy.logdebug("Retrieving a valid random end effector pose.")

                # Sample ee position from bounding region.
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

                # Create ee_pose msg.
                # NOTE: Only use boundary when {x,y,z}_min and {x,y,z}_max are different
                random_ee_pose = Pose()
                random_ee_pose.position.x = (
                    sampled_ee_position[0]
                    if (
                        get_random_ee_pose_req.bounding_region.x_min
                        != get_random_ee_pose_req.bounding_region.x_max
                    )
                    else random_ee_pose_unbounded.pose.position.x
                )
                random_ee_pose.position.y = (
                    sampled_ee_position[1]
                    if (
                        get_random_ee_pose_req.bounding_region.y_min
                        != get_random_ee_pose_req.bounding_region.y_max
                    )
                    else random_ee_pose_unbounded.pose.position.y
                )
                random_ee_pose.position.z = (
                    sampled_ee_position[2]
                    if (
                        get_random_ee_pose_req.bounding_region.z_min
                        != get_random_ee_pose_req.bounding_region.z_max
                    )
                    else random_ee_pose_unbounded.pose.position.z
                )
                random_ee_pose.orientation = random_ee_pose_unbounded.pose.orientation

                # Check if pose is valid (No exception and plan is not empty)
                try:
                    plan = self.move_group_arm.plan(random_ee_pose)
                    ee_pose_valid = (
                        True if len(plan[1].joint_trajectory.points) != 0 else False
                    )
                except MoveItCommanderException:
                    ee_pose_valid = False

                # Fill response message and break out of loop and if ee pose is valid
                # otherwise keep sampling till the max sample limit has been reached.
                if ee_pose_valid:  # If valid.
                    resp.success = True
                    resp.message = "Everything went OK"
                    resp.ee_pose = random_ee_pose
                    resp.joint_names = plan[1].joint_trajectory.joint_names
                    resp.joint_positions = plan[1].joint_trajectory.points[-1].positions
                    break
                elif n_sample >= max_attempts:
                    if not get_random_pose_srvs_exception:
                        rospy.logwarn(
                            "No valid random EE pose could be found within the set "
                            "boundary region within the set number of attempts "
                            f"({max_attempts}). Please make sure that the robot "
                            "end-effector can reach the points inside the bounding "
                            "region. Unbounded random EE pose returned instead."
                        )
                        resp.success = False
                        resp.message = (
                            "Random ee pose could not be retrieved within the boundary "
                            "within the set attempts. Unbounded random EE pose "
                            "returned instead."
                        )
                        resp.ee_pose = random_ee_pose_unbounded.pose
                    else:
                        resp.success = False
                        resp.message = "Random ee pose could not be retrieved."
                    break
                else:
                    rospy.logwarn(
                        "Failed to sample a valid random end effector pose from the "
                        f"bounding region. Trying again ({n_sample})."
                    )
                    n_sample += 1
        return resp

    def _get_controlled_joints_cb(self, _):
        """Returns the joints that are currently being controlled by moveit.

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
            resp.controlled_joints = (
                flatten_list(
                    [
                        self._controlled_joints_dict["hand"],
                        self._controlled_joints_dict["arm"],
                    ]
                )
                if self._joint_states.name[0] in self._controlled_joints_dict["hand"]
                else flatten_list(
                    [
                        self._controlled_joints_dict["arm"],
                        self._controlled_joints_dict["hand"],
                    ]
                )
            )
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
        pose_header = Header(
            frame_id=add_box_req.frame_id if add_box_req.frame_id else "world"
        )
        box_pose = Pose(
            position=add_box_req.pose.position,
            orientation=normalize_quaternion(add_box_req.pose.orientation),
        )

        # Send request.
        resp = AddBoxResponse()
        try:
            self.scene.add_box(
                add_box_req.name if add_box_req.name else "box",
                PoseStamped(header=pose_header, pose=box_pose)
                if add_box_req.pose
                else PoseStamped(orientation=Quaternion(0, 0, 0, 1)),
                size=add_box_req.size if add_box_req.size else (1, 1, 1),
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
        pose_header = Header(frame_id=add_plane_req.frame_id or "world")
        plane_pose = Pose(
            position=add_plane_req.pose.position,
            orientation=add_plane_req.pose.orientation,
        )
        pose_stamped = PoseStamped(header=pose_header, pose=plane_pose)

        # Send request.
        resp = AddPlaneResponse()
        try:
            self.scene.add_plane(
                add_plane_req.name or "plane",
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
