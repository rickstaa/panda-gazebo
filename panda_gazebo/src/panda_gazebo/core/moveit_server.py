#! /usr/bin/env python
"""A ros server that creates several of MoveIt services which can be used to control
the Panda robot or retrieve sensor data for the robot.

Main services:
    * panda_arm/set_ee_pose
    * panda_arm/get_ee
    * panda_arm/set_ee
    * panda_arm/get_ee_pose
    * panda_arm/get_ee_rpy
    * set_joint_positions
    * get_random_joint_positions
    * get_random_ee_pose
    * get_controlled_joints

Extra services:
    * panda_arm/set_joint_positions
    * panda_hand/set_joint_positions

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
from collections import OrderedDict
from itertools import compress

import moveit_commander
import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose
from moveit_commander.exception import MoveItCommanderException
from moveit_msgs.msg import DisplayTrajectory
from panda_gazebo.cfg import MoveitServerConfig
from panda_gazebo.common.functions import (
    flatten_list,
    get_duplicate_list,
    get_unique_list,
    joint_state_dict_2_joint_state_msg,
    lower_first_char,
)
from panda_gazebo.common.quaternion import Quaternion
from panda_gazebo.exceptions import InputMessageInvalidError
from panda_gazebo.srv import (
    GetEe,
    GetEePose,
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
from rospy.exceptions import ROSException
from sensor_msgs.msg import JointState

# The maximum number times the get_random_ee_pose service tries to sample from the
# bounding region before ignoring it.
MAX_RANDOM_SAMPLES = 5


class PandaMoveitPlannerServer(object):
    """Used to control or request information from the Panda Robot. This is done using
    the Moveit `moveit_commander` module.

    Attributes:
        robot (:obj:`moveit_commander.robot.RobotCommander`): The Moveit robot
            commander object.
        scene (:obj:`moveit_commander.planning_scene_interface.PlanningSceneInterface`):
            The Moveit robot scene commander object.
        move_group_arm (:obj:`moveit_commander.move_group.MoveGroupCommander`):
            The Moveit arm move group object.
        move_group_hand (:obj:`moveit_commander.move_group.MoveGroupCommander`):
            The Moveit hand move group object.
        ee_pose_target (:obj:`geometry_msgs.msg.Pose`): The last set ee pose.
        joint_positions_target (:obj:`dict`): Dictionary containing the last Panda arm
            and hand joint positions setpoint.
    """

    def __init__(
        self,
        arm_move_group="panda_arm",
        arm_ee_link="panda_link8",
        hand_move_group="hand",
        create_extra_services=False,
    ):
        """Initializes the PandaMoveitPlannerServer object.

        Args:
            arm_move_group (str, optional): The name of the move group you want to use
                for controlling the Panda arm. Defaults to ``panda_arm``.
            arm_ee_link (str, optional): The end effector you want moveit to use when
                controlling the Panda arm. Defaults to ``panda_link8``.
            hand_move_group (str, optional): The name of the move group you want to use
                for controlling the Panda hand. Defaults to ``hand``.
            create_extra_services (bool, optional): Specifies whether the extra services
                should also be created.
        """
        self._joint_state_topic = "/joint_states"

        # Initialize Moveit/Robot/Scene and group commanders
        rospy.logdebug("Initialize Moveit Robot/Scene and group commanders.")
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group_arm = moveit_commander.MoveGroupCommander(arm_move_group)
            self.move_group_hand = moveit_commander.MoveGroupCommander(hand_move_group)
        except Exception as e:
            if "invalid robot mode" in e.args[0]:
                rospy.logerr(
                    "Shutting down '%s' because robot_description was not found."
                    % rospy.get_name()
                )
                sys.exit(0)
            elif len(re.findall("Group '(.*)' was not found", e.args[0])) >= 1:
                if hasattr(self, "move_group_arm"):
                    logerror_move_group = "hand"
                else:
                    logerror_move_group = "arm"
                rospy.logerr(
                    "Shutting down '%s' because Panda %s move group '%s' was not found."
                    % (
                        rospy.get_name(),
                        logerror_move_group,
                        re.match(r"Group \'(.*)\' was not found.", e.args[0]).group(1),
                    )
                )
                sys.exit(0)
            else:
                rospy.logerr(
                    "Shutting down '%s' because %s" % (rospy.get_name(), e.args[0])
                )
                sys.exit(0)

        self.move_group_arm.set_end_effector_link(arm_ee_link)
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=10,
        )

        # Retrieve parameters and create dynamic reconfigure server
        self._get_params()
        self._dyn_reconfigure_srv = Server(MoveitServerConfig, self._dyn_reconfigure_cb)

        ########################################
        # Create node services services ########
        ########################################

        # Create main PandaMoveitPlannerServer services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/panda_arm/set_ee_pose' service." % rospy.get_name()
        )
        self._arm_set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_ee_pose" % rospy.get_name()[1:],
            SetEePose,
            self._arm_set_ee_pose_callback,
        )
        rospy.logdebug("Creating '%s/panda_arm/get_ee' service." % rospy.get_name())
        self._arm_get_ee = rospy.Service(
            "%s/panda_arm/get_ee" % rospy.get_name()[1:],
            GetEe,
            self._arm_get_ee_callback,
        )
        rospy.logdebug("Creating '%s/panda_arm/set_ee' service." % rospy.get_name())
        self._arm_set_ee = rospy.Service(
            "%s/panda_arm/set_ee" % rospy.get_name()[1:],
            SetEe,
            self._arm_set_ee_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/get_ee_pose' service." % rospy.get_name()
        )
        self._arm_get_ee_pose_srv = rospy.Service(
            "%s/panda_arm/get_ee_pose" % rospy.get_name()[1:],
            GetEePose,
            self._arm_get_ee_pose_callback,
        )
        rospy.logdebug("Creating '%s/panda_arm/get_ee_rpy' service." % rospy.get_name())
        self._arm_get_ee_rpy_srv = rospy.Service(
            "%s/panda_arm/get_ee_rpy" % rospy.get_name()[1:],
            GetEeRpy,
            self._arm_get_ee_rpy_callback,
        )
        rospy.logdebug("Creating '%s/set_joint_positions' service." % rospy.get_name())
        self._set_joint_positions_srv = rospy.Service(
            "%s/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/get_random_joint_positions' service." % rospy.get_name()
        )
        self._get_random_joints_positions_srv = rospy.Service(
            "%s/get_random_joint_positions" % rospy.get_name()[1:],
            GetRandomJointPositions,
            self._get_random_joint_positions_callback,
        )
        rospy.logdebug("Creating '%s/get_random_ee_pose' service." % rospy.get_name())
        self._get_random_ee_pose_srv = rospy.Service(
            "%s/get_random_ee_pose" % rospy.get_name()[1:],
            GetRandomEePose,
            self._get_random_ee_pose_callback,
        )
        rospy.logdebug(
            "Creating '%s/get_controlled_joints' service." % rospy.get_name()
        )
        self._get_controlled_joints_srv = rospy.Service(
            "%s/get_controlled_joints" % rospy.get_name()[1:],
            GetMoveItControlledJoints,
            self._get_controlled_joints_cb,
        )

        # Create extra services
        if create_extra_services:
            rospy.logdebug(
                "Creating '%s/panda_arm/set_joint_positions' service."
                % rospy.get_name()
            )
            self._arm_set_joint_positions_srv = rospy.Service(
                "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
                SetJointPositions,
                self._arm_set_joint_positions_callback,
            )
            rospy.logdebug(
                "Creating '%s/panda_hand/set_joint_positions' service."
                % rospy.get_name()
            )
            self._hand_set_joint_positions_srv = rospy.Service(
                "%s/panda_hand/set_joint_positions" % rospy.get_name()[1:],
                SetJointPositions,
                self._hand_set_joint_positions_callback,
            )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        # Initiate service msgs
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
                    self._joint_state_topic, JointState, timeout=1.0
                )
            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting %s"
                    % self._joint_state_topic
                )
        self._controlled_joints_dict = {
            "arm": flatten_list(self.move_group_arm.get_active_joints()),
            "hand": flatten_list(self.move_group_hand.get_active_joints()),
        }

        # Retrieve joint state mask
        # NOTE: Used to split joint_states into arm and hand
        self._arm_states_mask = [
            joint in self._controlled_joints_dict["arm"]
            for joint in self._joint_states.name
        ]
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

    def _execute(self, control_group="both"):
        """Plan and execute a trajectory/pose or orientation setpoints

        Args:
            control_group (str, optional): The robot control group for which you want
                to execute the control. Options are ``arm`` or ``hand`` or ``both``.
                Defaults to ``both``.

        Returns:
            list: List specifying whether the arm and/or hand execution was successfull.
                If ``control_group == "both"`` then ``["arm_success", "hand_success"]``.
        """
        if control_group.lower() == "arm":
            self.arm_plan = self.move_group_arm.plan()
            arm_retval = self.move_group_arm.go(wait=True)
            retval = [arm_retval]
        elif control_group.lower() == "hand":
            self.hand_plan = self.move_group_hand.plan()
            hand_retval = self.move_group_hand.go(wait=True)
            retval = [hand_retval]
        elif control_group.lower() == "both":
            self.arm_plan = self.move_group_arm.plan()
            arm_retval = self.move_group_arm.go(wait=True)
            self.hand_plan = self.move_group_hand.plan()
            hand_retval = self.move_group_hand.go(wait=True)
            retval = [arm_retval, hand_retval]
        else:
            logwarn_msg = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            rospy.logwarn(logwarn_msg)
            retval = [False]
        return retval

    def _create_joint_positions_commands(  # noqa: C901
        self, input_msg, control_group="both", verbose=False
    ):
        """Converts the service input message in `moveit_commander` compatible joint
        position setpoint commands. While doing this it also verifies whether the given
        input message is valid.

        Args:
            input_msg (str): The service input message we want to validate.
            control_group (str, optional): The robot control group for which you want to
                execute the control. Options are ``arm`` or ``hand`` or ``both``.
                Defaults to ``both``.
            verbose (bool): Boolean specifying whether you want to send a warning
                message to the ROS logger.

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

        # Get controlled joints
        if control_group.lower() == "arm":
            controlled_joints = self._controlled_joints_dict["arm"]
        elif control_group.lower() == "hand":
            controlled_joints = self._controlled_joints_dict["hand"]
        elif control_group.lower() == "both":
            controlled_joints = flatten_list(
                [
                    self._controlled_joints_dict["arm"],
                    self._controlled_joints_dict["hand"],
                ]
            )
        else:
            logwarn_msg = (
                "The '%s' control group does not exist. Please specify a valid control "
                "group (options: %s)."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            if verbose:
                rospy.logwarn(logwarn_msg)
            raise InputMessageInvalidError(
                message="Control group '%s' does not exist." % control_group.lower(),
                log_message=logwarn_msg,
            )

        # Generate 'moveit_commander' control command
        controlled_joints_size = len(controlled_joints)
        if len(joint_names) == 0:
            # Check if enough joint position commands were given
            if len(joint_positions) != controlled_joints_size:
                logwarn_msg = "You specified %s while the Panda Robot %s %s %s." % (
                    "%s %s"
                    % (
                        len(joint_positions),
                        "joint position"
                        if len(joint_positions) == 1
                        else "joint positions",
                    ),
                    "arm and hand"
                    if control_group.lower() == "both"
                    else control_group.lower(),
                    "contain" if control_group.lower() == "both" else "contains",
                    "%s %s"
                    % (
                        controlled_joints_size,
                        "active joint"
                        if controlled_joints_size == 1
                        else "active joints",
                    ),
                )
                if verbose:
                    rospy.logwarn(logwarn_msg)
                raise InputMessageInvalidError(
                    message="Invalid number of joint position commands.",
                    log_message=logwarn_msg,
                    details={
                        "joint_positions_command_length": len(joint_positions),
                        "controlled_joints": controlled_joints_size,
                    },
                )

            # Return moveit_commander_control command dictionary
            if control_group.lower() == "arm":
                control_commands = {"arm": joint_positions}
            elif control_group.lower() == "hand":
                control_commands = {"hand": joint_positions}
            else:
                control_commands = {
                    "arm": list(compress(joint_positions, self._arm_states_mask)),
                    "hand": list(compress(joint_positions, self._hand_states_mask)),
                }
            return control_commands
        else:
            # Check if enough control values were given
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
                if verbose:
                    rospy.logwarn(logwarn_msg)
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

            # Validate joint_names
            invalid_joint_names = [
                joint_name
                for joint_name in joint_names
                if joint_name not in controlled_joints
            ]
            if len(invalid_joint_names) != 0:
                logwarn_msg = (
                    "%s %s that %s specified in the 'joint_names' field of the "
                    "'panda_gazebo/SetJointPositions' message %s invalid. Valid "
                    "joint names for controlling the panda %s are %s."
                    % (
                        "Joint" if len(invalid_joint_names) == 1 else "Joints",
                        invalid_joint_names,
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "arm and hand"
                        if control_group.lower() == "both"
                        else control_group.lower(),
                        controlled_joints,
                    )
                )
                if verbose:
                    rospy.logwarn(logwarn_msg)
                raise InputMessageInvalidError(
                    message="Invalid joint_names were given.",
                    log_message=logwarn_msg,
                    details={"invalid_joint_names": invalid_joint_names},
                )

            # Get the current state of the arm and hand
            arm_state_dict = OrderedDict(
                zip(
                    self.move_group_arm.get_active_joints(),
                    self.move_group_arm.get_current_joint_values(),
                )
            )
            hand_state_dict = OrderedDict(
                zip(
                    self.move_group_hand.get_active_joints(),
                    self.move_group_hand.get_current_joint_values(),
                )
            )
            input_command_dict = OrderedDict(zip(joint_names, joint_positions))

            # Update current state dictionary with given joint_position setpoints
            arm_output_command_dict = copy.deepcopy(arm_state_dict)
            hand_output_command_dict = copy.deepcopy(hand_state_dict)
            for (joint, position) in input_command_dict.items():  # Update arm
                if joint in arm_state_dict:
                    arm_output_command_dict[joint] = position
            for (joint, position) in input_command_dict.items():  # Update hand
                if joint in hand_state_dict:
                    hand_output_command_dict[joint] = position

            # Return moveit_commander_control command dictionary
            if control_group.lower() == "arm":
                control_commands = {"arm": list(arm_output_command_dict.values())}
            elif control_group.lower() == "hand":
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
    def _dyn_reconfigure_cb(self, config, level):
        """Dynamic reconfigure callback function.

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
            # Update initial values to user supplied parameters
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

            # Set initial scaling factors
            self.move_group_arm.set_max_velocity_scaling_factor(
                config["max_velocity_scaling_factor"]
            )
            self.move_group_hand.set_max_velocity_scaling_factor(
                config["max_velocity_scaling_factor"]
            )
            self.move_group_arm.set_max_acceleration_scaling_factor(
                config["max_acceleration_scaling_factor"]
            )
            self.move_group_hand.set_max_acceleration_scaling_factor(
                config["max_acceleration_scaling_factor"]
            )
        elif level == 0:
            # Update move group velocity settings
            self.move_group_arm.set_max_velocity_scaling_factor(
                config["max_velocity_scaling_factor"]
            )
            self.move_group_hand.set_max_velocity_scaling_factor(
                config["max_velocity_scaling_factor"]
            )
        elif level == 1:
            # Update move group accelleration settings
            self.move_group_arm.set_max_acceleration_scaling_factor(
                config["max_acceleration_scaling_factor"]
            )
            self.move_group_hand.set_max_acceleration_scaling_factor(
                config["max_acceleration_scaling_factor"]
            )
        return config

    def _arm_set_ee_pose_callback(self, set_ee_pose_req):
        """Request the Panda arm to control to a given end effector
        (EE) pose.

        Args:
            set_ee_pose_req :obj:`geometry_msgs.msg.Pose`: The trajectory you want the
                EE to follow.

        Returns:
            :obj:`panda_train.srv.SetEePoseResponse`: Response message containing (
                success bool, message).
        """

        # Make sure quaternion is normalized
        if Quaternion.quaternion_norm(set_ee_pose_req.pose.orientation) != 1.0:
            rospy.logwarn(
                "The quaternion in the set ee pose was normalized since moveit expects "
                "normalized quaternions."
            )
            set_ee_pose_req.pose.orientation = Quaternion.normalize_quaternion(
                set_ee_pose_req.pose.orientation
            )

        # Fill trajectory message
        rospy.logdebug("Setting ee pose.")
        resp = SetEePoseResponse()
        self.ee_pose_target.orientation.x = set_ee_pose_req.pose.orientation.x
        self.ee_pose_target.orientation.y = set_ee_pose_req.pose.orientation.y
        self.ee_pose_target.orientation.z = set_ee_pose_req.pose.orientation.z
        self.ee_pose_target.orientation.w = set_ee_pose_req.pose.orientation.w
        self.ee_pose_target.position.x = set_ee_pose_req.pose.position.x
        self.ee_pose_target.position.y = set_ee_pose_req.pose.position.y
        self.ee_pose_target.position.z = set_ee_pose_req.pose.position.z

        # Send trajectory message and return response
        try:
            self.move_group_arm.set_pose_target(self.ee_pose_target)
            retval = self._execute(control_group="arm")

            # Check if setpoint execution was successfull
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
            set_joint_positions_req (:obj:`panda_train.srv.SetJointPositionRequest`):
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_train.srv.SetJointPositionResponse`: Response message containing
                (success bool, message).
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

        # Create moveit_commander command
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

        # Set joint positions setpoint
        arm_joint_states = self.move_group_arm.get_current_joint_values()
        hand_joint_states = self.move_group_hand.get_current_joint_values()
        rospy.logdebug("Current arm joint positions: %s" % arm_joint_states)
        rospy.logdebug(
            "Arm joint positions setpoint: %s" % moveit_commander_commands["arm"]
        )
        rospy.logdebug("Current hand joint positions: %s" % hand_joint_states)
        rospy.logdebug(
            "Hand joint positions setpoint: %s" % moveit_commander_commands["hand"]
        )
        rospy.logdebug("Setting arm and hand setpoints.")
        self.joint_positions_target = moveit_commander_commands
        set_joint_value_target_success_bool = []
        set_joint_value_target_error_msg = []
        try:
            self.move_group_arm.set_joint_value_target(moveit_commander_commands["arm"])
            set_joint_value_target_success_bool.append(True)
        except MoveItCommanderException as e:
            set_joint_value_target_success_bool.append(False)
            set_joint_value_target_error_msg.append(e.args[0])
        try:
            self.move_group_hand.set_joint_value_target(
                moveit_commander_commands["hand"]
            )
            set_joint_value_target_success_bool.append(True)
        except MoveItCommanderException as e:
            set_joint_value_target_success_bool.append(False)
            set_joint_value_target_error_msg.append(e.args[0])

        # Print error message if an error occurred and return
        if set_joint_value_target_error_msg:
            log_warn_string = (
                "arm and hand"
                if len(set_joint_value_target_error_msg) > 1
                else ("arm" if set_joint_value_target_success_bool[0] else "hand")
            )
            if len(set_joint_value_target_error_msg) > 1:
                rospy.logwarn(
                    "Setting arm joint position targets failed since there was an %s"
                    % (lower_first_char(set_joint_value_target_error_msg[0]))
                )
                rospy.logwarn(
                    "Setting hand joint position targets failed since there was an %s"
                    % (lower_first_char(set_joint_value_target_error_msg[1]))
                )
            else:
                rospy.logwarn(
                    "Setting %s joint position targets failed since there was an %s"
                    % (
                        log_warn_string,
                        lower_first_char(set_joint_value_target_error_msg[0]),
                    )
                )
            resp.success = False
            resp.message = "Failed to set %s setpoints." % log_warn_string
            return resp

        # Execute setpoints
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
            set_joint_positions_req (:obj:`panda_train.srv.SetJointPositionRequest`):
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_train.srv.SetJointPositionResponse`: Response message
                containing (success bool, message).
        """
        rospy.logdebug("Setting arm joint position targets.")

        # Check if set_joint_positions_req.joint_names contains duplicates\
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

        # Create moveit_commander command
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

        # Set joint positions setpoint
        arm_joint_states = self.move_group_arm.get_current_joint_values()
        rospy.logdebug("Current arm joint positions: %s" % arm_joint_states)
        rospy.logdebug(
            "Arm joint positions setpoint: %s" % moveit_commander_commands["arm"]
        )
        self.joint_positions_target = moveit_commander_commands
        rospy.logdebug("Setting arm setpoints.")
        try:
            self.move_group_arm.set_joint_value_target(moveit_commander_commands["arm"])
        except MoveItCommanderException as e:
            rospy.logwarn(
                "Setting arm joint position targets failed since there was an %s"
                % (lower_first_char(e.args[0]))
            )
            resp.success = False
            resp.message = "Failed to set arm setpoints."
            return resp

        # Execute setpoint
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
            set_joint_positions_req (:obj:`panda_train.srv.SetJointPositionRequest`):
                The joint positions you want to control the joints to.

        Returns:
            :obj:`panda_train.srv.SetJointPositionResponse`: Response message
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

        # Create moveit_commander command
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

        # Set joint positions setpoint
        hand_joint_states = self.move_group_hand.get_current_joint_values()
        rospy.logdebug("Current hand joint positions: %s" % hand_joint_states)
        rospy.logdebug(
            "Hand joint positions setpoint: %s" % moveit_commander_commands["hand"]
        )
        self.joint_positions_target = moveit_commander_commands
        rospy.logdebug("Setting hand joint position setpoints.")
        try:
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

        # Execute setpoints
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

    def _arm_get_ee_pose_callback(self, get_ee_pose_req):
        """Request end effector pose.

        Args:
            get_ee_pose_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The current end effector pose.
        """
        rospy.logdebug("Retrieving ee pose.")
        ee_pose = self.move_group_arm.get_current_pose()
        ee_pose_resp = GetEePoseResponse()
        ee_pose_resp = ee_pose.pose
        return ee_pose_resp

    def _arm_get_ee_rpy_callback(self, get_ee_rpy_req):
        """Request current end effector (EE) orientation.

        Args:
            get_ee_rpy_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`panda_train.srv.GetEeResponse`: Response message containing
                containing the roll (x), yaw (z), pitch (y) euler angles.
        """
        rospy.logdebug("Retrieving ee orientation.")
        ee_rpy = self.move_group_arm.get_current_rpy()
        ee_rpy_res = GetEeRpyResponse()
        ee_rpy_res.r = ee_rpy[0]
        ee_rpy_res.y = ee_rpy[1]
        ee_rpy_res.p = ee_rpy[2]
        return ee_rpy_res

    def _arm_get_ee_callback(self, get_ee_req):
        """Request end effector (EE) name.

        Args:
            get_ee_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`panda_train.srv.GetEeResponse`: Response message containing the name
                of the current EE.
        """
        rospy.logdebug("Retrieving ee name.")
        resp = GetEeResponse()
        resp.ee_name = self.move_group_arm.get_end_effector_link()
        return resp

    def _arm_set_ee_callback(self, set_ee_req):
        """Request end effector (EE) change.

        Args:
            set_ee_req (:obj:`panda_gazebo.srv.SetEeRequest`):  Request message
                containing the name of the end effector you want to be set.

        Returns:
            :obj:`panda_train.srv.SetEeResponse`: Response message containing (success
                bool, message).
        """
        rospy.logdebug("Setting ee to '%s'." % set_ee_req.ee_name)
        resp = SetEeResponse()
        if self._link_exists(set_ee_req.ee_name):  # Check if valid
            try:
                self.move_group_arm.set_end_effector_link(set_ee_req.ee_name)
            except MoveItCommanderException as e:
                rospy.logwarn("Ee could not be set.")
                resp = False
                resp.message = e.args[0]
            resp.success = True
            resp.message = "Everything went OK"
        else:
            rospy.logwarn(
                "EE could not be as '%s' is not a valid ee link." % set_ee_req.ee_name
            )
            resp.success = False
            resp.message = "'%s' is not a valid ee link." % set_ee_req.ee_name
        return resp

    def _get_random_joint_positions_callback(  # noqa: C901
        self, get_random_position_req
    ):
        """Returns valid joint position commands for the Panda arm and hand.

        Args:
            get_random_position_req (:obj:`std_srvs.srv.Empty`): Empty request.

        Returns:
            :obj:`panda_train.srv.GetRandomPositionsResponse`: Response message
                containing the random joints positions.
        """
        # Retrieve possible joints
        arm_joints = self.move_group_arm.get_active_joints()
        hand_joints = self.move_group_hand.get_active_joints()
        valid_joint_names = flatten_list(
            [
                [item + "_min", item + "_max"]
                for item in flatten_list([arm_joints, hand_joints])
            ]
        )

        # Validate joint limits if supplied (remove them if invalid)
        if (
            get_random_position_req.joint_limits.names
            and get_random_position_req.joint_limits.values
        ):
            # Check if limit names and limit values are of equal length
            if len(get_random_position_req.joint_limits.names) != len(
                get_random_position_req.joint_limits.values
            ):
                rospy.logwarn(
                    "Joint limits ignored as the number of joints (%s) is "
                    "unequal to the number of limit values (%s)."
                )
                get_random_position_req.joint_limits.names = []
                get_random_position_req.joint_limits.values = []
            else:  # Check if the names in the joint_limits message are valid
                invalid_names = []
                for name in get_random_position_req.joint_limits.names:
                    if name not in valid_joint_names:
                        invalid_names.append(name)

                # Throw warning if a name is invalid and remove joint limits
                if len(invalid_names) != 0:
                    rospy.logwarn(
                        "Joint limits ignored as the the "
                        "'panda_gazebo.msg.JointLimits' field of the "
                        "'panda_gazebo.srv.GetRandomJointPositionsRequest' "
                        "contains %s %s. Valid values are %s."
                        % (
                            "an invalid joint name"
                            if len(invalid_names) == 1
                            else "invalid joint names",
                            invalid_names,
                            valid_joint_names,
                        )
                    )
                    get_random_position_req.joint_limits.names = []
                    get_random_position_req.joint_limits.values = []

        # Retrieve random joint position values
        get_random_arm_joint_positions_srvs_exception = False
        get_random_hand_joint_positions_srvs_exception = False
        try:
            random_arm_joint_values_unbounded = OrderedDict(
                zip(arm_joints, self.move_group_arm.get_random_joint_values())
            )
        except MoveItCommanderException:
            get_random_arm_joint_positions_srvs_exception = True
        try:
            random_hand_joint_values_unbounded = OrderedDict(
                zip(hand_joints, self.move_group_hand.get_random_joint_values())
            )
        except MoveItCommanderException:
            get_random_hand_joint_positions_srvs_exception = True

        # Get random joint positions (while taking into possible joint limits)
        resp = GetRandomJointPositionsResponse()
        random_arm_joint_values = random_arm_joint_values_unbounded  # Fixme: Why here?
        random_hand_joint_values = random_hand_joint_values_unbounded
        if (
            not get_random_position_req.joint_limits.names
            and not get_random_position_req.joint_limits.values
        ):  # If no joint limits were given
            if (
                not get_random_arm_joint_positions_srvs_exception
                and not get_random_hand_joint_positions_srvs_exception
            ):
                random_arm_joint_values = random_arm_joint_values_unbounded
                random_hand_joint_values = random_hand_joint_values_unbounded
                resp.success = True
            else:
                resp.success = False
        else:  # Joint limits were set
            # Create joint limit dictionary
            joint_limits_dict = OrderedDict(
                zip(
                    get_random_position_req.joint_limits.names,
                    get_random_position_req.joint_limits.values,
                )
            )

            # Try to find random joint values within the joint limits
            n_sample = 0
            arm_joint_commands_valid = False
            hand_joint_commands_valid = False
            while True:  # Continue till joint positions are valid or max samples size
                for joint in get_unique_list(
                    [
                        names.replace("_min", "").replace("_max", "")
                        for names in get_random_position_req.joint_limits.names
                    ]
                ):
                    # Sample random value for the given joint within the joint limits
                    if (
                        joint in random_arm_joint_values.keys()
                        and not arm_joint_commands_valid
                    ):
                        random_arm_joint_values[joint] = np.random.uniform(
                            joint_limits_dict[joint + "_min"],
                            joint_limits_dict[joint + "_max"],
                        )
                    if (
                        joint in random_hand_joint_values.keys()
                        and not hand_joint_commands_valid
                    ):
                        random_hand_joint_values[joint] = np.random.uniform(
                            joint_limits_dict[joint + "_min"],
                            joint_limits_dict[joint + "_max"],
                        )

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
                # otherwise keep sampling till the max sample limit has been reached
                if arm_joint_commands_valid and hand_joint_commands_valid:  # If valid
                    resp.success = True
                    break
                elif n_sample >= MAX_RANDOM_SAMPLES:
                    rospy.logwarn(
                        "Ignoring bounding region as the maximum number of sample "
                        "iterations has been reached. Please make sure that the robot "
                        "joints can reach the set joint_limits. "
                    )

                    # Use unbounded joint positions, set success bool and break out of
                    # the loop
                    if (
                        not get_random_arm_joint_positions_srvs_exception
                        and not get_random_hand_joint_positions_srvs_exception
                    ):

                        # Create response message and break out of loop
                        random_arm_joint_values = random_arm_joint_values_unbounded
                        random_hand_joint_values = random_hand_joint_values_unbounded
                        resp.success = True
                        break
                    else:
                        resp.success = False
                else:
                    rospy.logwarn(
                        "Failed to sample valid random joint positions from the "
                        "bounding region. Trying again."
                    )
                    n_sample += 1  # Increase sampling counter

        # Fill and resturn response message
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
            :obj:`panda_train.srv.GetRandomEePoseResponse`: Response message containing
                the random joints positions.
        """
        # Get a random ee pose
        rospy.logdebug("Retrieving a valid random end effector pose.")
        get_random_pose_srvs_exception = False
        try:
            random_ee_pose_unbounded = self.move_group_arm.get_random_pose()
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
        ):  # No bounding region was set
            if not get_random_pose_srvs_exception:
                resp.success = True
                resp.ee_pose = random_ee_pose_unbounded.pose
            else:
                resp.success = False
        else:  # A bounding region was set

            # Try to find a valid ee_pose within the bounding region
            n_sample = 0
            ee_pose_valid = False
            while True:  # Continue till ee pose is valid or max samples size
                if n_sample > 0:
                    rospy.logdebug("Retrieving a valid random end effector pose.")

                # Sample ee position from bounding region
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

                # Create ee_pose msg
                random_ee_pose = Pose()
                random_ee_pose.position.x = sampled_ee_position[0]
                random_ee_pose.position.y = sampled_ee_position[1]
                random_ee_pose.position.z = sampled_ee_position[2]
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
                # otherwise keep sampling till the max sample limit has been reached
                if ee_pose_valid:  # If valid
                    resp.success = True
                    resp.ee_pose = random_ee_pose
                    break
                elif n_sample >= MAX_RANDOM_SAMPLES:
                    rospy.logwarn(
                        "Ignoring bounding region as the maximum number of sample "
                        "iterations has been reached. Please make sure that the robot "
                        "end effector can reach the points inside the bounding region."
                    )

                    # Use unbounded ee_pose, set success bool and break out of the loop
                    if not get_random_pose_srvs_exception:
                        resp.success = True
                        resp.ee_pose = random_ee_pose_unbounded.pose
                        break
                    else:
                        resp.success = False
                else:
                    rospy.logwarn(
                        "Failed to sample a valid random end effector pose from the "
                        "bounding region. Trying again."
                    )
                    n_sample += 1
        return resp

    def _get_controlled_joints_cb(self, get_controlled_joints_req):
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
        return resp
