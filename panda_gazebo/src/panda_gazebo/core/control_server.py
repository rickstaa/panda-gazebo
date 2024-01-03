#! /usr/bin/env python3
"""This server is responsible for controlling the Panda arm. It created several
(action) services that can be used to send control commands to the Panda Robot arm and
hand. These services wrap around the original control topics and services created by
the `franka_ros`_ and `ros_control`_ packages and add extra functionality. They allow
you to ``wait`` for control commands to be completed, send incomplete control commands
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
"""
import copy
import os
import time
from collections import defaultdict
from itertools import compress

import control_msgs.msg as control_msgs
import numpy as np
import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from rospy.exceptions import ROSException, ROSInterruptException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from panda_gazebo.common import ControlledJointsDict
from panda_gazebo.common.helpers import (
    action_server_exists,
    controller_list_array_2_dict,
    get_duplicate_list,
    get_unique_list,
    list_2_human_text,
    lower_first_char,
    panda_action_msg_2_control_msgs_action_msg,
    ros_exit_gracefully,
    translate_actionclient_result_error_code,
)
from panda_gazebo.exceptions import InputMessageInvalidError
from panda_gazebo.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from panda_gazebo.srv import (
    GetControlledJoints,
    GetControlledJointsResponse,
    SetGripperWidth,
    SetGripperWidthRequest,
    SetGripperWidthResponse,
    SetJointCommands,
    SetJointCommandsResponse,
    SetJointEfforts,
    SetJointEffortsResponse,
    SetJointPositions,
    SetJointPositionsResponse,
)

# Global script variables.
GRASP_EPSILON = 0.003  # NOTE: Uses 'kGraspRestingThreshold' from 'franka_gripper.sim.h'
GRASP_SPEED = 0.1  # NOTE: Uses 'kDefaultGripperActionSpeed' from 'franka_gripper.sim.h'
GRASP_FORCE = 10  # Panda force information: {continuous force: 70N, max_force: 140 N}
GRIPPER_ACTION_TIMEOUT = (
    5  # How long to wait for the 'franka_gripper' action to finish [s].  # noqa: E501
)
DIRNAME = os.path.dirname(__file__)
PANDA_JOINTS = {
    "arm": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ],
    "hand": ["panda_finger_joint1", "panda_finger_joint2"],
}
ARM_POSITION_CONTROLLER = "panda_arm_joint_position_controller"
ARM_EFFORT_CONTROLLER = "panda_arm_joint_effort_controller"
ARM_TRAJ_CONTROLLER = "panda_arm_trajectory_controller"
HAND_CONTROLLER = "franka_gripper"
CONTROLLER_INFO_RATE = 1 / 10  # Rate [hz] for retrieving controller information.
CONNECTION_TIMEOUT = 10  # Service connection timeout [s].


class PandaControlServer(object):
    """Controller server used to send control commands to the simulated Panda Robot.

    Attributes:
        joint_states (:obj:`sensor_msgs.JointState`): The current joint states.
        arm_joint_positions_threshold (float): The current threshold for determining
            whether the arm joint positions are within the given setpoint.
        arm_joint_efforts_threshold (float): The current threshold for determining
            whether the arm joint efforts are within the given setpoint.
        arm_velocity_threshold (float): The current threshold for determining whether
            the arm has zero velocity.
    """

    def __init__(  # noqa: C901
        self,
        autofill_traj_positions=False,
        load_gripper=True,
        load_set_joint_commands_service=True,
        load_arm_follow_joint_trajectory_action=False,
        load_extra_services=False,
        controllers_check_rate=CONTROLLER_INFO_RATE,
    ):
        """Initialise PandaControlServer object.

        Args:
            autofill_traj_positions (bool, optional): Whether you want to automatically
                set the current states as positions when the positions field of the
                joint trajectory message is left empty. Defaults to ``False``.
            load_gripper (boolean, optional): Whether we also want to load the gripper
                control services.
            load_set_joint_commands_service (boolean, optional): Whether the set joint
                commands service should be loaded. This service is used by the
                :ros-gazebo-gym:`ros_gazebo_gym <>` package when the control type is
                **NOT** set to ``trajectory``. Defaults, to ``True``.
            load_arm_follow_joint_trajectory_action (boolean, optional): Whether the
                arm follow joint trajectory action should be loaded. This service is
                used by the :ros-gazebo-gym:`ros_gazebo_gym <>` package when the control
                type is set to ``trajectory``. Defaults, to ``False``.
            load_extra_services (bool, optional): Whether to load extra services that
                are not used by the :ros-gazebo-gym:`ros_gazebo_gym <>` package.
                Defaults to ``False``.
            controllers_check_rate (float, optional): Rate at which the availability of
                the used controllers is checked. Setting this to ``-1`` will check at
                every time step. Defaults to ``0.1`` Hz.

        .. warning::
            Please note that increasing the ``controllers_check_rate`` decreases the
            control frequency.
        """
        self.arm_joint_positions_threshold = 0.07
        self.arm_joint_efforts_threshold = 7
        self.arm_velocity_threshold = 0.07
        self._autofill_traj_positions = autofill_traj_positions
        self._gripper_command_client_connected = False
        self._arm_joint_traj_client_connected = False
        self._load_gripper = load_gripper
        self._controllers_check_rate = controllers_check_rate
        self.__controller_check_t = 0
        self.__controllers = {}
        self.__controlled_joints = {}
        self.__joint_controllers = {}

        # Disable the `gripper_action` gripper width reached check.
        # NOTE: Done to allow grasping with the `gripper_action` service (see #33).
        rospy.set_param("/franka_gripper/gripper_action/width_tolerance", 0.1)

        ########################################
        # Create Panda control services ########
        ########################################

        # Create main PandaControlServer services.
        # NOTE: These services wrap around the controller command topics created by
        # the `ros_control` package to allow the user to wait for control commands
        # and specify incomplete joint control commands (i.e. joint commands that do
        # not contain all the joints).
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/get_controlled_joints' service." % rospy.get_name()
        )
        self._get_controlled_joints_srv = rospy.Service(
            "%s/get_controlled_joints" % rospy.get_name().split("/")[-1],
            GetControlledJoints,
            self._get_controlled_joints_cb,
        )
        if load_set_joint_commands_service:
            rospy.logdebug(
                "Creating '%s/set_joint_commands' service." % rospy.get_name()
            )
            self._set_joint_commands_srv = rospy.Service(
                "%s/set_joint_commands" % rospy.get_name().split("/")[-1],
                SetJointCommands,
                self._set_joint_commands_cb,
            )
        if load_extra_services:
            rospy.logdebug(
                "Creating '%s/panda_arm/set_joint_positions' service."
                % rospy.get_name()
            )
            self._set_arm_joint_positions_srv = rospy.Service(
                "%s/panda_arm/set_joint_positions" % rospy.get_name().split("/")[-1],
                SetJointPositions,
                self._arm_set_joint_positions_cb,
            )
            rospy.logdebug(
                "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
            )
            self._set_arm_joint_efforts_srv = rospy.Service(
                "%s/panda_arm/set_joint_efforts" % rospy.get_name().split("/")[-1],
                SetJointEfforts,
                self._arm_set_joint_efforts_cb,
            )
        # NOTE: The service below serves as a wrapper around the original
        # 'franka_gripper' grasp and move actions. It makes sure that the right action
        # is called and that users only need to set the gripper width while it
        # automatically sets the speed, epsilon and force. It also clips gripper width
        # that are not possible.
        if self._load_gripper:
            rospy.logdebug(
                "Creating '%s/panda_hand/set_gripper_width' service." % rospy.get_name()
            )
            self._set_gripper_width_srv = rospy.Service(
                "%s/panda_hand/set_gripper_width" % rospy.get_name().split("/")[-1],
                SetGripperWidth,
                self._set_gripper_width_cb,
            )
            rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        ########################################
        # Create panda_control publishers and ##
        # and connect to required (action) #####
        # services and messages. ###############
        ########################################
        if load_set_joint_commands_service:
            # Create arm joint position controller publisher.
            self._arm_joint_position_pub = rospy.Publisher(
                "%s/command" % (ARM_POSITION_CONTROLLER),
                Float64MultiArray,
                queue_size=10,
            )

            # Create arm joint effort publisher.
            self._arm_joint_effort_pub = rospy.Publisher(
                "%s/command" % (ARM_EFFORT_CONTROLLER), Float64MultiArray, queue_size=10
            )

        # Connect to controller_manager services.
        try:
            rospy.logdebug(
                "Connecting to 'controller_manager/list_controllers' service."
            )
            rospy.wait_for_service(
                "controller_manager/list_controllers",
                timeout=rospy.Duration.from_sec(CONNECTION_TIMEOUT),
            )
            self._list_controllers_client = rospy.ServiceProxy(
                "controller_manager/list_controllers", ListControllers
            )
            rospy.logdebug(
                "Connected to 'controller_manager/list_controllers' service!"
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            err_msg = (
                f"Shutting down '{rospy.get_name()}' because no connection could be "
                "established with the 'controller_manager/list_controllers' service."
            )
            ros_exit_gracefully(shutdown_message=err_msg, exit_code=1)

        # Connect to the gripper command action server.
        if self._load_gripper:
            franka_gripper_command_topic = "franka_gripper/gripper_action"
            rospy.logdebug(
                "Connecting to '%s' action service." % franka_gripper_command_topic
            )
            if action_server_exists(franka_gripper_command_topic):
                # Connect to robot control action server.
                self._gripper_command_client = SimpleActionClient(
                    franka_gripper_command_topic,
                    GripperCommandAction,
                )

                # Waits until the action server has started up.
                retval = self._gripper_command_client.wait_for_server(
                    timeout=rospy.Duration.from_sec(CONNECTION_TIMEOUT)
                )
                if not retval:
                    rospy.logwarn(
                        "No connection could be established with the "
                        f"'{franka_gripper_command_topic}' service. The Panda Robot "
                        "environment therefore can not use this action service to "
                        "control the Panda Robot."
                    )
                else:
                    self._gripper_command_client_connected = True
            else:
                rospy.logwarn(
                    "No connection could be established with the "
                    f"'{franka_gripper_command_topic}' service. The Panda Robot "
                    "environment therefore can not use this action service to control "
                    "the Panda Robot."
                )

        ########################################
        # Connect joint state subscriber #######
        ########################################

        # Retrieve current robot joint state and effort information.
        self.joint_states = None
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    "joint_states", JointState, timeout=1.0
                )
            except ROSException:
                rospy.logwarn(
                    "Current joint_states not ready yet, retrying for getting "
                    "'joint_states'"
                )

        # Create joint_state subscriber.
        self._joint_states_sub = rospy.Subscriber(
            "joint_states", JointState, self._joint_states_cb, queue_size=1
        )

        ########################################
        # Create joint trajectory action #######
        # servers ##############################
        ########################################
        # NOTE: Here setup a new action service that serves as a wrapper around the
        # original 'panda_arm_trajectory_controller/follow_joint_trajectory'. By doing
        # this we add the following features to the original action servers.
        #   - The ability to send partial joint messages.
        #   - The ability to send joint trajectory messages that do not specify joints.
        #   - The ability to automatic generate a time axes when the create_time_axis
        #     field is set to True.
        if load_arm_follow_joint_trajectory_action:
            # Connect to the 'panda_arm_trajectory_controller/follow_joint_trajectory'
            # action server.
            rospy.logdebug(
                "Connecting to '{}' action service.".format(
                    "panda_arm_trajectory_controller/follow_joint_trajectory"
                )
            )
            if action_server_exists(
                "panda_arm_trajectory_controller/follow_joint_trajectory"
            ):
                # Connect to robot control action server.
                self._arm_joint_traj_client = SimpleActionClient(
                    "panda_arm_trajectory_controller/follow_joint_trajectory",
                    control_msgs.FollowJointTrajectoryAction,
                )

                # Waits until the action server has started up.
                retval = self._arm_joint_traj_client.wait_for_server(
                    timeout=rospy.Duration(secs=5)
                )
                if not retval:
                    rospy.logwarn(
                        "No connection could be established with the "
                        "'panda_arm_trajectory_controller/follow_joint_trajectory' "
                        "service. The Panda Robot Environment therefore can not use "
                        "this action service to control the Panda Robot."
                    )
                else:
                    self._arm_joint_traj_client_connected = True
            else:
                rospy.logwarn(
                    "No connection could be established with the "
                    "'panda_arm_trajectory_controller/follow_joint_trajectory' "
                    "service. The Panda Robot Environment therefore can not use this "
                    "action service to control the Panda Robot."
                )

            # Setup a new Panda arm joint trajectory action server.
            rospy.logdebug(
                "Creating '%s/panda_arm/follow_joint_trajectory' service."
                % rospy.get_name()
            )
            self._arm_joint_traj_as = SimpleActionServer(
                "%s/panda_arm/follow_joint_trajectory"
                % rospy.get_name().split("/")[-1],
                FollowJointTrajectoryAction,
                execute_cb=self._arm_joint_traj_execute_cb,
                auto_start=False,
            )
            self._arm_joint_traj_as.register_preempt_callback(
                self._arm_joint_traj_preempt_cb
            )
            self._arm_joint_traj_as.start()

    ################################################
    # Panda control member functions ###############
    ################################################
    def _wait_till_arm_control_done(
        self,
        control_type,
        joint_setpoint,
        timeout=5,
        check_gradient=True,
    ):
        """Wait till arm control is finished. Meaning the robot state is within range
        of the joint position and joint effort setpoints (or the velocity is zero).

        Args:
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort`` and ``position``.
            joint_setpoint (list): The setpoint to wait for.
            timeout (int, optional): The timeout when waiting for the control
                to be done. Defaults to ``5``.
            check_gradient (boolean, optional): If enabled the script will also return
                when the gradients become zero. Defaults to ``True``.

        Raises:
            :obj:`ValueError`: Raised when the control_type is invalid.
        """
        control_type = control_type.lower()
        if control_type not in ["position", "effort"]:
            raise ValueError(
                "Please specify a valid control type. Valid values are 'position' & "
                "'effort'."
            )

        # Compute the state masks.
        arm_states_mask = [
            joint in self.controlled_joints[control_type]["arm"]
            for joint in self.joint_states.name
        ]
        if not any(arm_states_mask):
            controller = (
                ARM_POSITION_CONTROLLER
                if control_type == "position"
                else ARM_EFFORT_CONTROLLER
            )
            rospy.logwarn(
                f"Not waiting for control to be completed as no joints appear to be "
                f"controlled when using '{control_type}' control. Please make sure the "
                f"'{controller}' controller that is needed for '{control_type}' "
                "control is loaded."
            )
            return False

        # Wait till robot positions/efforts reach the setpoint or the velocities are
        # not changing anymore.
        timeout_time = rospy.get_rostime() + rospy.Duration.from_sec(timeout)
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:
            joint_states = np.array(
                list(
                    compress(
                        self.joint_states.position
                        if control_type == "position"
                        else self.joint_states.effort,
                        arm_states_mask,
                    )
                )
            )
            state_threshold = (
                self.arm_joint_positions_threshold
                if control_type == "position"
                else self.arm_joint_efforts_threshold
            )
            grad_threshold = self.arm_velocity_threshold
            joint_setpoint_tmp = np.append(
                np.array(joint_setpoint),
                joint_states[len(joint_setpoint) :],  # noqa: E203, E501
            )

            # Add current state to state_buffer and delete oldest entry.
            state_buffer = np.full((2, len(joint_states)), np.nan)
            grad_buffer = np.full((2, len(joint_states)), np.nan)
            state_buffer = np.delete(
                np.append(state_buffer, [joint_states], axis=0), 0, axis=0
            )
            grad_buffer = np.gradient(state_buffer, axis=0)

            # Check if setpoint is reached.
            if (
                np.linalg.norm(joint_states - joint_setpoint_tmp)
                <= state_threshold  # Check if difference norm is within threshold.
            ) and (
                not check_gradient
                or all(
                    (np.abs(val) <= grad_threshold and val != 0.0)
                    for val in grad_buffer[
                        -1
                    ]  # Check if all velocities are close to zero.
                )
            ):
                break

        return True

    def _create_arm_traj_action_server_msg(self, input_msg):  # noqa: C901
        """Converts the ``control_msgs.msg.FollowJointTrajectoryGoal`` message that
        is received by the ``panda_control_server`` follow joint trajectory wrapper
        action servers into the right format for the original ``panda_arm_trajectory_controller``
        `follow_joint_trajectory <https://wiki.ros.org/joint_trajectory_action/>`_
        action server.

        By doing this, we allow our users to send incomplete joint trajectory messages
        (i.e., joint trajectory messages that do not contain all the joints). We also
        allow them to generate the time axes automatically when the ``create_time_axis``
        field is set to ``True``.

        Args:
            input_msg (:obj:`trajectory_msgs/JointTrajectory`): The service input
                message we want to validate.

        Returns:
            dict: A dictionary containing the arm and hand panda_arm_trajectory_controller
                :control_msgs:`control_msgs.msg.FollowJointTrajectoryGoal <html/action/FollowJointTrajectory.html>`
                messages.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                input_msg could not be converted into panda_arm_trajectory_controller control
                messages.
        """  # noqa: E501
        # Retrieve controlled joints and current state of the controlled arm joints.
        input_joints = input_msg.trajectory.joint_names
        controlled_joints = self.controlled_joints["trajectory"]["arm"]
        controlled_joints_size = len(controlled_joints)
        joint_states_dict = {
            name: (position, effort, velocity)
            for name, position, effort, velocity in zip(
                self.joint_states.name,
                self.joint_states.position,
                self.joint_states.effort,
                self.joint_states.velocity,
            )
        }
        current_controlled_arm_state = {
            "positions": [
                states[0]
                for joint, states in joint_states_dict.items()
                if joint in controlled_joints
            ],
            "effort": [
                states[1]
                for joint, states in joint_states_dict.items()
                if joint in controlled_joints
            ],
            "velocities": [
                states[2]
                for joint, states in joint_states_dict.items()
                if joint in controlled_joints
            ],
        }

        # Initialise new arm action server messages using the current robot state.
        arm_control_msg = copy.deepcopy(input_msg)
        arm_control_msg.trajectory.joint_names = controlled_joints
        for idx, waypoint in enumerate(input_msg.trajectory.points):
            point = arm_control_msg.trajectory.points[idx]
            point.positions = list(current_controlled_arm_state["positions"])
            point.effort = list(current_controlled_arm_state["effort"])
            point.velocities = list(current_controlled_arm_state["velocities"])
            point.accelerations = [0.0] * len(arm_control_msg.trajectory.joint_names)

        # Retrieve the length of the positions/velocities/accelerations and efforts
        # fields in all the trajectory waypoints.
        waypoints = input_msg.trajectory.points
        waypoints_lengths_array = {
            "positions": [len(waypoint.positions) for waypoint in waypoints],
            "effort": [len(waypoint.effort) for waypoint in waypoints],
            "velocities": [len(waypoint.velocities) for waypoint in waypoints],
            "accelerations": [len(waypoint.accelerations) for waypoint in waypoints],
        }

        # Validate time axis step size and throw warning if invalid.
        if input_msg.create_time_axis and input_msg.time_axis_step <= 0.0:
            rospy.logwarn(
                f"A time axis step size of '{input_msg.time_axis_step}' is not "
                "supported. Please supply a time axis step greater than 0.0 if you "
                "want to automatically create the trajectory time axis."
            )
            input_msg.create_time_axis = False  # Disable time axis generation.

        # Check action server goal request message.
        if len(input_joints) == 0:  # If not joint_names were given.
            # Check if enough control values were given for the requested joints and
            # throw error if to many were given and warning if to few were given.
            waypoint_types = ["positions", "effort", "velocities", "accelerations"]
            waypoints_lengths_to_big = {}
            waypoints_lengths_to_small = {}
            for waypoint_type in waypoint_types:
                lengths = waypoints_lengths_array[waypoint_type]
                waypoints_lengths_to_big[waypoint_type] = any(
                    length > controlled_joints_size for length in lengths
                )
                waypoints_lengths_to_small[waypoint_type] = any(
                    length < controlled_joints_size and length != 0
                    for length in lengths
                )
            if any(waypoints_lengths_to_big.values()):
                invalid_fields_string = list_2_human_text(
                    [key for key, val in waypoints_lengths_to_big.items() if val]
                )
                logwarn_message = (
                    "Your joint trajectory goal message contains more joint "
                    f"{invalid_fields_string} than the {controlled_joints_size} "
                    "joints that are found in the arm control group."
                )
                raise InputMessageInvalidError(
                    message="Invalid number of joint position commands.",
                    log_message=logwarn_message,
                    controlled_joints=controlled_joints_size,
                    error_code=-1,
                )
            elif any(waypoints_lengths_to_small.values()):
                invalid_fields_string = list_2_human_text(
                    [key for key, val in waypoints_lengths_to_small.items() if val]
                )
                logwarn_message = (
                    "Some of the trajectory waypoints contain less "
                    f"{invalid_fields_string} commands than "
                    f"{controlled_joints_size} joints that are found in the arm "
                    "control group. When this is the case the current joint states "
                    "will be used for the joints without a position command."
                )
                rospy.logwarn_once(logwarn_message)

            # Iterates through waypoints to construct new joint trajectory control
            # message.
            # NOTE: Empty fields are preserved for consistency with the original action
            # server except when 'autofill_traj_positions' is set to True. In that case
            # we make sure to always populate 'positions' with current joint states.
            for idx, waypoint in enumerate(input_msg.trajectory.points):
                # Add time_from_start variable if time axis should be created.
                if input_msg.create_time_axis:
                    arm_control_msg.trajectory.points[
                        idx
                    ].time_from_start = rospy.Duration.from_sec(
                        (idx + 1) * input_msg.time_axis_step
                    )

                # Add joint commands.
                attributes = ["positions", "effort", "velocities", "accelerations"]
                for attribute in attributes:
                    waypoint_attr = getattr(waypoint, attribute)
                    if len(getattr(waypoint, attribute)) != 0:
                        getattr(arm_control_msg.trajectory.points[idx], attribute)[
                            : len(waypoint_attr)
                        ] = list(waypoint_attr)
                    else:
                        # Make sure empty field stays empty.
                        if (
                            attribute == "positions"
                            and not self._autofill_traj_positions
                        ):
                            getattr(
                                arm_control_msg.trajectory.points[idx], attribute
                            ).clear()
                        else:
                            getattr(
                                arm_control_msg.trajectory.points[idx], attribute
                            ).clear()

            return panda_action_msg_2_control_msgs_action_msg(arm_control_msg)

        # Check if enough control values were given for the requested joints and throw
        # error if needed.
        keys = ["positions", "effort", "velocities", "accelerations"]
        waypoints_length_not_equal = {
            key: any(
                length != len(input_joints) and length != 0
                for length in waypoints_lengths_array[key]
            )
            for key in keys
        }
        if any(waypoints_length_not_equal.values()):
            invalid_fields_string = list_2_human_text(
                [key for key, val in waypoints_length_not_equal.items() if val]
            )
            logwarn_message = (
                "Your joint trajectory goal message contains more or less joint "
                f"{invalid_fields_string} than the {controlled_joints_size} "
                "joints that are found in the arm control group."
            )
            raise InputMessageInvalidError(
                message=logwarn_message,
                log_message=logwarn_message,
                joint_names_length=len(input_joints),
                error_code=-1,
            )

        # Validate joint_names and throw error if needed.
        invalid_joint_names = [
            joint_name
            for joint_name in input_joints
            if joint_name not in controlled_joints
        ]
        invalid_count = len(invalid_joint_names)
        if invalid_count != 0:  # Joint names invalid.
            singular = invalid_count == 1
            joint_word = "Joint" if singular else "Joints"
            verb = "was" if singular else "were"
            logwarn_message = (
                f"{joint_word} '{', '.join(invalid_joint_names)}' that {verb} "
                "specified in the 'joint_names' field of the "
                f"'panda_gazebo/SetJointPositions' message {verb} invalid. Valid joint "
                "names for controlling the Panda arm are "
                f"'{', '.join(controlled_joints)}'."
            )
            raise InputMessageInvalidError(
                message="Invalid joint_names were given.",
                log_message=logwarn_message,
                invalid_joint_names=invalid_joint_names,
                error_code=-2,
            )

        # Iterates through waypoints to construct new joint trajectory control message.
        # NOTE: Empty fields are preserved for consistency with the original action
        # server except when 'autofill_traj_positions' is set to True. In that case we
        # make sure to always populate 'positions' with current joint states.
        joint_to_index = {
            joint_name: idx
            for idx, joint_name in enumerate(arm_control_msg.trajectory.joint_names)
        }
        attributes = ["positions", "velocities", "accelerations", "effort"]
        for idx, waypoint in enumerate(input_msg.trajectory.points):
            input_command_dict = {
                attribute: dict(zip(input_joints, getattr(waypoint, attribute)))
                for attribute in attributes
            }

            # Add time_from_start variable if time axis should be created.
            if input_msg.create_time_axis:
                arm_control_msg.trajectory.points[
                    idx
                ].time_from_start = rospy.Duration.from_sec(
                    (idx + 1) * input_msg.time_axis_step
                )

            # Add joint commands.
            for attribute in attributes:
                if len(getattr(waypoint, attribute)) != 0:
                    for joint, value in input_command_dict[attribute].items():
                        if joint in joint_to_index:
                            getattr(arm_control_msg.trajectory.points[idx], attribute)[
                                joint_to_index[joint]
                            ] = value
                else:
                    # Make sure empty field stays empty.
                    if attribute == "positions" and not self._autofill_traj_positions:
                        getattr(
                            arm_control_msg.trajectory.points[idx], attribute
                        ).clear()
                    else:
                        getattr(
                            arm_control_msg.trajectory.points[idx], attribute
                        ).clear()

        return panda_action_msg_2_control_msgs_action_msg(arm_control_msg)

    def _create_arm_control_publisher_msg(self, input_msg, control_type):  # noqa: C901
        """Converts the input message of the arm position/effort control service into a
        control commands that is used by the control publishers. While doing this it
        also verifies whether the given input message is valid.

        Args:
            input_msg (union[:obj:`panda_gazebo.msgs.SetJointPositions`, :obj:`panda_gazebo.msgs.SetJointEfforts`]):
                The service input message we want to convert and validate.
            control_type (str): The type of control that is being executed. Options
                are ``effort`` and ``position``.

        Returns:
            dict: The Panda arm control commands in the order which is are required by
                the publishers.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                input_msg could not be converted into arm joint position/effort
                commands.
        """  # noqa: E501
        valid_control_types = ["position", "effort"]
        if control_type not in valid_control_types:
            raise ValueError(
                "Invalid control type. Valid values are 'position' & 'effort'."
            )

        # Parse control input.
        control_type = control_type.lower()
        input_joints = input_msg.joint_names
        control_input = (
            input_msg.joint_positions
            if control_type == "position"
            else input_msg.joint_efforts
        )

        # Retrieve controlled joints and current joint state.
        controlled_joints = self.controlled_joints[control_type]["arm"]
        joint_values = (
            self.joint_states.position
            if control_type == "position"
            else self.joint_states.effort
        )
        arm_commands_dict = {
            key: val
            for key, val in zip(self.joint_states.name, joint_values)
            if key in controlled_joints
        }

        # Handle the case when no joint_names were given.
        controlled_joints_size = len(controlled_joints)
        if len(input_joints) == 0:
            # Check if enough joint position commands were given otherwise give warning.
            if len(control_input) != controlled_joints_size:
                joint_str = (
                    f"joint {control_type}"
                    if len(control_input) == 1
                    else f"joint {control_type}s"
                )
                joint_names_str = "joint" if controlled_joints_size == 1 else "joints"

                # Check if control input is bigger than controllable joints.
                if len(control_input) != controlled_joints_size:
                    logwarn_message = (
                        f"You specified {len(control_input)} {joint_str} while the "
                        f"Panda arm control group has {controlled_joints_size} "
                        f"{joint_names_str}."
                    )
                    if len(control_input) > controlled_joints_size:
                        raise InputMessageInvalidError(
                            message=f"Invalid number of joint {control_type} commands.",
                            log_message=logwarn_message,
                            joint_commands_length=len(control_input),
                            controlled_joints=controlled_joints_size,
                        )
                    else:
                        logwarn_message += (
                            f" As a result only joints "
                            f"{controlled_joints[: len(control_input)]} will be "
                            "controlled."
                        )
                        rospy.logwarn(logwarn_message)

                        # Add control commands to arm command dict.
                        for k, v in zip(list(arm_commands_dict.keys()), control_input):
                            arm_commands_dict[k] = v
            return Float64MultiArray(data=list(arm_commands_dict.values()))

        # Throw warning if not enough joint commands commands were given.
        if len(input_joints) != len(control_input):
            joint_str = (
                f"joint {control_type}"
                if len(control_input) == 1
                else f"joint {control_type}s"
            )
            joint_names_str = "joint" if len(input_joints) == 1 else "joints"
            logwarn_message = (
                f"You specified {len(control_input)} {joint_str} while the "
                "'joint_names' field of the "
                f"'panda_gazebo/SetJoint{control_type.capitalize()}s' message "
                f"contains {len(input_joints)} {joint_names_str}. "
                f"Please make sure you supply a {joint_str} for each of the joints "
                "contained in the 'joint_names' field."
            )
            raise InputMessageInvalidError(
                message=(
                    f"The 'joint_names' and 'joint_{control_type}s' fields of the "
                    "input message are of different lengths."
                ),
                log_message=logwarn_message,
                joint_commands_length=len(control_input),
                joint_names_length=len(input_joints),
            )

        # Throw error if joint_names are invalid.
        invalid_joint_names = [
            joint_name
            for joint_name in input_joints
            if joint_name not in controlled_joints
        ]
        if invalid_joint_names:  # Joint names invalid.
            joint_str = "Joint" if len(invalid_joint_names) == 1 else "Joints"
            was_were = "was" if len(invalid_joint_names) == 1 else "were"
            logwarn_message = (
                f"{joint_str} {invalid_joint_names} that {was_were} specified in "
                "the 'joint_names' field of the "
                f"'panda_gazebo/SetJoint{control_type.capitalize()}' message "
                f"{was_were} invalid. Valid joint names for controlling the Panda "
                f"arm are {controlled_joints}."
            )
            raise InputMessageInvalidError(
                message="Invalid joint_names were given.",
                log_message=logwarn_message,
                invalid_joint_names=invalid_joint_names,
            )

        # Add control commands to arm command dict.
        for joint, command in zip(input_joints, control_input):
            if joint in arm_commands_dict:
                arm_commands_dict[joint] = command

        # Return command message.
        return Float64MultiArray(data=list(arm_commands_dict.values()))

    def _split_joint_commands_req(self, joint_commands_req, control_type):
        """Splits the combined :obj:`~panda_gazebo.msg.SetJointControlCommandRequest`
        message into separate arm and gripper messages.

        Args:
            joint_commands_req (:obj:`~panda_gazebo.msg.SetJointControlCommandRequest`):
                The joint control command message.
            control_type (str): The type of control that is being executed. Options are
                ``effort``, ``position`` and ``trajectory``

        Returns:
            (tuple): tuple containing:
                - union[:obj:`~panda_gazebo.msg.SetJointPositionsRequest`, :obj:`~panda_gazebo.msg.SetJointEffortsRequest`, ``None``]:
                    The arm set joint position/effort message or ``None`` if no
                    joint positions/efforts were found in the joint control command.
                - union[:obj:`~panda_gazebo.msg.SetGripperWidth`, ``None``]: The gripper
                    action goal message or ``None`` if 'gripper_width' was not found in
                    the joint control command.

        Raises:
            :obj:`ValueError`: Raised when the control_type is invalid.
        """  # noqa: E501
        # Validate control type
        if control_type not in ["position", "effort"]:
            raise ValueError(
                "Please specify a valid control type. Valid values are 'position' & "
                "'effort'."
            )

        # Validate joint commands.
        if len(joint_commands_req.joint_names) != len(
            joint_commands_req.joint_commands
        ):
            rospy.logwarn_once(
                "The length of the joints command 'joint_names' array is "
                f"{len(joint_commands_req.joint_names)} while the length of the "
                f"'joint_commands' array is {len(joint_commands_req.joint_commands)}. "
                "As a result not all joint commands will be applied."
            )
        valid_joints = PANDA_JOINTS["arm"] + ["gripper_width", "gripper_max_effort"]
        invalid_joints = [
            joint
            for joint in joint_commands_req.joint_names
            if joint not in valid_joints
        ]
        if invalid_joints:
            joint_word = "Joint" if len(invalid_joints) == 1 else "Joints"
            validity_word = (
                "it is not a valid"
                if len(invalid_joints) == 1
                else "they are not valid"
            )
            rospy.logwarn_once(
                f"{joint_word} {invalid_joints} will be ignored since {validity_word} "
                f"panda control joint. Valid control joints are {valid_joints}."
            )

        # Create arm and hand control messages.
        joint_commands_dict = dict(
            zip(joint_commands_req.joint_names, joint_commands_req.joint_commands)
        )
        arm_joint_commands_dict = {
            key: val
            for key, val in joint_commands_dict.items()
            if key in PANDA_JOINTS["arm"]
        }
        gripper_width_command = next(
            (
                val
                for key, val in joint_commands_dict.items()
                if key.lower() == "gripper_width"
            ),
            None,
        )
        gripper_max_effort_command = next(
            (
                val
                for key, val in joint_commands_dict.items()
                if key.lower() == "gripper_max_effort"
            ),
            None,
        )
        arm_req = None
        if arm_joint_commands_dict:
            arm_req = (
                SetJointPositions() if control_type == "position" else SetJointEfforts()
            )
            arm_req.joint_names = list(arm_joint_commands_dict.keys())
            arm_req.joint_positions = (
                list(arm_joint_commands_dict.values())
                if control_type == "position"
                else None
            )
            arm_req.joint_efforts = (
                list(arm_joint_commands_dict.values())
                if control_type == "effort"
                else None
            )
            arm_req.wait = joint_commands_req.arm_wait
        gripper_req = None
        if gripper_width_command or gripper_max_effort_command:
            gripper_req = SetGripperWidthRequest()
            gripper_req.grasping = joint_commands_req.grasping
            gripper_req.wait = joint_commands_req.hand_wait
            if gripper_width_command:
                gripper_req.width = gripper_width_command
            if gripper_max_effort_command:
                gripper_req.max_effort = gripper_max_effort_command

        return arm_req, gripper_req

    def _controllers_running(self, controllers):
        """Check if all controllers that are needed for controlling a given control
        group are running.

        Args:
            controllers (list): A list of controllers for which you want to check if
                they are running.

        Returns:
            (tuple): tuple containing:

                - missing_controllers (:obj:`list`): The controllers that are not
                    loaded.
                - stopped_controllers (:obj:`list`): The controllers that are loaded but
                    not started.
        """
        missing_controllers = [
            controller
            for controller in controllers
            if controller not in self.controllers
        ]
        stopped_controllers = [
            controller
            for controller in controllers
            if self.controllers.get(controller)
            and self.controllers.get(controller).state != "running"
        ]
        return missing_controllers, stopped_controllers

    @property
    def arm_trajectory_action_preempted(self):
        """Returns whether the arm joint trajectory action server is preempted."""
        return (
            self._arm_joint_traj_as.is_preempt_requested()
            or self._arm_joint_traj_client.get_state() == GoalStatus.PREEMPTED
        )

    @property
    def controlled_joints(self):  # noqa: C901
        """Returns the joints that can be controlled by a each control type.

        .. important::
            This does not mean that the necessary controller is running. It only means
            that the controller was loaded and can be used to control the joints when
            it is started.

        Returns:
            dict: A dictionary containing the joints that are controlled when using a
                given control type
                (i.e. ``control_type``>``control_group``>``controller``).
        """
        if not self.__controlled_joints:
            controllers = self.controllers
            controlled_joints_dict = ControlledJointsDict()

            # Retrieve the arm joints that can be controlled by each controller.
            controller_types = [
                (ARM_POSITION_CONTROLLER, "position"),
                (ARM_EFFORT_CONTROLLER, "effort"),
                (ARM_TRAJ_CONTROLLER, "trajectory"),
            ]
            for controller, control_type in controller_types:
                try:
                    for claimed_resources in controllers[controller].claimed_resources:
                        for resource in claimed_resources.resources:
                            controlled_joints_dict[control_type]["arm"].append(resource)
                            controlled_joints_dict[control_type]["both"].append(
                                resource
                            )
                except KeyError:  # Controller not loaded.
                    pass

            # Retrieve hand joints that can be controlled by the hand controller.
            try:
                for claimed_resources in controllers[HAND_CONTROLLER].claimed_resources:
                    for resource in claimed_resources.resources:
                        controlled_joints_dict["position"]["hand"].append(resource)
                        controlled_joints_dict["position"]["both"].append(resource)
            except KeyError:  # Controller not loaded.
                pass

            self.__controlled_joints = controlled_joints_dict

        return self.__controlled_joints

    @property
    def joint_controllers(self):
        """Retrieves the controllers which are currently loaded to control each
        joint.

        Returns:
            dict: A dictionary where the keys are joint names and the values are
                lists of controllers that can control the joint.
        """
        if not self.__joint_controllers:
            joint_controllers_dict = defaultdict(list)
            for controller, val in self.controllers.items():
                for resources_item in val.claimed_resources:
                    for resource in resources_item.resources:
                        joint_controllers_dict[resource].append(controller)
            self.__joint_controllers = dict(joint_controllers_dict)
        return self.__joint_controllers

    @property
    def controllers(self):
        """Retrieves info about the loaded controllers.

        .. note:: This method is cached to reduce the number of calls to the
            controller manager and therefore increase performance.

        Returns:
            dict: Dictionary with information about the currently loaded controllers.
        """
        if (
            time.time() - self.__controller_check_t > (self._controllers_check_rate)
            or not self.__controllers
        ):
            self.__controllers = controller_list_array_2_dict(
                self._list_controllers_client.call(ListControllersRequest())
            )
            self.__joint_controllers = {}  # Trigger refresh.
            self.__controlled_joints = {}  # Trigger refresh.
            self.__controller_check_t = time.time()
        return self.__controllers

    @property
    def gripper_width(self):
        """Returns the gripper width as calculated based on the Panda finger joints.

        Returns:
            float: The gripper width.
        """
        return (
            dict(zip(self.joint_states.name, self.joint_states.position))[
                "panda_finger_joint1"
            ]
            * 2
        )

    ################################################
    # Subscribers callback functions ###############
    ################################################
    def _joint_states_cb(self, data):
        """Callback function for the joint data subscriber."""
        self.joint_states = data

    ################################################
    # Control services callback functions ##########
    ################################################
    def _set_joint_commands_cb(self, set_joint_commands_req):
        """Request arm and hand joint command control.

        .. note::
            The gripper `max_effort` is determined based on the `grasping` field if it's
            set to `0`. Specifically, if `grasping` is `True`, `max_effort` is set to
            10N. Otherwise, it remains at `0`.

        Args:
            set_joint_commands_req (:obj:`panda_gazebo.srv.SetJointPositionsRequest`):
                Service request message specifying the joint position/effort commands
                for the robot arm and hand joints.

        Returns:
            :obj:`panda_gazebo.srv.SetJointCommandsResponse`: Service response.
        """
        resp = SetJointCommandsResponse()
        control_type = set_joint_commands_req.control_type.lower()
        if control_type not in ["position", "effort"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are 'position' & "
                "'effort'."
            )
            resp.success = False
            resp.message = "Specified 'control_type' invalid."
            return resp

        # Split input message and send arm and gripper control commands.
        arm_command_msg, gripper_command_msg = self._split_joint_commands_req(
            set_joint_commands_req, control_type
        )

        # Send arm control commands.
        arm_resp = None
        if arm_command_msg is not None:
            arm_resp = (
                self._arm_set_joint_positions_cb(arm_command_msg)
                if control_type == "position"
                else self._arm_set_joint_efforts_cb(arm_command_msg)
            )

        # Send gripper control commands.
        gripper_resp = None
        if gripper_command_msg is not None:
            if self._load_gripper:
                gripper_resp = self._set_gripper_width_cb(gripper_command_msg)
            else:
                rospy.logwarn_once(
                    f"Gripper command could not be set since the '{rospy.get_name()}' "
                    "gripper services were not loaded. Please set the 'load_gripper` "
                    "argument to `True` if you want to control the gripper through the "
                    f"'{rospy.get_name()}'."
                )

        # Check if everything went OK and return response.
        arm_control_failed = (
            arm_command_msg is not None
            and arm_resp is not None
            and not arm_resp.success
        )
        gripper_control_failed = (
            self._load_gripper
            and gripper_command_msg is not None
            and gripper_resp is not None
            and not gripper_resp.success
        )
        if arm_control_failed and gripper_control_failed:
            resp.success = False
            resp.message = "Joint control failed"
        elif arm_control_failed:
            resp.success = False
            resp.message = "Arm control failed"
        elif gripper_control_failed:
            resp.success = False
            resp.message = "Gripper control failed"
        else:
            resp.success = True
            resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_positions_cb(self, set_joint_positions_req):  # noqa: C901
        """Request arm joint position control.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionsRequest`):
                Service request message specifying the positions for the robot arm
                joints.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionsResponse`: Service response.
        """
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:
            joint_word = "joints" if len(duplicate_list) > 1 else "joint"
            rospy.logwarn(
                f"Multiple entries were found for {joint_word} '{duplicate_list}' in "
                f"the '{rospy.get_name()}/panda_arm/set_joint_positions' message. "
                "Consequently, only the first occurrence was used in setting the joint "
                "positions."
            )

            # Remove duplicate entries.
            joint_dict = {}
            for joint_name, joint_position in zip(
                set_joint_positions_req.joint_names,
                set_joint_positions_req.joint_positions,
            ):
                if joint_name not in joint_dict:
                    joint_dict[joint_name] = joint_position
            set_joint_positions_req.joint_names = list(joint_dict.keys())
            set_joint_positions_req.joint_positions = list(joint_dict.values())

        # Check if all controllers are available and running.
        # NOTE: Fail if is missing and display warning if not started.
        resp = SetJointPositionsResponse()
        missing_controllers, stopped_controllers = self._controllers_running(
            [ARM_POSITION_CONTROLLER]
        )
        if missing_controllers:
            controller_word = (
                "controller is" if len(missing_controllers) == 1 else "controllers are"
            )
            rospy.logwarn(
                f"Panda arm joint position command could not be sent as the "
                f"{missing_controllers} {controller_word} not loaded. Please make "
                "sure you load the controller parameters onto the ROS parameter server."
            )
            resp.success = False
            resp.message = "Arm controllers not loaded."
            return resp
        if stopped_controllers:
            # Check if these controllers are required for the current command.
            required_stopped_controllers = []
            if not set_joint_positions_req.joint_names:
                required_stopped_controllers = stopped_controllers
            else:
                for joint in set_joint_positions_req.joint_names:
                    if joint in self.joint_controllers.keys():
                        for stopped_controller in stopped_controllers:
                            if stopped_controller in self.joint_controllers[joint]:
                                required_stopped_controllers.append(stopped_controller)
                required_stopped_controllers = get_unique_list(
                    required_stopped_controllers
                )

            # Display warning if required controllers are not running.
            if required_stopped_controllers:
                controller_word = (
                    "controller is"
                    if len(required_stopped_controllers) == 1
                    else "controllers are"
                )
                rospy.logwarn(
                    f"Panda arm joint positions command sent but probably not executed "
                    f"as the {required_stopped_controllers} {controller_word} not "
                    "running."
                )

        # Validate request and publish control command.
        try:
            control_pub_msgs = self._create_arm_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="position",
            )
        except InputMessageInvalidError as e:
            logwarn_msg = (
                "Panda arm joint positions not set as "
                f"{lower_first_char(e.log_message)}"
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.args[0]
            return resp
        rospy.logdebug("Publishing Panda arm joint positions control message.")
        self._arm_joint_position_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached.
        if set_joint_positions_req.wait and not stopped_controllers:
            self._wait_till_arm_control_done(
                control_type="position",
                joint_setpoint=control_pub_msgs.data,
            )

        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_efforts_cb(self, set_joint_efforts_req):  # noqa: C901
        """Request arm Joint effort control.

        Args:
            set_joint_efforts_req (:obj:`panda_gazebo.srv.SetJointEffortsRequest`):
                Service request message specifying the efforts for the robot arm joints.

        Returns:
            :obj:`panda_gazebo.srv.SetJointEffortsResponse`: Service response.
        """
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:
            joint_word = "joints" if len(duplicate_list) > 1 else "joint"
            rospy.logwarn(
                f"Multiple entries were found for {joint_word} '{duplicate_list}' in "
                f"the '{rospy.get_name()}/panda_arm/set_joint_efforts' message. "
                "Consequently, only the first occurrence was used in setting the joint "
                "efforts."
            )

            # Remove duplicate entries.
            joint_dict = {}
            for joint_name, joint_effort in zip(
                set_joint_efforts_req.joint_names, set_joint_efforts_req.joint_efforts
            ):
                if joint_name not in joint_dict:
                    joint_dict[joint_name] = joint_effort
            set_joint_efforts_req.joint_names = list(joint_dict.keys())
            set_joint_efforts_req.joint_efforts = list(joint_dict.values())

        # Check if all controllers are available and running.
        # NOTE: Fail if is missing and display warning if not started.
        resp = SetJointEffortsResponse()
        missing_controllers, stopped_controllers = self._controllers_running(
            [ARM_EFFORT_CONTROLLER]
        )
        if missing_controllers:
            controller_word = (
                "controller is" if len(missing_controllers) == 1 else "controllers are"
            )
            rospy.logwarn(
                "Panda arm joint effort command could not be send as the "
                f"{missing_controllers} {controller_word} not loaded. Please make "
                "sure you load the controller parameters onto the ROS parameter server."
            )
            resp.success = False
            resp.message = "Arm controllers not loaded."
            return resp
        if stopped_controllers:
            # Check if these controllers are required for the current command.
            required_stopped_controllers = []
            if not set_joint_efforts_req.joint_names:
                required_stopped_controllers = stopped_controllers
            else:
                for joint in set_joint_efforts_req.joint_names:
                    if joint in self.joint_controllers.keys():
                        for stopped_controller in stopped_controllers:
                            if stopped_controller in self.joint_controllers[joint]:
                                required_stopped_controllers.append(stopped_controller)
                required_stopped_controllers = get_unique_list(
                    required_stopped_controllers
                )

            # Display warning if required controllers are not running.
            if required_stopped_controllers:
                controller_word = (
                    "controller is"
                    if len(required_stopped_controllers) == 1
                    else "controllers are"
                )
                rospy.logwarn(
                    f"Panda arm joint efforts command sent but probably not executed "
                    f"as the {required_stopped_controllers} {controller_word} not "
                    "running."
                )

        # Validate request and publish control command.
        try:
            control_pub_msgs = self._create_arm_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="effort",
            )
        except InputMessageInvalidError as e:
            logwarn_msg = (
                f"Panda arm joint efforts not set as {lower_first_char(e.log_message)}"
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.args[0]
            return resp
        rospy.logdebug("Publishing Panda arm joint efforts control message.")
        self._arm_joint_effort_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached.
        # NOTE: We currently do not have to wait for control efforts to be applied
        # since the 'FrankaHWSim' does not yet implement control latency. Torques
        # are therefore applied instantly.
        # if set_joint_efforts_req.wait and not stopped_controllers:
        #     self._wait_till_arm_control_done(
        #         control_type="effort",
        #         joint_setpoint=control_pub_msgs.data,
        #     )

        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _set_gripper_width_cb(self, set_gripper_width_req):
        """Request gripper width.

        .. note::
            The gripper `max_effort` is determined based on the `grasping` field if it's
            set to `0`. Specifically, if `grasping` is `True`, `max_effort` is set to
            10N. Otherwise, it remains at `0`.

        Args:
            set_gripper_width_req (:obj:`panda_gazebo.srv.SetGripperWidth`):
                Service request message specifying the gripper width for the robot hand.

        Returns:
            :obj:`panda_gazebo.srv.SetGripperWidthResponse`: Service response.
        """
        resp = SetGripperWidthResponse()

        # Check if gripper width is within boundaries and convert to finger position.
        set_gripper_width_req.width /= 2
        gripper_width = np.clip(set_gripper_width_req.width, 0, 0.08)
        if gripper_width != set_gripper_width_req.width:
            rospy.logwarn(
                "Gripper width was clipped as it was not within bounds [0, 0.08]."
            )

        # Create gripper command action message.
        # NOTE: The max_effort has to be 0 for the gripper to be able to move (see #33).
        req = GripperCommandGoal()
        req.command.position = gripper_width
        req.command.max_effort = (
            set_gripper_width_req.max_effort
            if set_gripper_width_req.max_effort != 0.0
            else GRASP_FORCE
            if set_gripper_width_req.grasping
            else 0.0
        )

        # Invoke 'franka_gripper' action service
        if self._gripper_command_client_connected:
            self._gripper_command_client.send_goal(req)

            # Wait for result.
            if set_gripper_width_req.wait:
                resp.success = self._gripper_command_client.wait_for_result(
                    timeout=set_gripper_width_req.timeout
                )
            else:
                resp.success = True
        else:
            rospy.logwarn(
                "Cloud not connect to franka_gripper/{} service.".format(
                    "grasp" if set_gripper_width_req.grasping else "move"
                )
            )
            resp.success = False

        # Add result message and return result.
        if resp.success:
            resp.message = "Everything went OK"
        else:
            resp.message = "Gripper width could not be set"
        return resp

    def _get_controlled_joints_cb(self, get_controlled_joints_req):
        """Returns the joints that are controlled when using a given control type.

        Args:
            get_controlled_joints_req (:obj:`panda_gazebo.srv.GetControlledJointsRequest`):
                The service request message specifying the control_type.

        Returns:
            :obj:`panda_gazebo.srv.GetControlledJointsResponse`: The response message
                that contains the ``controlled_joints`` list that specifies the joints
                that are controlled.
        """  # noqa: E501
        resp = GetControlledJointsResponse()
        resp.success = True
        resp.message = "Everything went OK"
        try:
            self.__controlled_joints, self.__controllers = {}, {}  # Trigger refresh.
            controlled_joints = self.controlled_joints[
                get_controlled_joints_req.control_type
            ]
            resp.controlled_joints = controlled_joints["both"]
            resp.controlled_joints_arm = controlled_joints["arm"]
            resp.controlled_joints_hand = controlled_joints["hand"]
        except KeyError:
            resp.success = False
            resp.message = "Controlled joints could not be retrieved"
        return resp

    ################################################
    # Action server callback functions #############
    ################################################
    def _arm_joint_traj_execute_cb(self, goal):
        """Goal execution callback function for the Panda arm joint trajectory action
        server.

        Args:
            goal (:obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`): Goal execution
                action server goal message.
        """
        if not self._arm_joint_traj_client_connected:
            self._set_aborted_result(
                -7, "Could not connect to original Panda arm action server."
            )
            return

        # Check if joint goal.joint_names contains duplicates
        duplicate_list = get_duplicate_list(goal.trajectory.joint_names)
        if duplicate_list:
            joint_word = "joints" if len(duplicate_list) > 1 else "joint"
            rospy.logwarn(
                f"Multiple entries were found for {joint_word} '{duplicate_list}' "
                f"in the '{rospy.get_name()}/panda_arm/follow_joint_trajectory' "
                "message. Consequently, only the first occurrence was used in "
                "setting the joint trajectory."
            )

        # Check the input goal message for errors and if valid convert it to.
        # the right format for the original panda joint trajectory action servers.
        try:
            # Convert input goal message to the right format.
            goal_msg = self._create_arm_traj_action_server_msg(goal)
        except InputMessageInvalidError as e:
            self._arm_joint_traj_as.set_aborted(
                FollowJointTrajectoryResult(
                    error_code=e.details["error_code"], error_string=e.log_message
                )
            )
            rospy.logwarn_once(
                "Joint trajectory control command failed because "
                f"{e.args[0].lower()}"
            )
            return

        # Send trajectory goal to the original Panda arm action servers.
        self._arm_joint_traj_client.send_goal(
            goal_msg, feedback_cb=self._arm_joint_traj_feedback_cb
        )
        done = self._arm_joint_traj_client.wait_for_result(timeout=goal.timeout)

        # Handle timeout.
        if not done:
            self._arm_joint_traj_as.set_aborted(
                FollowJointTrajectoryResult(
                    error_code=-6,
                    error_string="Goal was not executed within the given timeout.",
                )
            )
            return

        # Retrieve original action server result and set it as result for this action.
        result = self._arm_joint_traj_client.get_result()
        result.error_string = translate_actionclient_result_error_code(result)
        if (
            not self.arm_trajectory_action_preempted
            and result.error_code == result.SUCCESSFUL
        ):
            self._arm_joint_traj_as.set_succeeded(result)
        else:  # If not successful or preempted.
            self._arm_joint_traj_as.set_aborted(result)

    def _arm_joint_traj_feedback_cb(self, feedback):
        """Relays the feedback messages from the original
        ``panda_arm_trajectory_controller/follow_joint_trajectory`` server to our to our
        ``panda_control_server/panda_arm/follow_joint_trajectory`` wrapper action
        server.

        Args:
            feedback (:obj:`control_msgs.msg.FollowJointTrajectoryFeedback`): Goal
                execution action server feedback message.
        """
        self._arm_joint_traj_as.publish_feedback(feedback)

    def _arm_joint_traj_preempt_cb(self):
        """Relays the preempt request made to the
        ``panda_control_server/panda_arm/follow_joint_trajectory`` action server wrapper
        to the original ``panda_arm_trajectory_controller/follow_joint_trajectory``
        action server.
        """
        # Stop panda_arm_trajectory_controller action server.
        if self._arm_joint_traj_client.get_state() in [
            GoalStatus.PREEMPTING,
            GoalStatus.ACTIVE,
        ]:
            self._arm_joint_traj_client.cancel_goal()
        self._arm_joint_traj_as.set_preempted()
