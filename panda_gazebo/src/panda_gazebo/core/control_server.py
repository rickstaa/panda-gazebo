#! /usr/bin/env python3
"""This server is responsible for controlling the Panda arm. It created a number of
(action) services that can be used to send control commands to the Panda Robot arm and
hand.

Main services:
    * set_joint_positions
    * set_joint_efforts
    * get_controlled_joints
    * follow_joint_trajectory
    * panda_arm/set_joint_positions
    * panda_arm/set_joint_efforts
    * panda_arm/follow_joint_trajectory
"""
# TODO: Check if the current services still work.
# TODO: Add combined panda_control effort service
# TODO: Add combined panda_control position service
# TODO: Add cartesian joint impedance control
# TODO: Add gravity compensation effort control.

import copy
import os
import sys
from collections import OrderedDict
from itertools import compress

import actionlib
import numpy as np
import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from panda_gazebo.common import ActionClientState
from panda_gazebo.common.functions import (
    action_server_exists,
    controller_list_array_2_dict,
    flatten_list,
    get_duplicate_list,
    list_2_human_text,
    lower_first_char,
    panda_action_msg_2_control_msgs_action_msg,
    translate_actionclient_result_error_code,
)
from panda_gazebo.core.group_publisher import GroupPublisher
from panda_gazebo.exceptions import InputMessageInvalidError
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from panda_gazebo.srv import (
    GetControlledJoints,
    GetControlledJointsResponse,
    SetJointEfforts,
    SetJointEffortsResponse,
    SetJointPositions,
    SetJointPositionsResponse,
)
from rospy.exceptions import ROSException, ROSInterruptException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header

# Global script variables
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
ARM_POSITION_CONTROLLERS = [
    "panda_arm_joint1_position_controller",
    "panda_arm_joint2_position_controller",
    "panda_arm_joint3_position_controller",
    "panda_arm_joint4_position_controller",
    "panda_arm_joint5_position_controller",
    "panda_arm_joint6_position_controller",
    "panda_arm_joint7_position_controller",
]
ARM_EFFORT_CONTROLLERS = [
    "panda_arm_joint1_effort_controller",
    "panda_arm_joint2_effort_controller",
    "panda_arm_joint3_effort_controller",
    "panda_arm_joint4_effort_controller",
    "panda_arm_joint5_effort_controller",
    "panda_arm_joint6_effort_controller",
    "panda_arm_joint7_effort_controller",
]
ARM_TRAJ_CONTROLLERS = ["panda_arm_controller"]
ARM_TRAJ_ACTION_SERVER_TOPIC = "panda_arm_controller/follow_joint_trajectory"


class PandaControlServer(object):
    """Controller server used to send control commands to the simulated Panda Robot.

    Attributes:
        joints (:obj:`sensor_msgs.JointState`): The current joint states.
        controllers (dict): Dictionary with information about the currently loaded
            controllers.
        joint_positions_setpoint (dict): Dictionary containing the last Panda arm and
            hand positions setpoint.
        joint_efforts_setpoint (dict): Dictionary containing the last Panda arm and hand
            efforts setpoint.
        joint_positions_threshold (int): The current threshold for determining whether
            the joint positions are within the given setpoint.
        joint_efforts_threshold (int): Threshold for determining whether the joint
            efforts are within the given setpoint.
        autofill_traj_positions (bool): Whether you want to automatically set the
            current states as positions when the positions field of the joint trajectory
            message is left empty.
        arm_position_controllers (list): List containing the names of the position
            controllers used for the arm.
        arm_effort_controllers (list): List containing the names of the effort
            controllers used for the arm.
    """

    def __init__(  # noqa: C901
        self,
        autofill_traj_positions=False,
        connection_timeout=10,
    ):
        """Initializes the PandaControlServer object.

        Args:
            autofill_traj_positions (bool, optional): Whether you want to automatically
                set the current states as positions when the positions field of the
                joint trajectory message is left empty. Defaults to ``False``.
            connection_timeout (int, optional): The timeout for connecting to the
                controller_manager services. Defaults to 3 sec.
        """
        self.joint_positions_setpoint = []
        self.joint_efforts_setpoint = []
        self.joint_positions_threshold = 0.01
        self.joint_efforts_threshold = 0.01
        self.autofill_traj_positions = autofill_traj_positions
        self._wait_till_done_timeout = rospy.Duration(8)
        self._joint_efforts_grad_threshold = 0.01
        self._joint_positions_grad_threshold = 0.01
        self._joint_state_topic = "joint_states"
        self._as_arm_feedback = FollowJointTrajectoryFeedback()
        self._as_hand_feedback = FollowJointTrajectoryFeedback()
        self._as_feedback = FollowJointTrajectoryFeedback()

        ########################################
        # Create Panda control services ########
        ########################################

        # Create main PandaControlServer services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/get_controlled_joints' service." % rospy.get_name()
        )
        self._get_controlled_joints_srv = rospy.Service(
            "%s/get_controlled_joints" % rospy.get_name()[1:],
            GetControlledJoints,
            self._get_controlled_joints_cb,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self._set_arm_joint_positions_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._arm_set_joint_positions_cb,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self._set_arm_joint_efforts_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._arm_set_joint_efforts_cb,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        ########################################
        # Create panda_control publishers and ##
        # and connect to required services.   ##
        ########################################

        # Create arm joint position controller publishers
        self._arm_joint_position_pub = GroupPublisher()
        for position_controller in ARM_POSITION_CONTROLLERS:
            self._arm_joint_position_pub.append(
                rospy.Publisher(
                    "%s/command" % (position_controller),
                    Float64,
                    queue_size=10,
                )
            )

        # Create arm joint effort publishers
        self._arm_joint_effort_pub = GroupPublisher()
        for effort_controller in ARM_EFFORT_CONTROLLERS:
            self._arm_joint_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % (effort_controller),
                    Float64,
                    queue_size=10,
                )
            )

        # Connect to controller_manager services
        try:
            rospy.logdebug(
                "Connecting to 'controller_manager/list_controllers' service."
            )
            rospy.wait_for_service(
                "controller_manager/list_controllers", timeout=connection_timeout
            )
            self.list_controllers_client = rospy.ServiceProxy(
                "controller_manager/list_controllers", ListControllers
            )
            rospy.logdebug(
                "Connected to 'controller_manager/list_controllers' service!"
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException) as e:
            rospy.logerr(
                "Shutting down '%s' because no connection could be established "
                "with the '%s' service and this service is needed "
                "when using 'position_control'."
                % (
                    rospy.get_name(),
                    e.args[0].strip("timeout exceeded while waiting for service"),
                )
            )
            sys.exit(0)

        ########################################
        # Connect joint state subscriber #######
        ########################################

        # Retrieve current robot joint state and effort information
        self.joint_states = None
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    self._joint_state_topic, JointState, timeout=1.0
                )

                # Set joint setpoint to current position
                self.joint_positions_setpoint = self.joint_states.position
                self.joint_efforts_setpoint = self.joint_states.effort
            except ROSException:
                rospy.logwarn(
                    "Current joint_states not ready yet, retrying for getting %s"
                    % self._joint_state_topic
                )

        # Create joint_state subscriber
        self._joint_states_sub = rospy.Subscriber(
            self._joint_state_topic, JointState, self._joints_cb
        )

        ########################################
        # Get controller information ###########
        ########################################

        # Set panda_control to right controller type (Group or individual)
        self.arm_position_controllers = ARM_POSITION_CONTROLLERS
        self.arm_effort_controllers = ARM_EFFORT_CONTROLLERS
        self.arm_traj_controllers = ARM_TRAJ_CONTROLLERS

        # Retrieve the names of the joints that are controlled when we use joint
        # trajectory control
        try:
            self._controlled_joints_traj_control = self._get_controlled_joints(
                control_type="traj_control"
            )
        except InputMessageInvalidError:
            self._controlled_joints_traj_control = PANDA_JOINTS

        ########################################
        # Create joint trajectory action #######
        # servers ##############################
        ########################################
        # NOTE: Here setup a new action service that serves as a wrapper around the
        # original 'panda_arm_controller/follow_joint_trajectory'. By doing this
        # we add the following features to the original action servers.
        #   - The ability to send partial joint messages.
        #   - The ability to send joint trajectory messages that do not specify joints.
        #   - The ability to automatic generate a time axes when the create_time_axis
        #     field is set to True.

        # Connect to original 'panda_arm_controller/follow_joint_trajectory' action
        # server
        rospy.logdebug(
            "Connecting to '%s' action service." % ARM_TRAJ_ACTION_SERVER_TOPIC
        )
        if action_server_exists(ARM_TRAJ_ACTION_SERVER_TOPIC):  # Check if exists

            # Connect to robot control action server
            self._arm_joint_traj_client = actionlib.SimpleActionClient(
                ARM_TRAJ_ACTION_SERVER_TOPIC, FollowJointTrajectoryAction
            )

            # Waits until the action server has started up
            retval = self._arm_joint_traj_client.wait_for_server(
                timeout=rospy.Duration(secs=5)
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot."
                    % (ARM_TRAJ_ACTION_SERVER_TOPIC)
                )
                self._arm_joint_traj_connected = False
            else:
                self._arm_joint_traj_connected = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot." % (ARM_TRAJ_ACTION_SERVER_TOPIC)
            )
            self._arm_joint_traj_connected = False

        # Setup a new Panda arm joint trajectory action server
        rospy.logdebug(
            "Creating '%s/panda_arm/follow_joint_trajectory' service."
            % rospy.get_name()
        )
        self._arm_joint_traj_as = actionlib.SimpleActionServer(
            "%s/panda_arm/follow_joint_trajectory" % rospy.get_name()[1:],
            FollowJointTrajectoryAction,
            execute_cb=self._arm_joint_traj_execute_cb,
            auto_start=False,
        )
        self._arm_joint_traj_as.register_preempt_callback(
            self._arm_joint_traj_preempt_cb
        )
        self._arm_joint_traj_as.start()

        # Initialize joint trajectory feedback messages
        self._init_action_servers_fb_msgs()

    ################################################
    # Panda control member functions ###############
    ################################################
    def _init_action_servers_fb_msgs(self):
        """Initiate the ``panda_control_server/panda_arm/follow_joint_trajectory``
        feedback messages with the current robot states.
        """
        header = Header()
        header.stamp = rospy.get_rostime()

        # Get current robot state ditionary
        state_dict = self._retrieve_state_dict(self._controlled_joints_traj_control)
        arm_state_dict = state_dict["arm"]

        # Fill arm feedback message
        self._as_arm_feedback.joint_names = self._controlled_joints_traj_control["arm"]
        self._as_arm_feedback.header = header
        self._as_arm_feedback.actual.positions = arm_state_dict["positions"].values()
        self._as_arm_feedback.actual.velocities = arm_state_dict["velocities"].values()
        self._as_arm_feedback.actual.effort = arm_state_dict["efforts"].values()

    def _get_combined_action_server_fb_msg(self):
        """Combine the action server feedback messages from the original
        ``panda_arm_controller/follow_joint_trajectory`` and
        ``panda_hand_controller/follow_joint_trajectory`` action servers so that they
        can be used as a feedback message to the
        ``panda_control_server/follow_joint_trajectory`` wrapper action server.

        Returns:
            :obj:`control_msgs.msg.FollowJointTrajectoryFeedback`: New combined action
                server feedback message.
        """
        # Use newest message as starting point
        if self._as_arm_feedback.header.stamp >= self._as_hand_feedback.header.stamp:
            feedback_combined = copy.deepcopy(self._as_arm_feedback)
        else:
            feedback_combined = copy.deepcopy(self._as_hand_feedback)

        # Combine feedback messages
        if self.joint_states.name[0] in self._controlled_joints_traj_control["arm"]:
            feedback_combined.joint_names = flatten_list(
                [self._as_arm_feedback.joint_names, self._as_hand_feedback.joint_names]
            )
            feedback_combined.actual.positions = flatten_list(
                [
                    list(self._as_arm_feedback.actual.positions),
                    list(self._as_hand_feedback.actual.positions),
                ]
            )
            feedback_combined.actual.velocities = flatten_list(
                [
                    list(self._as_arm_feedback.actual.velocities),
                    list(self._as_hand_feedback.actual.velocities),
                ]
            )
            feedback_combined.actual.accelerations = flatten_list(
                [
                    list(self._as_arm_feedback.actual.accelerations),
                    list(self._as_hand_feedback.actual.accelerations),
                ]
            )
            feedback_combined.actual.effort = flatten_list(
                [
                    list(self._as_arm_feedback.actual.effort),
                    list(self._as_hand_feedback.actual.effort),
                ]
            )
            feedback_combined.desired.positions = flatten_list(
                [
                    list(self._as_arm_feedback.desired.positions),
                    list(self._as_hand_feedback.desired.positions),
                ]
            )
            feedback_combined.desired.velocities = flatten_list(
                [
                    list(self._as_arm_feedback.desired.velocities),
                    list(self._as_hand_feedback.desired.velocities),
                ]
            )
            feedback_combined.desired.accelerations = flatten_list(
                [
                    list(self._as_arm_feedback.desired.accelerations),
                    list(self._as_hand_feedback.desired.accelerations),
                ]
            )
            feedback_combined.desired.effort = flatten_list(
                [
                    list(self._as_arm_feedback.desired.effort),
                    list(self._as_hand_feedback.desired.effort),
                ]
            )
            feedback_combined.error.positions = flatten_list(
                [
                    list(self._as_arm_feedback.error.positions),
                    list(self._as_hand_feedback.error.positions),
                ]
            )
            feedback_combined.error.velocities = flatten_list(
                [
                    list(self._as_arm_feedback.error.velocities),
                    list(self._as_hand_feedback.error.velocities),
                ]
            )
            feedback_combined.error.accelerations = flatten_list(
                [
                    list(self._as_arm_feedback.error.accelerations),
                    list(self._as_hand_feedback.error.accelerations),
                ]
            )
            feedback_combined.error.effort = flatten_list(
                [
                    list(self._as_arm_feedback.error.effort),
                    list(self._as_hand_feedback.error.effort),
                ]
            )
        else:
            feedback_combined.joint_names = flatten_list(
                [self._as_hand_feedback.joint_names, self._as_arm_feedback.joint_names]
            )
            feedback_combined.actual.positions = flatten_list(
                [
                    list(self._as_hand_feedback.actual.positions),
                    list(self._as_arm_feedback.actual.positions),
                ]
            )
            feedback_combined.actual.velocities = flatten_list(
                [
                    list(self._as_hand_feedback.actual.velocities),
                    list(self._as_arm_feedback.actual.velocities),
                ]
            )
            feedback_combined.actual.accelerations = flatten_list(
                [
                    list(self._as_hand_feedback.actual.accelerations),
                    list(self._as_arm_feedback.actual.accelerations),
                ]
            )
            feedback_combined.actual.effort = flatten_list(
                [
                    list(self._as_hand_feedback.actual.effort),
                    list(self._as_arm_feedback.actual.effort),
                ]
            )
            feedback_combined.desired.positions = flatten_list(
                [
                    list(self._as_hand_feedback.desired.positions),
                    list(self._as_arm_feedback.desired.positions),
                ]
            )
            feedback_combined.desired.velocities = flatten_list(
                [
                    list(self._as_hand_feedback.desired.velocities),
                    list(self._as_arm_feedback.desired.velocities),
                ]
            )
            feedback_combined.desired.accelerations = flatten_list(
                [
                    list(self._as_hand_feedback.desired.accelerations),
                    list(self._as_arm_feedback.desired.accelerations),
                ]
            )
            feedback_combined.desired.effort = flatten_list(
                [
                    list(self._as_hand_feedback.desired.effort),
                    list(self._as_arm_feedback.desired.effort),
                ]
            )
            feedback_combined.error.positions = flatten_list(
                [
                    list(self._as_hand_feedback.error.positions),
                    list(self._as_arm_feedback.error.positions),
                ]
            )
            feedback_combined.error.velocities = flatten_list(
                [
                    list(self._as_hand_feedback.error.velocities),
                    list(self._as_arm_feedback.error.velocities),
                ]
            )
            feedback_combined.error.accelerations = flatten_list(
                [
                    list(self._as_hand_feedback.error.accelerations),
                    list(self._as_arm_feedback.error.accelerations),
                ]
            )
            feedback_combined.error.effort = flatten_list(
                [
                    list(self._as_hand_feedback.error.effort),
                    list(self._as_arm_feedback.error.effort),
                ]
            )
        return feedback_combined

    def _retrieve_state_dict(self, controlled_joints_dict):
        """Retrieves the current Panda arm and hand position, velocity and effort states
        in dictionary form.

        Args:
            controlled_joints_dict (dict): Dictionary containing the joints that are
                currently being controlled.

        Returns:
            dict: A dictionary containing the current Panda arm and hand positions,
                velocities and efforts.
        """
        # Retrieve arm position, velocity and effort states as a dictionary
        arm_state_position_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        list(self.joint_states.position),
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )
        arm_state_velocities_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        list(self.joint_states.velocity),
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )
        arm_state_efforts_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        list(self.joint_states.effort),
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )

        # Retrieve hand position, velocity and effort states as a dictionary
        hand_state_position_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        list(self.joint_states.position),
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )
        hand_state_velocities_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        list(self.joint_states.velocity),
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )
        hand_state_efforts_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        list(self.joint_states.effort),
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )

        return {
            "arm": {
                "positions": arm_state_position_dict,
                "velocities": arm_state_velocities_dict,
                "efforts": arm_state_efforts_dict,
            },
            "hand": {
                "positions": hand_state_position_dict,
                "velocities": hand_state_velocities_dict,
                "efforts": hand_state_efforts_dict,
            },
        }

    def _wait_till_done(  # noqa: C901
        self, control_type, control_group="both", timeout=None, controlled_joints=None
    ):
        """Wait control is finished. Meaning the robot state is within range of the
        Joint position and joint effort setpoints.

        Args:
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort_control`` and
                ``position_control``.
            control_group (str, optional): The control group  for which the control is
                being performed. Defaults to "both".
            controlled_joints (dict, optional): A dictionary containing the joints that
                are currently being controlled, these joints will be determined if no
                dictionary is given.
            connection_timeout (int, optional): The timeout when waiting for the control
                to be done. Defaults to :attr:`_wait_till_done_timeout`.
        """
        # Validate control type and control group
        if control_type not in ["position_control", "effort_control"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            return False
        else:
            control_type = control_type.lower()
        if control_group not in ["arm", "hand", "both"]:
            rospy.logwarn(
                "The control group '%s' you specified is not valid. Valid values are "
                "%s. Control group 'both' used instead." % ("['arm', 'hand', 'both']")
            )
            control_group = "both"
        else:
            control_group = control_group.lower()

        # Compute the state masks
        if controlled_joints:
            arm_states_mask = [
                joint in controlled_joints["arm"] for joint in self.joint_states.name
            ]
            hand_states_mask = [
                joint in controlled_joints["hand"] for joint in self.joint_states.name
            ]
        else:  # Try to determine the controlled joints and state mask
            try:
                controlled_joints = self._get_controlled_joints(
                    control_type=control_type
                )
                arm_states_mask = [
                    joint in controlled_joints["arm"]
                    for joint in self.joint_states.name
                ]
                hand_states_mask = [
                    joint in controlled_joints["hand"]
                    for joint in self.joint_states.name
                ]
            except InputMessageInvalidError as e:
                rospy.loginfo(
                    "Not waiting for control to be completed as no information could "
                    "be retrieved about which joints are controlled when using '%s' "
                    "control. Please make sure the '%s' controllers that are needed "
                    "for '%s' control are initialized."
                    % (
                        control_type,
                        e.details["controlled_joints"],
                        control_type,
                    )
                )
                return False

        # Get set input arguments
        if timeout:
            timeout = rospy.Duration(timeout)
        else:
            timeout = self._wait_till_done_timeout

        # Select the right mask for the control_group
        if control_group == "arm":
            states_mask = arm_states_mask
        elif control_group == "hand":
            states_mask = hand_states_mask
        else:
            states_mask = [
                any(bool_tuple) for bool_tuple in zip(arm_states_mask, hand_states_mask)
            ]

        # Wait till robot positions/efforts are not changing anymore
        # NOTE: We have to use the std to determine whether the control was finished
        # as the velocity in the joint_states topic is wrong (see issue 14)
        # Improve: We can use real gazebo velocity if issue 14 is fixed.
        timeout_time = rospy.get_rostime() + timeout
        positions_buffer = np.full((2, len(self.joint_states.position)), np.nan)
        positions_grad = np.full((2, len(self.joint_states.position)), np.nan)
        efforts_buffer = np.full((2, len(self.joint_states.effort)), np.nan)
        efforts_grad = np.full((2, len(self.joint_states.effort)), np.nan)
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:

            # Wait till joint positions are within range or arm not changing anymore
            if control_type == "position_control":

                # Retrieve positions setpoint
                joint_positions_setpoint = flatten_list(
                    [
                        self.joint_positions_setpoint["hand"],
                        self.joint_positions_setpoint["arm"],
                    ]
                    if hand_states_mask[0]
                    else [
                        self.joint_positions_setpoint["arm"],
                        self.joint_positions_setpoint["hand"],
                    ]
                )

                # Add state to position to buffer
                positions_buffer = np.append(
                    positions_buffer, [self.joint_states.position], axis=0
                )
                positions_buffer = np.delete(
                    positions_buffer, 0, axis=0
                )  # Delete oldest entry
                positions_grad = np.gradient(positions_buffer, axis=0)

                # Check if joint states are within setpoints
                if (
                    np.linalg.norm(
                        np.array(
                            list(compress(self.joint_states.position, states_mask))
                        )
                        - np.array(
                            list(compress(joint_positions_setpoint, states_mask))
                        )
                    )
                    <= self.joint_positions_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self._joint_positions_grad_threshold
                            and val != 0.0
                        )
                        for val in list(compress(positions_grad[-1], states_mask))
                    ]
                ):  # Check if difference norm is within threshold
                    break

            # Check if joint effort states are within setpoint
            elif control_type == "effort_control":

                # Retrieve positions setpoint
                joint_efforts_setpoint = flatten_list(
                    [
                        self.joint_efforts_setpoint["hand"],
                        self.joint_efforts_setpoint["arm"],
                    ]
                    if hand_states_mask[0]
                    else [
                        self.joint_efforts_setpoint["arm"],
                        self.joint_efforts_setpoint["hand"],
                    ]
                )

                # Add state to effort to buffer
                efforts_buffer = np.append(
                    efforts_buffer, [self.joint_states.effort], axis=0
                )
                efforts_buffer = np.delete(
                    efforts_buffer, 0, axis=0
                )  # Delete oldest entry
                efforts_grad = np.gradient(efforts_buffer, axis=0)
                if (
                    np.linalg.norm(
                        np.array(list(compress(self.joint_states.effort, states_mask)))
                        - np.array(list(compress(joint_efforts_setpoint, states_mask)))
                    )
                    <= self.joint_efforts_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self._joint_efforts_grad_threshold
                            and val != 0.0
                        )
                        for val in list(compress(efforts_grad[-1], states_mask))
                    ]
                ):  # Check if difference norm is within threshold
                    break
            else:
                rospy.loginfo(
                    "Not waiting for control to be completed as '%s' is not "
                    "a valid control type."
                )

        return True

    def _create_arm_traj_action_server_msg(  # noqa: C901
        self, input_msg, verbose=False
    ):
        """Converts the ``control_msgs.msg.FollowJointTrajectoryGoal`` message that
        is received by the ``panda_control_server`` follow joint trajectory wrapper
        action servers into the right format for the original ``panda_arm_controller``
        `follow_joint_trajectory <https://wiki.ros.org/joint_trajectory_action/>`_
        action server. # TODO: WHY convert?

        Args:
            input_msg (:obj:`trajectory_msgs/JointTrajectory`): The service input
                message we want to validate.
            verbose (bool): Boolean specifying whether you want to send a warning
                message to the ROSlogger.

        Returns:
            dict: A dictionary containing the arm and hand panda_arm_controller
                :control_msgs:`control_msgs.msg.FollowJointTrajectoryGoal <html/action/FollowJointTrajectory.html>`
                messages.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                input_msg could not be converted into panda_arm_controller control
                messages.
        """  # noqa: E501
        # Retrieve controlled joints
        try:
            controlled_joints_dict = self._get_controlled_joints(
                control_type="traj_control", verbose=verbose
            )
        except InputMessageInvalidError:
            logwarn_message = (
                "The joint trajectory publisher message could not be created as the "
                "'%s' are not initialized." % (self.arm_traj_controllers)
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Required controllers not initialised.",
                details={"controlled_joints": self.arm_effort_controllers},
                log_message=logwarn_message,
            )

        # Get needed contolled joints information out of dictionary
        controlled_joints = controlled_joints_dict["arm"]
        controlled_joints_size = len(controlled_joints)

        # Retrieve joint names and current robot states
        state_dict = self._retrieve_state_dict(controlled_joints_dict)
        arm_state_dict = state_dict["arm"]
        joint_names = input_msg.trajectory.joint_names

        # Initiate new arm and hand action server messages using the current robot state
        arm_control_msg = copy.deepcopy(input_msg)
        arm_control_msg.trajectory.joint_names = controlled_joints_dict["arm"]
        for idx, waypoint in enumerate(input_msg.trajectory.points):
            arm_control_msg.trajectory.points[idx].positions = list(
                arm_state_dict["positions"].values()
            )
            arm_control_msg.trajectory.points[idx].velocities = list(
                arm_state_dict["velocities"].values()
            )
            arm_control_msg.trajectory.points[idx].effort = list(
                arm_state_dict["efforts"].values()
            )
            arm_control_msg.trajectory.points[idx].accelerations = [0.0] * len(
                arm_control_msg.trajectory.joint_names
            )

        # Retrieve the length of the positions/velocities/accelerations and efforts in
        # all the trajectory waypoints
        waypoints_position_lengths_array = [
            len(waypoint.positions) for waypoint in input_msg.trajectory.points
        ]
        waypoints_velocity_lengths_array = [
            len(waypoint.velocities) for waypoint in input_msg.trajectory.points
        ]
        waypoints_acceleration_lengths_array = [
            len(waypoint.accelerations) for waypoint in input_msg.trajectory.points
        ]
        waypoints_effort_lengths_array = [
            len(waypoint.effort) for waypoint in input_msg.trajectory.points
        ]

        # Validate time axis step size and throw warning if invalid
        if input_msg.time_axis_step <= 0.0 and input_msg.create_time_axis:
            rospy.logwarn(
                "A time axis step size of %s is not supported. Please supply a time "
                "axis step greater than 0.0 if you want to automatically create the "
                "trajectory time axis." % input_msg.time_axis_step
            )
            input_msg.create_time_axis = False  # Disable time axis generation

        # Check action server goal request message
        if len(joint_names) == 0:  # If not joint_names were given

            # Check if the number of joint positions/velocities/accelerations or
            # efforts is unequal to the the number of joints
            waypoints_length_not_equal = [
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in waypoints_position_lengths_array
                    ]
                ),
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in waypoints_velocity_lengths_array
                    ]
                ),
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in waypoints_acceleration_lengths_array
                    ]
                ),
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in waypoints_effort_lengths_array
                    ]
                ),
            ]

            # Check if trajectory message is valid
            if any(waypoints_length_not_equal):

                # Check if the number of joint positions/velocities/accelerations or
                # efforts exceeds the number of joints
                waypoints_length_to_big = {
                    "positions": any(
                        [
                            length > controlled_joints_size
                            for length in waypoints_position_lengths_array
                        ]
                    ),
                    "velocities": any(
                        [
                            length > controlled_joints_size
                            for length in waypoints_velocity_lengths_array
                        ]
                    ),
                    "accelerations": any(
                        [
                            length > controlled_joints_size
                            for length in waypoints_acceleration_lengths_array
                        ]
                    ),
                    "joint_efforts": any(
                        [
                            length > controlled_joints_size
                            for length in waypoints_effort_lengths_array
                        ]
                    ),
                }

                # Throw error if aforementioned is the case
                if any(waypoints_length_to_big.values()):
                    invalid_fields_string = list_2_human_text(
                        [key for key, val in waypoints_length_to_big.items() if val]
                    )
                    logwarn_message = (
                        "Your joint trajectory goal message contains more joint %s "
                        "than the %s joints that are found in the arm control group."
                        % (
                            invalid_fields_string,
                            controlled_joints_size,
                        )
                    )
                    if verbose:
                        rospy.logwarn(logwarn_message)
                    raise InputMessageInvalidError(
                        message="Invalid number of joint position commands.",
                        log_message=logwarn_message,
                        details={"controlled_joints": controlled_joints_size},
                    )
                elif any(
                    [
                        length < controlled_joints_size
                        for length in waypoints_position_lengths_array
                    ]
                ):  # Show warning if less control commands are given than joints
                    logwarn_message = (
                        "Some of the trajectory waypoints contain less position "
                        "commands than %s joints that are found in the arm control "
                        "group. When this is the case the current joint states will be "
                        "used for the joints without a position command."
                        % (controlled_joints_size,)
                    )
                    if verbose:
                        rospy.logwarn(logwarn_message)

            # Loop through input message trajectory waypoints and add control commands
            # to the new trajectory control message
            # NOTE: We make sure empty position/velocity/acceleration and effort fields
            # stay empty to keep coherent to the original action server. For the
            # position field this behaviour can be changed by setting the
            # autofill_traj_positions ROS parameter to True. In this case the position
            # field will always be filled with the current joint states.
            for idx, waypoint in enumerate(input_msg.trajectory.points):
                # Create position/velocity/acceleration and effort control
                # command arrays

                # Add time_from_start variable if create_time_axis == TRUE
                if input_msg.create_time_axis:
                    time_axis_tmp = rospy.Duration.from_sec(
                        (idx * input_msg.time_axis_step) + input_msg.time_axis_step
                    )
                    arm_control_msg.trajectory.points[
                        idx
                    ].time_from_start = time_axis_tmp

                # Add joint position commands
                if len(waypoint.positions) != 0:
                    arm_control_msg.trajectory.points[idx].positions[
                        : len(waypoint.positions)
                    ] = list(waypoint.positions)
                else:
                    # Make sure empty position field stays empty
                    if not self.autofill_traj_positions:
                        arm_control_msg.trajectory.points[idx].positions = []

                # Add joint velocity commands
                if len(waypoint.velocities) != 0:
                    arm_control_msg.trajectory.points[idx].velocities[
                        : len(waypoint.velocities)
                    ] = list(waypoint.velocities)
                else:
                    # Make sure empty velocity field stays empty
                    arm_control_msg.trajectory.points[idx].velocities = []

                # Add joint acceleration commands
                if len(waypoint.accelerations) != 0:
                    arm_control_msg.trajectory.points[idx].accelerations[
                        : len(waypoint.accelerations)
                    ] = list(waypoint.accelerations)
                else:
                    # Make sure empty acceleration field stays empty
                    arm_control_msg.trajectory.points[idx].accelerations = []

                # Add joint effort commands
                if len(waypoint.effort) != 0:
                    arm_control_msg.trajectory.points[idx].effort[
                        : len(waypoint.effort)
                    ] = list(waypoint.effort)
                else:
                    # Make sure empty effort field stays empty
                    arm_control_msg.trajectory.points[idx].effort = []

            return panda_action_msg_2_control_msgs_action_msg(arm_control_msg)
        else:  # If joints were specified

            # Check if the number of joint positions/velocities/accelerations or
            # efforts is unequal to the the number of joints
            waypoints_length_not_equal = {
                "positions": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in waypoints_position_lengths_array
                    ]
                ),
                "velocities": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in waypoints_velocity_lengths_array
                    ]
                ),
                "accelerations": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in waypoints_acceleration_lengths_array
                    ]
                ),
                "efforts": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in waypoints_effort_lengths_array
                    ]
                ),
            }

            # Check if enough control values were given and throw error if needed
            # the case
            if any(waypoints_length_not_equal.values()):
                invalid_fields_string = list_2_human_text(
                    [key for key, val in waypoints_length_not_equal.items() if val]
                )
                logwarn_message = (
                    "Your joint trajectory goal message contains more or less joint %s "
                    "than the %s joints that are found in the 'joint_names' field."
                    % (
                        invalid_fields_string,
                        controlled_joints_size,
                    )
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalidError(
                    message=logwarn_message,
                    log_message=logwarn_message,
                    details={"joint_names_length": len(joint_names)},
                )
            else:
                # Validate joint_names and throw error if needed
                invalid_joint_names = [
                    joint_name
                    for joint_name in joint_names
                    if joint_name not in controlled_joints
                ]
                if len(invalid_joint_names) != 0:  # Joint names invalid
                    logwarn_msg_strings = [
                        "Joint" if len(invalid_joint_names) == 1 else "Joints",
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "panda_gazebo/SetJointPositions",  # NOTE: Why?
                    ]
                    logwarn_message = (
                        "%s that %s specified in the 'joint_names' field of the '%s' "
                        "message %s invalid. Valid joint names for controlling the "
                        "Panda arm are %s."
                        % (
                            "%s %s" % (logwarn_msg_strings[0], invalid_joint_names),
                            logwarn_msg_strings[1],
                            logwarn_msg_strings[2],
                            logwarn_msg_strings[1],
                            controlled_joints,
                        )
                    )
                    if verbose:
                        rospy.logwarn(logwarn_message)
                    raise InputMessageInvalidError(
                        message="Invalid joint_names were given.",
                        log_message=logwarn_message,
                        details={"invalid_joint_names": invalid_joint_names},
                    )
                else:
                    # Loop through waypoints and create the new joint trajectory control
                    # message
                    # NOTE: We make sure empty position/velocity/acceleration and
                    # effort fields stay empty to keep coherent to the original action
                    # server. For the position field this behaviour can be changed by
                    # setting the autofill_traj_positions ROS parameter to True. In this
                    # case the position field will always be filled with the current
                    # joint states.
                    for idx, waypoint in enumerate(input_msg.trajectory.points):

                        # Create input control command dictionary
                        input_command_dict = {
                            "positions": OrderedDict(
                                zip(joint_names, waypoint.positions)
                            ),
                            "velocities": OrderedDict(
                                zip(joint_names, waypoint.velocities)
                            ),
                            "accelerations": OrderedDict(
                                zip(joint_names, waypoint.accelerations)
                            ),
                            "efforts": OrderedDict(zip(joint_names, waypoint.effort)),
                        }

                        # Create position/velocity/acceleration and effort control
                        # command arrays

                        # Add time_from_start variable if create_time_axis == TRUE
                        if input_msg.create_time_axis:
                            time_axis_tmp = rospy.Duration.from_sec(
                                (idx * input_msg.time_axis_step)
                                + input_msg.time_axis_step
                            )
                            arm_control_msg.trajectory.points[
                                idx
                            ].time_from_start = time_axis_tmp

                        # Create position command array
                        if len(waypoint.positions) != 0:
                            arm_position_commands_dict = copy.deepcopy(
                                arm_state_dict["positions"]
                            )  # Start from the current states
                            for (joint, position,) in input_command_dict[
                                "positions"
                            ].items():  # Add control commands
                                if joint in arm_state_dict["positions"]:
                                    arm_position_commands_dict[joint] = position
                            arm_position_commands = arm_position_commands_dict.values()
                        else:
                            # Make sure empty position field stays empty
                            if not self.autofill_traj_positions:
                                arm_position_commands = []

                        # Create velocity command array
                        if len(waypoint.velocities) != 0:
                            arm_velocity_commands_dict = copy.deepcopy(
                                arm_state_dict["velocities"]
                            )  # Start from the current states
                            for (joint, velocity,) in input_command_dict[
                                "velocities"
                            ].items():  # Add control commands
                                if joint in arm_state_dict["velocities"]:
                                    arm_velocity_commands_dict[joint] = velocity
                            arm_velocity_commands = arm_velocity_commands_dict.values()
                        else:
                            # Make sure empty velocity field stays empty
                            arm_velocity_commands = []

                        # Create acceleration command array
                        if len(waypoint.accelerations) != 0:
                            arm_acceleration_commands_dict = OrderedDict(
                                zip(
                                    arm_state_dict["velocities"].keys(),
                                    [0.0] * len(arm_state_dict["velocities"].keys()),
                                )
                            )  # Initiate at 0 as accelerations are unknown
                            for (joint, acceleration,) in input_command_dict[
                                "accelerations"
                            ].items():  # Add control commands
                                if joint in arm_state_dict["velocities"]:
                                    arm_acceleration_commands_dict[joint] = acceleration
                            arm_acceleration_commands = (
                                arm_acceleration_commands_dict.values()
                            )
                        else:
                            # Make sure empty acceleration field stays empty
                            arm_acceleration_commands = []

                        # Create effort command array
                        if len(waypoint.effort) != 0:
                            arm_effort_commands_dict = copy.deepcopy(
                                arm_state_dict["efforts"]
                            )  # Start from the current states
                            for (joint, effort,) in input_command_dict[
                                "efforts"
                            ].items():  # Add control commands
                                if joint in arm_state_dict["efforts"]:
                                    arm_effort_commands_dict[joint] = effort
                            arm_effort_commands = arm_effort_commands_dict.values()
                        else:
                            # Make sure empty effort field stays empty
                            arm_effort_commands = []

                        # Add new control commands to joint trajectory control message
                        # waypoint
                        arm_control_msg.trajectory.points[
                            idx
                        ].positions = arm_position_commands
                        arm_control_msg.trajectory.points[
                            idx
                        ].velocities = arm_velocity_commands
                        arm_control_msg.trajectory.points[
                            idx
                        ].accelerations = arm_acceleration_commands
                        arm_control_msg.trajectory.points[
                            idx
                        ].effort = arm_effort_commands

                    return panda_action_msg_2_control_msgs_action_msg(arm_control_msg)

    def _create_control_publisher_msg(  # noqa: C901
        self, input_msg, control_type, verbose=False
    ):
        """Converts the service input message into a control commands that is used by
        the control publishers. While doing this it also verifies whether the given
        input message is valid.

        Args:
            input_msg (:obj:`trajectory_msgs/JointTrajectory`): The service input
                message we want to validate.
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort_control`` and
                ``position_control``.
            verbose (bool, optional): Boolean specifying whether you want to send a
                warning message to the ROSlogger. Defaults to ``False``.

        Returns:
            (tuple): tuple containing:

                - :obj:`dict`: The Panda arm and hand control commands in the order
                    which is are required by the publishers.
                - :obj:`dict`: The joints that are being controlled.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                input_msg could not be converted into ``moveit_commander`` arm hand
                joint position/effort commands.
        """
        # Validate control_type argument and extract information from input message
        if control_type == "position_control":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_positions
        elif control_type == "effort_control":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_efforts
        else:
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Retrieve controlled joints
        try:
            controlled_joints_dict = self._get_controlled_joints(
                control_type=control_type, verbose=verbose
            )
        except InputMessageInvalidError:
            logwarn_message = (
                "The '%s' publisher messages could not be created as the '%s' %s "
                "are not initialized."
                % (
                    "effort control"
                    if control_type == "effort_control"
                    else "position control",
                    self.arm_effort_controllers
                    if control_type == "effort_control"
                    else self.arm_position_controllers,
                    "joint effort controllers"
                    if control_type == "effort_control"
                    else "joint position controllers",
                )
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Required controllers not initialised.",
                details={"controlled_joints": self.arm_effort_controllers},
                log_message=logwarn_message,
            )

        # Get controlled joints and robot state
        controlled_joints = controlled_joints_dict["arm"]
        controlled_joints_size = len(controlled_joints)
        state_dict = self._retrieve_state_dict(controlled_joints_dict)

        # Get control publisher message type and current joint states
        if control_type == "position_control":
            arm_state_dict = state_dict["arm"]["positions"]
        elif control_type == "effort_control":
            arm_state_dict = state_dict["arm"]["efforts"]

        # Check service request input
        if len(joint_names) == 0:  # If not joint_names were given

            # Check if enough joint position commands were given otherwise give warning
            if len(control_input) != controlled_joints_size:
                if control_type == "position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "joint" if controlled_joints_size == 1 else "joints",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "joint" if controlled_joints_size == 1 else "joints",
                    ]

                # Check if control input is bigger than controllable joints
                if len(control_input) > controlled_joints_size:
                    logwarn_message = (
                        "You specified %s while the Panda arm control group has %s."
                        % (
                            "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                            "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
                        )
                    )
                    if verbose:
                        rospy.logwarn(logwarn_message)
                    raise InputMessageInvalidError(
                        message="Invalid number of joint position commands.",
                        log_message=logwarn_message,
                        details={
                            "joint_positions_command_length": len(control_input),
                            "controlled_joints": controlled_joints_size,
                        },
                    )
                elif len(control_input) < controlled_joints_size:
                    logwarn_message = (
                        "You specified %s while the Panda arm control group has %s."
                        % (
                            "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                            "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
                        )
                        + " As a result only joints %s will be controlled."
                        % (controlled_joints[: len(control_input)])
                    )
                    rospy.logwarn(logwarn_message)

            # Update current state dictionary with given joint_position commands
            arm_position_commands = list(arm_state_dict.values())
            arm_position_commands[: len(control_input)] = control_input

            # Return publishers command dictionary
            control_commands = [Float64(item) for item in arm_position_commands]
            return control_commands, controlled_joints_dict
        else:
            # Check if enough control values were given
            if len(joint_names) != len(control_input):
                if control_type == "position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "panda_gazebo/SetJointPositions",
                        "joint" if len(joint_names) == 1 else "joints",
                        "joint position",
                        "joint_positions",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "panda_gazebo/SetJointEfforts",
                        "joint effort" if len(joint_names) == 1 else "joint efforts",
                        "joint effort",
                        "joint_efforts",
                    ]
                logwarn_message = (
                    "You specified %s while the 'joint_names' field of the "
                    "'%s' message contains %s. Please make sure you supply "
                    "a %s for each of the joints contained in the 'joint_names' "
                    "field."
                    % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        logwarn_msg_strings[1],
                        "%s %s" % (len(joint_names), logwarn_msg_strings[2]),
                        logwarn_msg_strings[3],
                    )
                )

                # Send log warn message and raise InputMessageInvalidError exception
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalidError(
                    message=(
                        "The joint_names and %s fields of the input message are of "
                        "different lengths." % (logwarn_msg_strings[4])
                    ),
                    log_message=logwarn_message,
                    details={
                        logwarn_msg_strings[4] + "_length": len(control_input),
                        "joint_names_length": len(joint_names),
                    },
                )
            else:
                # Validate joint_names
                invalid_joint_names = [
                    joint_name
                    for joint_name in joint_names
                    if joint_name not in controlled_joints
                ]
                if len(invalid_joint_names) != 0:  # Joint names invalid
                    if control_type == "position_control":
                        logwarn_msg_strings = [
                            "Joint" if len(invalid_joint_names) == 1 else "Joints",
                            "was" if len(invalid_joint_names) == 1 else "were",
                            "panda_gazebo/SetJointPositions",
                        ]
                    else:
                        logwarn_msg_strings = [
                            "Joint" if len(invalid_joint_names) == 1 else "Joints",
                            "was" if len(invalid_joint_names) == 1 else "were",
                            "panda_gazebo/SetJointEfforts",
                        ]
                    logwarn_message = (
                        "%s that %s specified in the 'joint_names' field of the '%s' "
                        "message %s invalid. Valid joint names for controlling the "
                        "Panda arm are %s."
                        % (
                            "%s %s" % (logwarn_msg_strings[0], invalid_joint_names),
                            logwarn_msg_strings[1],
                            logwarn_msg_strings[2],
                            logwarn_msg_strings[1],
                            controlled_joints,
                        )
                    )

                    # Send log warn message and raise InputMessageInvalidError exception
                    if verbose:
                        rospy.logwarn(logwarn_message)
                    raise InputMessageInvalidError(
                        message="Invalid joint_names were given.",
                        log_message=logwarn_message,
                        details={"invalid_joint_names": invalid_joint_names},
                    )
                else:
                    # Create input control command dictionary
                    input_command_dict = OrderedDict(zip(joint_names, control_input))

                    # Update current state dictionary with given joint_position commands
                    arm_position_commands_dict = copy.deepcopy(
                        arm_state_dict
                    )  # Start from the current state
                    for (
                        joint,
                        position,
                    ) in input_command_dict.items():  # Add control commands
                        if joint in arm_state_dict:
                            arm_position_commands_dict[joint] = position
                    arm_position_commands = arm_position_commands_dict.values()

                    # Return publishers command dictionary
                    control_commands = [
                        arm_msg_type(item) for item in arm_position_commands
                    ]
                    return control_commands, controlled_joints_dict

    def _get_controlled_joints(self, control_type, verbose=False):  # noqa: C901
        """Returns the joints that can be controlled by a given control type.

        Args:
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort_control`` and
                ``position_control``.
            verbose (bool, optional): Boolean specifying whether you want to send a
                warning message to the ROS logger. Defaults to ``False``.

        Returns:
            dict: A dictionary containing the joints that are controlled when using a
                given control type, grouped by control group (``arm`` and ``hand``).

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                the control_type is invalid.
        """
        control_type = control_type.lower()

        # Get the joints which are contolled by a given control type
        controlled_joints = []
        if control_type == "position_control":
            for position_controller in self.arm_position_controllers:
                try:
                    for claimed_resources in self.controllers[
                        position_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if position_controller in self.arm_position_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif position_controller in self.hand_position_controllers:
                                controlled_joints_dict["hand"].append(resource)
                except KeyError:  # Controller not initialized
                    pass
        elif control_type == "effort_control":
            for effort_controller in self.arm_effort_controllers:
                try:
                    for claimed_resources in self.controllers[
                        effort_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if effort_controller in self.arm_effort_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif effort_controller in self.hand_effort_controllers:
                                controlled_joints_dict["hand"].append(resource)
                except KeyError:  # Controller not initialized
                    pass
        elif control_type == "traj_control" or control_type == "ee_control":
            controlled_joints =
            for traj_controller in self.arm_traj_controllers:
                try:
                    for claimed_resources in self.controllers[
                        traj_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if traj_controller in self.arm_traj_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif traj_controller in self.hand_traj_controllers:
                                controlled_joints_dict["hand"].append(resource)
                except KeyError:  # Controller not initialized
                    pass
        else:
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control', 'traj_control']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Return controlled joints dict
        controlled_joints_dict["arm"] = flatten_list(controlled_joints_dict["arm"])
        controlled_joints_dict["hand"] = flatten_list(controlled_joints_dict["hand"])
        controlled_joints_dict["both"] = (
            flatten_list(
                [controlled_joints_dict["hand"], controlled_joints_dict["arm"]]
            )
            if self.joint_states.name[0] in controlled_joints_dict["hand"]
            else flatten_list(
                [controlled_joints_dict["arm"], controlled_joints_dict["hand"]]
            )
        )
        return controlled_joints_dict

    def _get_joint_controllers(self):
        """Retrieves the controllers which are currently initialized to work with a
        given joint.

        Returns:
            dict: Dictionary containing each panda joint and the controllers that are
                able to control these joints.
        """
        # Loop through active controllers
        joint_controllers_dict = {}
        for (key, val) in self.controllers.items():
            for resources_item in val.claimed_resources:
                for resource in resources_item.resources:
                    if resource in joint_controllers_dict.keys():
                        joint_controllers_dict[resource].append(key)
                    else:
                        joint_controllers_dict[resource] = [key]

        return joint_controllers_dict

    @property
    def controllers(self):
        """Retrieves information about the currently running controllers."""
        return controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

    ################################################
    # Subscribers callback functions ###############
    ################################################
    def _joints_cb(self, data):
        """Callback function for the joint data subscriber."""
        self.joint_states = data

    ################################################
    # Control services callback functions ##########
    ################################################
    def _arm_set_joint_positions_cb(self, set_joint_positions_req):  # noqa: C901
        """Request arm joint position control.

        Args:
            set_joint_positions_req (:obj:`panda_gazebo.srv.SetJointPositionsRequest`):
                Service request message specifying the positions for the robot arm
                joints.

        Returns:
            :obj:`panda_gazebo.srv.SetJointPositionsResponse`: Service response.
        """
        # Check if set_joint_efforts_req.joint_names contains duplicates
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

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointPositionsResponse()

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for position_controller in self.arm_position_controllers:
            try:
                if self.controllers[position_controller].state != "running":
                    stopped_controllers.append(position_controller)
            except KeyError:
                missing_controllers.append(position_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda arm joint position command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (
                    missing_controllers,
                    "joint position controller is"
                    if len(missing_controllers) == 1
                    else "joint position controllers are",
                )
            )
            resp.success = False
            resp.message = "Arm controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="position_control",
            )
        except InputMessageInvalidError as e:
            logwarn_msg = "Panda arm joint positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(stopped_controllers) >= 1:

            # Set controllers missing boolean
            controllers_missing = True

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_positions_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers)
            else:
                for joint in set_joint_positions_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            if log_warning:
                rospy.logwarn(
                    "Panda arm joint positions command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint position controller is"
                        if len(stopped_controllers) == 1
                        else "joint position controllers are",
                    )
                )
        # Save position control setpoint
        self.joint_positions_setpoint = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }

        # Publish request
        rospy.logdebug("Publishing Panda arm joint positions control message.")
        self._arm_joint_position_pub.publish(control_pub_msgs["arm"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait and not controllers_missing:
            self._wait_till_done(
                control_type="position_control",
                control_group="arm",
                controlled_joints=controlled_joints,
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
        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "efforts."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_arm/set_joint_efforts" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointEffortsResponse()

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for effort_controller in self.arm_effort_controllers:
            try:
                if self.controllers[effort_controller].state != "running":
                    stopped_controllers.append(effort_controller)
            except KeyError:
                missing_controllers.append(effort_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda arm joint effort command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (
                    missing_controllers,
                    "joint effort controller is"
                    if len(missing_controllers) == 1
                    else "joint effort controllers are",
                )
            )
            resp.success = False
            resp.message = "Arm controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="effort_control",
            )
        except InputMessageInvalidError as e:
            logwarn_msg = "Panda arm joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required controllers are running
        if len(stopped_controllers) >= 1:

            # Set controllers missing boolean
            controllers_missing = True

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_efforts_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers)
            else:
                for joint in set_joint_efforts_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            if log_warning:
                rospy.logwarn(
                    "Panda arm joint efforts command send but probably not executed as "
                    "the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint effort controller is"
                        if len(stopped_controllers) == 1
                        else "joint effort controllers are",
                    )
                )

        # Save effort control setpoint
        self.joint_efforts_setpoint = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }

        # Publish request
        rospy.logdebug("Publishing Panda arm joint efforts control message.")
        self._arm_joint_effort_pub.publish(control_pub_msgs["arm"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait and not controllers_missing:
            self._wait_till_done(
                control_type="effort_control",
                control_group="arm",
                controlled_joints=controlled_joints,
            )

        resp.success = True
        resp.message = "Everything went OK"
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
        try:
            controlled_joints_dict = self._get_controlled_joints(
                control_type=get_controlled_joints_req.control_type
            )
            resp.controlled_joints = controlled_joints_dict["both"]
            resp.controlled_joints_arm = controlled_joints_dict["arm"]
            resp.controlled_joints_hand = controlled_joints_dict["hand"]
        except InputMessageInvalidError:
            resp.success = False
        return resp

    ################################################
    # Action server callback functions #############
    ################################################
    def _arm_joint_traj_execute_cb(self, goal):
        """Goal execution callback function for the Panda arm joint trajectory action
        server.

        Args:
            Goal (:obj:`control_msgs.msg.FollowJointTrajectoryGoal`): Goal execution
                action server goal message.
        """
        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(goal.trajectory.joint_names)
        if duplicate_list:
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "trajectory."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_arm/follow_joint_trajectory" % rospy.get_name(),
                )
            )
        error_occurred = False

        # Check the input goal message for errors and if valid convert it to
        # the right format for the original panda joint trajectory action servers.
        try:
            # Convert input goal message to the right format
            goal_msg_dict = self._create_arm_traj_action_server_msg(goal)
        except InputMessageInvalidError as e:
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = (
                -2 if e.args[0] == "Invalid joint_names were given." else -6
            )
            self._result.error_string = e.log_message
            self._arm_joint_traj_as.set_aborted(result=self._result)
            error_occurred = True

        # Abort joint trajectory control if original panda arm control server is not
        # available
        if not self._arm_joint_traj_connected:
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = -7
            self._result.error_string = (
                "Could not connect to original Panda arm action server."
            )
            self._arm_joint_traj_as.set_aborted(result=self._result)

        # Execute joint trajectory goal
        if not error_occurred:

            # Send goal to the original Panda arm action servers
            self._arm_joint_traj_client.send_goal(
                goal_msg_dict["arm"], feedback_cb=self._arm_joint_traj_feedback_cb
            )

            # Wait for the server to finish performing the action
            self._arm_joint_traj_client.wait_for_result()

            # Get result from action server
            self._result = self._arm_joint_traj_client.get_result()
            self._state = ActionClientState(self._arm_joint_traj_client)
            self._result.error_string = translate_actionclient_result_error_code(
                self._result
            )

            # Set result
            if not (
                self._arm_joint_traj_as.is_preempt_requested()
                or self._state.state == self._state.state_dict["PREEMPTED"]
            ):
                if self._result.error_code == self._result.SUCCESSFUL:  # If successful
                    self._arm_joint_traj_as.set_succeeded(self._result)
                else:  # If not successful
                    self._arm_joint_traj_as.set_aborted(self._result)

    def _arm_joint_traj_feedback_cb(self, feedback):
        """Relays the feedback messages from the original
        ``panda_arm_controller/follow_joint_trajectory`` server to our to our
        ``panda_control_server/panda_arm/follow_joint_trajectory`` wrapper action
        server.

        Args:
            feedback (:obj:`control_msgs.msg.FollowJointTrajectoryFeedback`): Goal
                execution action server feedback message.
        """
        self._as_arm_feedback = feedback

        # Publish feedback to arm action server
        self._arm_joint_traj_as.publish_feedback(feedback)

    def _arm_joint_traj_preempt_cb(self):
        """Relays the preempt request made to the
        ``panda_control_server/panda_arm/follow_joint_trajectory`` action server wrapper
        to the original ``panda_arm_controller/follow_joint_trajectory`` action server.
        """
        # Stop panda_arm_controller action server
        self._arm_joint_traj_client.cancel_goal()
        self._arm_joint_traj_as.set_preempted()
