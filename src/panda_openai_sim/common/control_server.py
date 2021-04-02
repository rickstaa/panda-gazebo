#! /usr/bin/env python
"""This server is responsible for controlling the Panda arm/hand. It created a number of
(action) services that can be used to send control commands to the Panda Robot arm and
hand. It currently contains joint position, joint effort services and a joint trajectory
action server.
"""

# Main python imports
import sys
import os
from itertools import compress
from collections import OrderedDict
import copy
import numpy as np
import yaml

from panda_openai_sim.functions import (
    action_server_exists,
    controller_list_array_2_dict,
    lower_first_char,
    flatten_list,
    dict_clean,
    get_duplicate_list,
    translate_actionclient_result_error_code,
    list_2_human_text,
    panda_action_msg_2_control_msgs_action_msg,
)
from panda_openai_sim.exceptions import InputMessageInvalidError
from panda_openai_sim.core import GroupPublisher
from panda_openai_sim.extras import ActionClientState

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException
import actionlib

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Header
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import control_msgs.msg as control_msgs

from panda_openai_sim.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
)
from panda_openai_sim.srv import (
    GetControlledJoints,
    GetControlledJointsResponse,
    SetJointPositions,
    SetJointPositionsResponse,
    SetJointEfforts,
    SetJointEffortsResponse,
    ListControlType,
    ListControlTypeResponse,
    SwitchControlType,
    SwitchControlTypeResponse,
)

# Script variables
DIRNAME = os.path.dirname(__file__)
PARAMS_CONFIG_PATH = os.path.abspath(
    os.path.join(DIRNAME, "../../../cfg/_cfg/parms_config.yaml")
)
ARM_POSITION_CONTROLLERS = [
    "panda_arm_joint1_position_controller",
    "panda_arm_joint2_position_controller",
    "panda_arm_joint3_position_controller",
    "panda_arm_joint4_position_controller",
    "panda_arm_joint5_position_controller",
    "panda_arm_joint6_position_controller",
    "panda_arm_joint7_position_controller",
]
ARM_POSITION_GROUP_CONTROLLERS = ["panda_arm_joint_group_position_controller"]
ARM_EFFORT_CONTROLLERS = [
    "panda_arm_joint1_effort_controller",
    "panda_arm_joint2_effort_controller",
    "panda_arm_joint3_effort_controller",
    "panda_arm_joint4_effort_controller",
    "panda_arm_joint5_effort_controller",
    "panda_arm_joint6_effort_controller",
    "panda_arm_joint7_effort_controller",
]
ARM_EFFORT_GROUP_CONTROLLERS = ["panda_arm_joint_group_effort_controller"]
ARM_TRAJ_CONTROLLERS = ["panda_arm_controller"]
HAND_POSITION_CONTROLLERS = [
    "panda_hand_finger1_position_controller",
    "panda_hand_finger2_position_controller",
]
HAND_POSITION_GROUP_CONTROLLERS = ["panda_hand_joint_group_position_controller"]
HAND_EFFORT_CONTROLLERS = [
    "panda_hand_finger1_effort_controller",
    "panda_hand_finger2_effort_controller",
]
HAND_EFFORT_GROUP_CONTROLLERS = ["panda_hand_joint_group_effort_controller"]
HAND_TRAJ_CONTROLLERS = ["panda_hand_controller"]
ARM_TRAJ_ACTION_SERVER_TOPIC = "/panda_arm_controller/follow_joint_trajectory"
HAND_TRAJ_ACTION_SERVER_TOPIC = "/panda_hand_controller/follow_joint_trajectory"


#################################################
# Joint Group Position Controller class #########
#################################################
class PandaControlServer(object):
    """Controller server used to send control commands to the simulated Panda Robot.

    Attributes
    ----------
    joints : sensor_msgs.JointState
        The current joint states.
    joint_positions_setpoint : dict
        Dictionary containing the last Panda arm and hand positions setpoint.
    joint_efforts_setpoint : dict
        Dictionary containing the last Panda arm and hand efforts setpoint.
    joint_positions_threshold : int
        The current threshold for determining whether the joint positions are within
        the given setpoint.
    joint_efforts_threshold : int
        Threshold for determining whether the joint efforts are within
        the given setpoint.
    use_group_controller : bool
        Whether we are using the group controllers or controlling the individual joints.
    autofill_traj_positions : bool
        Whether you want to automatically set the current states as positions when the
        positions field of the joint trajectory message is left empty.
    arm_position_controllers : list
        List containing the names of the position controllers used for the arm.
    arm_effort_controllers : list
        List containing the names of the effort controllers used for the arm.
    hand_position_controllers  : list
        List containing the names of the position controllers used for the hand.
    hand_effort_controller : list
        List containing the names of the effort controllers used for the hand.
    """

    def __init__(
        self,
        use_group_controller=False,
        autofill_traj_positions=False,
        create_all_services=False,
        connection_timeout=10,
    ):
        """Initializes the PandaControlServer object.

        Parameters
        ----------
        use_group_controller : bool, optional
            Whether you want to use the group controllers, by default False.
        autofill_traj_positions : bool, optional
            Whether you want to automatically set the current states as positions when
            the positions field of the joint trajectory message is left empty.
        create_all_services : bool, optional
            Specifies whether we want to create all the available services or only the
            ones that are crucial for the panda_openai_sim package, by default
            False.
        connection_timeout : int, optional
            The timeout for connecting to the controller_manager services,
            by default 3 sec.
        """

        # Try to load package parameters from the package configuration file
        try:
            with open(PARAMS_CONFIG_PATH, "r") as stream:
                try:
                    parms_config = yaml.safe_load(stream)
                except yaml.YAMLError as e:
                    rospy.logwarn(
                        "Shutting down '%s' as the 'panda_openai_sim' parameters "
                        "could not be loaded from the parameter configuration '%s' "
                        "as the following error was thrown: %s"
                        % (rospy.get_name(), PARAMS_CONFIG_PATH, e)
                    )
                    sys.exit(0)
        except FileNotFoundError:
            rospy.logwarn(
                "Shutting down '%s' as the 'panda_openai_sim' package parameters "
                "could not be loaded since the required parameter configuration file "
                "'%s' was not found. Please make sure the configuration file is "
                "present." % (rospy.get_name(), PARAMS_CONFIG_PATH)
            )
            sys.exit(0)
        panda_joints = parms_config["panda_joints"]

        # Create class attributes
        self.joint_positions_setpoint = []
        self.joint_efforts_setpoint = []
        self.joint_positions_threshold = 0.01
        self.joint_efforts_threshold = 0.01
        self.use_group_controller = use_group_controller
        self.autofill_traj_positions = autofill_traj_positions
        self._wait_till_done_timeout = rospy.Duration(8)
        self._joint_efforts_grad_threshold = 0.01
        self._joint_positions_grad_threshold = 0.01
        self._joint_state_topic = "/joint_states"
        self._full_joint_traj_control = False
        self._as_arm_feedback = FollowJointTrajectoryFeedback()
        self._as_hand_feedback = FollowJointTrajectoryFeedback()
        self._as_feedback = FollowJointTrajectoryFeedback()

        #############################################
        # Create Panda control services #############
        #############################################

        # Create main PandaControlServer services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug("Creating '%s/set_joint_positions' service." % rospy.get_name())
        self._set_joint_positions_srv = rospy.Service(
            "%s/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._set_joint_positions_cb,
        )
        rospy.logdebug("Creating '%s/set_joint_efforts' service." % rospy.get_name())
        self._set_joint_efforts_srv = rospy.Service(
            "%s/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._set_joint_efforts_cb,
        )
        rospy.logdebug("Creating '%s/switch_control_type' service." % rospy.get_name())
        self._switch_control_type_srv = rospy.Service(
            "%s/switch_control_type" % rospy.get_name()[1:],
            SwitchControlType,
            self._switch_control_type_cb,
        )
        rospy.logdebug("Creating '%s/list_control_type' service." % rospy.get_name())
        self._list_control_type_srv = rospy.Service(
            "%s/list_control_type" % rospy.get_name()[1:],
            ListControlType,
            self._list_control_type_cb,
        )
        rospy.logdebug(
            "Creating '%s/get_controlled_joints' service." % rospy.get_name()
        )
        self._get_controlled_joints_srv = rospy.Service(
            "%s/get_controlled_joints" % rospy.get_name()[1:],
            GetControlledJoints,
            self._get_controlled_joints_cb,
        )

        # Create other services
        if create_all_services:
            rospy.logdebug(
                "Creating '%s/panda_arm/set_joint_positions' service."
                % rospy.get_name()
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
            rospy.logdebug(
                "Creating '%s/panda_hand/set_joint_positions' service."
                % rospy.get_name()
            )
            self._set_hand_joint_positions_srv = rospy.Service(
                "%s/panda_hand/set_joint_positions" % rospy.get_name()[1:],
                SetJointPositions,
                self._hand_set_joint_positions_cb,
            )
            rospy.logdebug(
                "Creating '%s/panda_hand/set_joint_efforts' service." % rospy.get_name()
            )
            self._set_hand_joint_efforts_srv = rospy.Service(
                "%s/panda_hand/set_joint_efforts" % rospy.get_name()[1:],
                SetJointEfforts,
                self._hand_set_joint_efforts_cb,
            )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        #############################################
        # Create panda_control publishers and #######
        # and connect to required services.   #######
        #############################################

        # Create arm joint position group controller publisher
        self._arm_joint_positions_group_pub = GroupPublisher()
        for position_controller in ARM_POSITION_GROUP_CONTROLLERS:
            self._arm_joint_positions_group_pub.append(
                rospy.Publisher(
                    "%s/command" % (position_controller),
                    Float64MultiArray,
                    queue_size=10,
                )
            )

        # Create (individual) arm joint position controller publishers
        self._arm_joint_position_pub = GroupPublisher()
        for position_controller in ARM_POSITION_CONTROLLERS:
            self._arm_joint_position_pub.append(
                rospy.Publisher(
                    "%s/command" % (position_controller),
                    Float64,
                    queue_size=10,
                )
            )

        # Create arm joint effort group publisher
        self._arm_joint_efforts_group_pub = GroupPublisher()
        for effort_controller in ARM_EFFORT_GROUP_CONTROLLERS:
            self._arm_joint_efforts_group_pub.append(
                rospy.Publisher(
                    "%s/command" % (effort_controller),
                    Float64MultiArray,
                    queue_size=10,
                )
            )

        # Create (individual) arm joint effort publishers
        self._arm_joint_effort_pub = GroupPublisher()
        for effort_controller in ARM_EFFORT_CONTROLLERS:
            self._arm_joint_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % (effort_controller),
                    Float64,
                    queue_size=10,
                )
            )

        # Create hand joint position group publishers
        self._hand_joint_positions_group_pub = GroupPublisher()
        for position_controller in HAND_POSITION_GROUP_CONTROLLERS:
            self._hand_joint_positions_group_pub.append(
                rospy.Publisher(
                    "%s/command" % (position_controller),
                    Float64MultiArray,
                    queue_size=10,
                )
            )

        # Create (individual) hand joint position publishers
        self._hand_joint_position_pub = GroupPublisher()
        for position_controller in HAND_POSITION_CONTROLLERS:
            self._hand_joint_position_pub.append(
                rospy.Publisher(
                    "%s/command" % (position_controller),
                    Float64,
                    queue_size=10,
                )
            )

        # Create hand joint group effort publishers
        self._hand_joint_efforts_group_pub = GroupPublisher()
        for effort_controller in HAND_EFFORT_GROUP_CONTROLLERS:
            self._hand_joint_efforts_group_pub.append(
                rospy.Publisher(
                    "%s/command" % (effort_controller),
                    Float64MultiArray,
                    queue_size=10,
                )
            )

        # Create hand joint group effort publishers
        self._hand_joint_effort_pub = GroupPublisher()
        for effort_controller in HAND_EFFORT_CONTROLLERS:
            self._hand_joint_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % (effort_controller),
                    Float64,
                    queue_size=10,
                )
            )

        # Connect to controller_manager services
        try:

            # Connect to list service
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
                "when using 'joint_position_control'."
                % (
                    rospy.get_name(),
                    e.args[0].strip("timeout exceeded while waiting for service"),
                )
            )
            sys.exit(0)

        #############################################
        # Connect joint state subscriber ############
        #############################################

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
                    "Current /joint_states not ready yet, retrying for getting %s"
                    % self._joint_state_topic
                )

        # Create joint_state subscriber
        self._joint_states_sub = rospy.Subscriber(
            self._joint_state_topic, JointState, self._joints_cb
        )

        #############################################
        # Get controller information ################
        #############################################

        # Set panda_control to right controller type (Group or individual)
        if not self.use_group_controller:
            self.arm_position_controllers = ARM_POSITION_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_CONTROLLERS
            self.arm_traj_controllers = ARM_TRAJ_CONTROLLERS
            self.hand_traj_controllers = HAND_TRAJ_CONTROLLERS
            self._arm_position_pub = self._arm_joint_position_pub
            self._arm_effort_pub = self._arm_joint_effort_pub
            self._hand_position_pub = self._hand_joint_position_pub
            self._hand_effort_pub = self._hand_joint_effort_pub
            self._arm_position_controller_msg_type = Float64
            self._arm_effort_controller_msg_type = Float64
            self._hand_position_controller_msg_type = Float64
            self._hand_effort_controller_msg_type = Float64
        else:
            self.arm_position_controllers = ARM_POSITION_GROUP_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_GROUP_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_GROUP_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_GROUP_CONTROLLERS
            self.arm_traj_controllers = ARM_TRAJ_CONTROLLERS
            self.hand_traj_controllers = HAND_TRAJ_CONTROLLERS
            self._arm_position_pub = self._arm_joint_positions_group_pub
            self._arm_effort_pub = self._arm_joint_efforts_group_pub
            self._hand_position_pub = self._hand_joint_positions_group_pub
            self._hand_effort_pub = self._hand_joint_efforts_group_pub
            self._arm_position_controller_msg_type = Float64MultiArray
            self._arm_effort_controller_msg_type = Float64MultiArray
            self._hand_position_controller_msg_type = Float64MultiArray
            self._hand_effort_controller_msg_type = Float64MultiArray

        # Create combined position/effort controllers lists
        self._position_controllers = flatten_list(
            [self.arm_position_controllers, self.hand_position_controllers]
        )
        self._effort_controllers = flatten_list(
            [self.arm_effort_controllers, self.hand_effort_controllers]
        )
        self._traj_controllers = flatten_list(
            [self.arm_traj_controllers, self.hand_traj_controllers]
        )

        # Retrieve informations about the controllers
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Retrieve the names of the joints that are controlled when we use joint
        # trajectory control
        try:
            self._controlled_joints_traj_control = self._get_controlled_joints(
                control_type="traj_control"
            )
        except InputMessageInvalidError:
            self._controlled_joints_traj_control = panda_joints

        #############################################
        # Create joint trajectory action servers ####
        #############################################
        # NOTE: Here setup three new action services that serve as a wrapper around the
        # original 'panda_arm_controller/follow_joint_trajectory' and
        # 'panda_hand_controller/follow_joint_trajectory' action servers. By doing this
        # we add the following features to the original action servers.
        #   - The ability to send partial joint messages.
        #   - The ability to send joint trajectory messages that do not specify joints.
        #   - The ability to send both arm and hand traj commands at the same time.
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
                ARM_TRAJ_ACTION_SERVER_TOPIC, control_msgs.FollowJointTrajectoryAction
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

        # Connect to original 'panda_hand_controller/follow_joint_trajectory' action
        # server
        rospy.logdebug(
            "Connecting to '%s' action service." % HAND_TRAJ_ACTION_SERVER_TOPIC
        )
        if action_server_exists(HAND_TRAJ_ACTION_SERVER_TOPIC):  # Check if exists

            # Connect to robot control action server
            self._hand_joint_traj_client = actionlib.SimpleActionClient(
                HAND_TRAJ_ACTION_SERVER_TOPIC, control_msgs.FollowJointTrajectoryAction
            )

            # Waits until the action server has started up
            retval = self._hand_joint_traj_client.wait_for_server(
                timeout=rospy.Duration(secs=5)
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot."
                    % (HAND_TRAJ_ACTION_SERVER_TOPIC)
                )
                self._hand_joint_traj_connected = False
            else:
                self._hand_joint_traj_connected = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot." % (HAND_TRAJ_ACTION_SERVER_TOPIC)
            )
            self._hand_joint_traj_connected = False

        # Setup a new Panda joint trajectory action server
        rospy.loginfo(
            "Creating '%s' joint trajectory action services." % rospy.get_name()
        )
        rospy.logdebug(
            "Creating '%s/follow_joint_trajectory' service." % rospy.get_name()
        )
        self._joint_traj_as = actionlib.SimpleActionServer(
            "%s/follow_joint_trajectory" % rospy.get_name()[1:],
            FollowJointTrajectoryAction,
            execute_cb=self._joint_traj_execute_cb,
            auto_start=False,
        )
        self._joint_traj_as.register_preempt_callback(self._joint_traj_preempt_cb)
        self._joint_traj_as.start()

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

        # Setup a new Panda hand joint trajectory action server
        rospy.logdebug(
            "Creating '%s/panda_hand/follow_joint_trajectory' service."
            % rospy.get_name()
        )
        self._hand_joint_traj_as = actionlib.SimpleActionServer(
            "%s/panda_hand/follow_joint_trajectory" % rospy.get_name()[1:],
            FollowJointTrajectoryAction,
            execute_cb=self._hand_joint_traj_execute_cb,
            auto_start=False,
        )
        self._hand_joint_traj_as.register_preempt_callback(
            self._hand_joint_traj_preempt_cb
        )
        self._hand_joint_traj_as.start()
        rospy.loginfo(
            "'%s' joint trajectory action services created successfully."
            % rospy.get_name()
        )

        # Initialize joint trajectory feedback messages
        self._init_action_servers_fb_msgs()

    ###############################################
    # Panda control member functions ##############
    ###############################################
    def _init_action_servers_fb_msgs(self):
        """Initiate the ``panda_control_server/panda_arm/follow_joint_trajectory`` and
        ``panda_control_server/panda_hand/follow_joint_trajectory`` feedback messages
        with the current robot states.
        """

        # Create header
        header = Header()
        header.stamp = rospy.get_rostime()

        # Get current robot state ditionary
        state_dict = self._retrieve_state_dict(self._controlled_joints_traj_control)
        arm_state_dict = state_dict["arm"]
        hand_state_dict = state_dict["hand"]

        # Fill arm feedback message
        self._as_arm_feedback.joint_names = self._controlled_joints_traj_control["arm"]
        self._as_arm_feedback.header = header
        self._as_arm_feedback.actual.positions = arm_state_dict["positions"].values()
        self._as_arm_feedback.actual.velocities = arm_state_dict["velocities"].values()
        self._as_arm_feedback.actual.effort = arm_state_dict["efforts"].values()

        # Fill hand feedback message
        self._as_hand_feedback.joint_names = self._controlled_joints_traj_control[
            "hand"
        ]
        self._as_hand_feedback.header = header
        self._as_hand_feedback.actual.positions = hand_state_dict["positions"].values()
        self._as_hand_feedback.actual.velocities = hand_state_dict[
            "velocities"
        ].values()
        self._as_hand_feedback.actual.effort = hand_state_dict["efforts"].values()
        self._as_feedback.joint_names = self._controlled_joints_traj_control["both"]
        self._as_hand_feedback.header = header

        # Fill combined feedback message
        self._as_feedback.actual.positions = list(self.joint_states.position)
        self._as_feedback.actual.velocities = list(self.joint_states.velocity)
        self._as_feedback.actual.effort = list(self.joint_states.effort)

    def _get_combined_action_server_fb_msg(self):
        """Combine the action server feedback messages from the original
        ``panda_arm_controller/follow_joint_trajectory`` and
        ``panda_hand_controller/follow_joint_trajectory`` action servers so that they
        can be used as a feedback message to the
        ``panda_control_server/follow_joint_trajectory`` wrapper action server.

        Returns
        -------
        panda_openai_sim.msg.FollowJointTrajectoryFeedback
            New combined action server feedback message.
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

        Parameters
        ----------
        controlled_joints_dict : dict
            Dictionary containing the joints that are currently being controlled.

        Returns
        -------
        dict
            A dictionary containing the current Panda arm and hand positions, velocities
            and efforts.
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

        # Return and and arm position, velocity and effort states dicts as a dictionary
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

    def _wait_till_done(
        self, control_type, control_group="both", timeout=None, controlled_joints=None
    ):
        """Wait control is finished. Meaning the robot state is within range of the
        Joint position and joint effort setpoints.

        Parameters
        ----------
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are ``joint_effort_control`` and ``joint_position_control``.
        control_group : str, optional
            The control group  for which the control is being performed, defaults to
            "both".
        controlled_joints : dict, optional
            A dictionary containing the joints that are currently being controlled,
            these joints will be determined if no dictionary is given.
        connection_timeout : int, optional
            The timeout when waiting for the control to be done, by default
            self._wait_till_done_timeout.
        """

        # Validate control type and control group
        if control_type not in ["joint_position_control", "joint_effort_control"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['joint_position_control', 'joint_effort_control']")
            )
            return False
        else:

            # De-capitalize control type
            control_type = control_type.lower()
        if control_group not in ["arm", "hand", "both"]:
            rospy.logwarn(
                "The control group '%s' you specified is not valid. Valid values are "
                "%s. Control group 'both' used instead." % ("['arm', 'hand', 'both']")
            )
            control_group = "both"
        else:

            # De-capitalize control group
            control_group = control_group.lower()

        # Check if the controlled joints dictionary was given and compute the
        # state masks
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
        if timeout:  # If not supplied
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
            if control_type == "joint_position_control":

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
            elif control_type == "joint_effort_control":

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

        # Return value
        return True

    def _create_traj_action_server_msg(self, input_msg, control_group, verbose=False):
        """Converts the ``panda_openai_sim.msg.FollowJointTrajectoryGoal`` message that
        is received by the ``panda_control_server`` follow joint trajectory wrapper
        action servers into the right format for the original ``panda_arm_controller``
        and ``panda_hand_controller``
        `follow_joint_trajectory <https://wiki.ros.org/joint_trajectory_action/>`_
        action servers.

        Parameters
        ----------
        input_msg :
            The service input message we want to validate.
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are ``joint_effort_control`` and ``joint_position_control``.
        verbose : bool
            Boolean specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        dict
            A dictionary containing the arm and hand panda_arm_controller
            :control_msgs:`control_msgs.msg.FollowJointTrajectoryGoal
            <html/action/FollowJointTrajectory.html>` messages.

        Raises
        ----------
        panda_openai_sim.exceptions.InputMessageInvalidError
            Raised when the input_msg could not be converted into panda_arm_controller
            control messages.
        """

        # Validate control_group argument
        if control_group.lower() not in ["arm", "hand", "both"]:

            # Log message and return result
            logwarn_message = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_group '%s' invalid." % control_group.lower(),
                log_message=logwarn_message,
            )
        else:

            # De-capitalize control_group input argument
            control_group = control_group.lower()

        # Retrieve controlled joints
        try:
            controlled_joints_dict = self._get_controlled_joints(
                control_type="traj_control", verbose=verbose
            )
        except InputMessageInvalidError:

            # Throw error if controlled joints could not be retrieved
            logwarn_message = (
                "The joint trajectory publisher message could not be created as the "
                "'%s' are not initialized." % (self._traj_controllers)
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Required controllers not initialised.",
                details={"controlled_joints": self._effort_controllers},
                log_message=logwarn_message,
            )

        # Get needed contolled joints information out of dictionary
        if control_group == "arm":
            controlled_joints = controlled_joints_dict["arm"]
        elif control_group == "hand":
            controlled_joints = controlled_joints_dict["hand"]
        else:
            controlled_joints = controlled_joints_dict["both"]
        controlled_joints_size = len(controlled_joints)

        # Retrieve the current robot states
        state_dict = self._retrieve_state_dict(controlled_joints_dict)
        arm_state_dict = state_dict["arm"]
        hand_state_dict = state_dict["hand"]

        # Retrieve joint names from the input message
        joint_names = input_msg.trajectory.joint_names

        # Initiate new arm and hand action server messages using the current robot state
        arm_control_msg = copy.deepcopy(input_msg)
        arm_control_msg.trajectory.joint_names = controlled_joints_dict["arm"]
        for idx, waypoint in enumerate(input_msg.trajectory.points):

            # Add positions
            arm_control_msg.trajectory.points[idx].positions = arm_state_dict[
                "positions"
            ].values()

            # Add velocities
            arm_control_msg.trajectory.points[idx].velocities = arm_state_dict[
                "velocities"
            ].values()

            # Add efforts
            arm_control_msg.trajectory.points[idx].effort = arm_state_dict[
                "efforts"
            ].values()

            # Add accelerations
            arm_control_msg.trajectory.points[idx].accelerations = [0.0] * len(
                arm_control_msg.trajectory.joint_names
            )
        hand_control_msg = copy.deepcopy(input_msg)
        hand_control_msg.trajectory.joint_names = controlled_joints_dict["hand"]
        for idx, waypoint in enumerate(input_msg.trajectory.points):

            # Add positions
            hand_control_msg.trajectory.points[idx].positions = hand_state_dict[
                "positions"
            ].values()

            # Add velocities
            hand_control_msg.trajectory.points[idx].velocities = hand_state_dict[
                "velocities"
            ].values()

            # Add efforts
            hand_control_msg.trajectory.points[idx].effort = hand_state_dict[
                "efforts"
            ].values()

            # Add accelerations
            hand_control_msg.trajectory.points[idx].accelerations = [0.0] * len(
                hand_control_msg.trajectory.joint_names
            )

        # Retrieve the length of the positions/velocities/accelerations and efforts in
        # all the trajectory waypoints
        position_waypoints_length = [
            len(waypoint.positions) for waypoint in input_msg.trajectory.points
        ]
        velocity_waypoints_length = [
            len(waypoint.velocities) for waypoint in input_msg.trajectory.points
        ]
        acceleration_waypoints_length = [
            len(waypoint.accelerations) for waypoint in input_msg.trajectory.points
        ]
        effort_waypoints_length = [
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
                        for length in position_waypoints_length
                    ]
                ),
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in velocity_waypoints_length
                    ]
                ),
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in acceleration_waypoints_length
                    ]
                ),
                any(
                    [
                        length != controlled_joints_size and length != 0
                        for length in effort_waypoints_length
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
                            for length in position_waypoints_length
                        ]
                    ),
                    "velocities": any(
                        [
                            length > controlled_joints_size
                            for length in velocity_waypoints_length
                        ]
                    ),
                    "accelerations": any(
                        [
                            length > controlled_joints_size
                            for length in acceleration_waypoints_length
                        ]
                    ),
                    "joint_efforts": any(
                        [
                            length > controlled_joints_size
                            for length in effort_waypoints_length
                        ]
                    ),
                }

                # Throw error if aforementioned is the case
                if any(waypoints_length_to_big.values()):

                    # Create log warning message
                    invalid_fields_string = list_2_human_text(
                        [key for key, val in waypoints_length_to_big.items() if val]
                    )
                    logwarn_message = (
                        "Your joint trajectory goal message contains more joint %s "
                        "than the %s joints that are found in the %s."
                        % (
                            invalid_fields_string,
                            controlled_joints_size,
                            (
                                "arm and hand control groups"
                                if control_group == "both"
                                else control_group + "control group"
                            ),
                        )
                    )

                    # Send log warn message and raise InputMessageInvalidError exception
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
                        for length in position_waypoints_length
                    ]
                ):  # Show warning if less control commands are given than joints

                    # Log warning messages
                    logwarn_message = (
                        "Some of the trajectory waypoints contain less position "
                        "commands than %s joints that are found in the %s. When "
                        "this is the case the current joint states will be used for "
                        "the joints without a position command."
                        % (
                            controlled_joints_size,
                            (
                                "arm and hand control groups"
                                if control_group == "both"
                                else control_group + "control group"
                            ),
                        )
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

                # Add position/velocity/acceleration and effort control commmands
                if control_group == "arm":

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

                elif control_group == "hand":

                    # Add time_from_start variable if create_time_axis == TRUE
                    if input_msg.create_time_axis:
                        time_axis_tmp = rospy.Duration.from_sec(
                            (idx * input_msg.time_axis_step) + input_msg.time_axis_step
                        )
                        hand_control_msg.trajectory.points[
                            idx
                        ].time_from_start = time_axis_tmp

                    # Add joint position commands
                    if len(waypoint.positions) != 0:
                        hand_control_msg.trajectory.points[idx].positions[
                            : len(waypoint.positions)
                        ] = list(waypoint.positions)
                    else:
                        # Make sure empty position field stays empty
                        if not self.autofill_traj_positions:
                            hand_control_msg.trajectory.points[idx].positions = []

                    # Add joint velocity commands
                    if len(waypoint.velocities) != 0:
                        hand_control_msg.trajectory.points[idx].velocities[
                            : len(waypoint.velocities)
                        ] = list(waypoint.velocities)
                    else:
                        # Make sure empty velocity field stays empty
                        hand_control_msg.trajectory.points[idx].velocities = []

                    # Add joint acceleration commands
                    if len(waypoint.accelerations) != 0:
                        hand_control_msg.trajectory.points[idx].accelerations[
                            : len(waypoint.accelerations)
                        ] = list(waypoint.accelerations)
                    else:
                        # Make sure empty acceleration field stays empty
                        hand_control_msg.trajectory.points[idx].accelerations = []

                    # Add joint effort commands
                    if len(waypoint.effort) != 0:
                        hand_control_msg.trajectory.points[idx].effort[
                            : len(waypoint.effort)
                        ] = list(waypoint.effort)
                    else:
                        # Make sure empty effort field stays empty
                        hand_control_msg.trajectory.points[idx].effort = []
                else:

                    # Add time_from_start variable if create_time_axis == TRUE
                    if input_msg.create_time_axis:
                        time_axis_tmp = rospy.Duration.from_sec(
                            (idx * input_msg.time_axis_step) + input_msg.time_axis_step
                        )
                        arm_control_msg.trajectory.points[
                            idx
                        ].time_from_start = time_axis_tmp

                    # Add time_from_start variable if create_time_axis == TRUE
                    if input_msg.create_time_axis:
                        time_axis_tmp = rospy.Duration.from_sec(
                            (idx * input_msg.time_axis_step) + input_msg.time_axis_step
                        )
                        hand_control_msg.trajectory.points[
                            idx
                        ].time_from_start = time_axis_tmp

                    # Check joint state order (hand,arm) or (arm, hand) and add control
                    # commands accordingly
                    if self.joint_states.name[0] in controlled_joints_dict["arm"]:

                        # Add joint position commands
                        if len(waypoint.positions) != 0:
                            if len(waypoint.positions) <= len(
                                arm_control_msg.trajectory.joint_names
                            ):  # If less commands than there are arm joints
                                arm_control_msg.trajectory.points[idx].positions[
                                    : len(waypoint.positions)
                                ] = list(waypoint.positions)
                            else:
                                arm_control_msg.trajectory.points[idx].positions[
                                    :
                                ] = waypoint.positions[
                                    : len(arm_control_msg.trajectory.joint_names)
                                ]
                                hand_control_msg.trajectory.points[idx].positions[
                                    : (
                                        len(waypoint.positions)
                                        - len(arm_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.positions[
                                    len(arm_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty position field stays empty
                            if not self.autofill_traj_positions:
                                arm_control_msg.trajectory.points[idx].positions = []
                                hand_control_msg.trajectory.points[idx].positions = []

                        # Add joint velocity commands
                        if len(waypoint.velocities) != 0:
                            if len(waypoint.velocities) <= len(
                                arm_control_msg.trajectory.joint_names
                            ):  # If less commands than there are arm joints
                                arm_control_msg.trajectory.points[idx].velocities[
                                    : len(waypoint.velocities)
                                ] = list(waypoint.velocities)
                            else:
                                arm_control_msg.trajectory.points[idx].velocities[
                                    :
                                ] = waypoint.velocities[
                                    : len(arm_control_msg.trajectory.joint_names)
                                ]
                                hand_control_msg.trajectory.points[idx].velocities[
                                    : (
                                        len(waypoint.velocities)
                                        - len(arm_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.velocities[
                                    len(arm_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty velocity field stays empty
                            arm_control_msg.trajectory.points[idx].velocities = []
                            hand_control_msg.trajectory.points[idx].velocities = []

                        # Add joint acceleration commands
                        if len(waypoint.accelerations) != 0:
                            if len(waypoint.accelerations) <= len(
                                arm_control_msg.trajectory.joint_names
                            ):  # If less commands than there are arm joints
                                arm_control_msg.trajectory.points[idx].accelerations[
                                    : len(waypoint.accelerations)
                                ] = list(waypoint.accelerations)
                            else:
                                arm_control_msg.trajectory.points[idx].accelerations[
                                    :
                                ] = waypoint.accelerations[
                                    : len(arm_control_msg.trajectory.joint_names)
                                ]
                                hand_control_msg.trajectory.points[idx].accelerations[
                                    : (
                                        len(waypoint.accelerations)
                                        - len(arm_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.accelerations[
                                    len(arm_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty acceleration field stays empty
                            arm_control_msg.trajectory.points[idx].accelerations = []
                            hand_control_msg.trajectory.points[idx].accelerations = []

                        # Add joint effort commands
                        if len(waypoint.effort) != 0:
                            if len(waypoint.effort) <= len(
                                arm_control_msg.trajectory.joint_names
                            ):  # If less commands than there are arm joints
                                arm_control_msg.trajectory.points[idx].effort[
                                    : len(waypoint.effort)
                                ] = list(waypoint.effort)
                            else:
                                arm_control_msg.trajectory.points[idx].effort[
                                    :
                                ] = waypoint.effort[
                                    : len(arm_control_msg.trajectory.joint_names)
                                ]
                                hand_control_msg.trajectory.points[idx].effort[
                                    : (
                                        len(waypoint.effort)
                                        - len(arm_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.effort[
                                    len(arm_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty effort field stays empty
                            arm_control_msg.trajectory.points[idx].effort = []
                            hand_control_msg.trajectory.points[idx].effort = []
                    else:

                        # Add joint position commands
                        if len(waypoint.positions) != 0:
                            if len(waypoint.positions) <= len(
                                hand_control_msg.trajectory.joint_names
                            ):  # If less commands than there are hand joints
                                hand_control_msg.trajectory.points[idx].positions[
                                    : len(waypoint.positions)
                                ] = list(waypoint.positions)
                            else:
                                hand_control_msg.trajectory.points[idx].positions[
                                    :
                                ] = waypoint.positions[
                                    : len(hand_control_msg.trajectory.joint_names)
                                ]
                                arm_control_msg.trajectory.points[idx].positions[
                                    : (
                                        len(waypoint.positions)
                                        - len(hand_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.positions[
                                    len(hand_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty position field stays empty
                            if not self.autofill_traj_positions:
                                arm_control_msg.trajectory.points[idx].positions = []
                                hand_control_msg.trajectory.points[idx].positions = []

                        # Add joint velocity commands
                        if len(waypoint.velocities) != 0:
                            if len(waypoint.velocities) <= len(
                                hand_control_msg.trajectory.joint_names
                            ):  # If less commands than there are hand joints
                                hand_control_msg.trajectory.points[idx].velocities[
                                    : len(waypoint.velocities)
                                ] = list(waypoint.velocities)
                            else:
                                hand_control_msg.trajectory.points[idx].velocities[
                                    :
                                ] = waypoint.velocities[
                                    : len(hand_control_msg.trajectory.joint_names)
                                ]
                                arm_control_msg.trajectory.points[idx].velocities[
                                    : (
                                        len(waypoint.velocities)
                                        - len(hand_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.velocities[
                                    len(hand_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty velocity field stays empty
                            arm_control_msg.trajectory.points[idx].velocities = []
                            hand_control_msg.trajectory.points[idx].velocities = []

                        # Add joint acceleration commands
                        if len(waypoint.accelerations) != 0:
                            if len(waypoint.accelerations) <= len(
                                hand_control_msg.trajectory.joint_names
                            ):  # If less commands than there are hand joints
                                hand_control_msg.trajectory.points[idx].accelerations[
                                    : len(waypoint.accelerations)
                                ] = list(waypoint.accelerations)
                            else:
                                hand_control_msg.trajectory.points[idx].accelerations[
                                    :
                                ] = waypoint.accelerations[
                                    : len(hand_control_msg.trajectory.joint_names)
                                ]
                                arm_control_msg.trajectory.points[idx].accelerations[
                                    : (
                                        len(waypoint.accelerations)
                                        - len(hand_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.accelerations[
                                    len(hand_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty acceleration field stays empty
                            arm_control_msg.trajectory.points[idx].accelerations = []
                            hand_control_msg.trajectory.points[idx].accelerations = []

                        # Add joint effort commands
                        if len(waypoint.effort) != 0:
                            if len(waypoint.effort) <= len(
                                hand_control_msg.trajectory.joint_names
                            ):  # If less commands than there are hand joints
                                hand_control_msg.trajectory.points[idx].effort[
                                    : len(waypoint.effort)
                                ] = list(waypoint.effort)
                            else:
                                hand_control_msg.trajectory.points[idx].effort[
                                    :
                                ] = waypoint.effort[
                                    : len(hand_control_msg.trajectory.joint_names)
                                ]
                                arm_control_msg.trajectory.points[idx].effort[
                                    : (
                                        len(waypoint.effort)
                                        - len(hand_control_msg.trajectory.joint_names)
                                    )
                                ] = waypoint.effort[
                                    len(hand_control_msg.trajectory.joint_names) :
                                ]
                        else:
                            # Make sure empty effort field stays empty
                            arm_control_msg.trajectory.points[idx].effort = []
                            hand_control_msg.trajectory.points[idx].effort = []

            # Return new action server messages
            return {
                "arm": panda_action_msg_2_control_msgs_action_msg(arm_control_msg),
                "hand": panda_action_msg_2_control_msgs_action_msg(hand_control_msg),
            }

        else:  # If joints were specified

            # Check if the number of joint positions/velocities/accelerations or
            # efforts is unequal to the the number of joints
            waypoints_length_not_equal = {
                "positions": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in position_waypoints_length
                    ]
                ),
                "velocities": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in velocity_waypoints_length
                    ]
                ),
                "accelerations": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in acceleration_waypoints_length
                    ]
                ),
                "efforts": any(
                    [
                        length != len(joint_names) and length != 0
                        for length in effort_waypoints_length
                    ]
                ),
            }

            # Check if enough control values were given and throw error if needed
            # the case
            if any(waypoints_length_not_equal.values()):

                # Create log warning message
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

                # Send log warn message and raise InputMessageInvalidError exception
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

                    # Create joint names invalid warning message
                    logwarn_msg_strings = [
                        "Joint" if len(invalid_joint_names) == 1 else "Joints",
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "panda_openai_sim/SetJointPositions",
                    ]
                    logwarn_message = (
                        "%s that %s specified in the 'joint_names' field of the '%s' "
                        "message %s invalid. Valid joint names for controlling the "
                        "Panda %s are %s."
                        % (
                            "%s %s" % (logwarn_msg_strings[0], invalid_joint_names),
                            logwarn_msg_strings[1],
                            logwarn_msg_strings[2],
                            logwarn_msg_strings[1],
                            control_group
                            if control_group in ["hand", "arm"]
                            else "arm & hand",
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
                        if control_group == "arm":

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
                                arm_position_commands = (
                                    arm_position_commands_dict.values()
                                )
                                hand_position_commands = hand_state_dict[
                                    "positions"
                                ].values()
                            else:
                                # Make sure empty position field stays empty
                                if not self.autofill_traj_positions:
                                    arm_position_commands = []
                                    hand_position_commands = []

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
                                arm_velocity_commands = (
                                    arm_velocity_commands_dict.values()
                                )
                                hand_velocity_commands = hand_state_dict[
                                    "velocities"
                                ].values()
                            else:
                                # Make sure empty velocity field stays empty
                                arm_velocity_commands = []
                                hand_velocity_commands = []

                            # Create acceleration command array
                            if len(waypoint.accelerations) != 0:
                                arm_acceleration_commands_dict = OrderedDict(
                                    zip(
                                        arm_state_dict["velocities"].keys(),
                                        [0.0]
                                        * len(arm_state_dict["velocities"].keys()),
                                    )
                                )  # Initiate at 0 as accelerations are unknown
                                for (joint, acceleration,) in input_command_dict[
                                    "accelerations"
                                ].items():  # Add control commands
                                    if joint in arm_state_dict["velocities"]:
                                        arm_acceleration_commands_dict[
                                            joint
                                        ] = acceleration
                                arm_acceleration_commands = (
                                    arm_acceleration_commands_dict.values()
                                )
                                hand_acceleration_commands = [0.0] * len(
                                    hand_control_msg.trajectory.joint_names
                                )  # Initiate at 0 as accelerations are unknown
                            else:
                                # Make sure empty acceleration field stays empty
                                arm_acceleration_commands = []
                                hand_acceleration_commands = []

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
                                hand_effort_commands = hand_state_dict[
                                    "efforts"
                                ].values()
                            else:
                                # Make sure empty effort field stays empty
                                arm_effort_commands = []
                                hand_effort_commands = []

                        elif control_group == "hand":

                            # Add time_from_start variable if create_time_axis == TRUE
                            if input_msg.create_time_axis:
                                time_axis_tmp = rospy.Duration.from_sec(
                                    (idx * input_msg.time_axis_step)
                                    + input_msg.time_axis_step
                                )
                                hand_control_msg.trajectory.points[
                                    idx
                                ].time_from_start = time_axis_tmp

                            # Create position command array
                            if len(waypoint.positions) != 0:
                                hand_position_commands_dict = copy.deepcopy(
                                    hand_state_dict["positions"]
                                )  # Start from the current states
                                for (joint, position,) in input_command_dict[
                                    "positions"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["positions"]:
                                        hand_position_commands_dict[joint] = position
                                arm_position_commands = arm_state_dict[
                                    "positions"
                                ].values()
                                hand_position_commands = (
                                    hand_position_commands_dict.values()
                                )
                            else:
                                # Make sure empty position field stays empty
                                if not self.autofill_traj_positions:
                                    arm_position_commands = []
                                    hand_position_commands = []

                            # Create velocity command array
                            if len(waypoint.velocities) != 0:
                                hand_velocity_commands_dict = copy.deepcopy(
                                    hand_state_dict["velocities"]
                                )  # Start from the current states
                                for (joint, velocity,) in input_command_dict[
                                    "velocities"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["velocities"]:
                                        hand_velocity_commands_dict[joint] = velocity
                                arm_velocity_commands = arm_state_dict[
                                    "velocities"
                                ].values()
                                hand_velocity_commands = (
                                    hand_velocity_commands_dict.values()
                                )
                            else:
                                # Make sure empty velocity field stays empty
                                arm_velocity_commands = []
                                hand_velocity_commands = []

                            # Create acceleration command array
                            if len(waypoint.accelerations) != 0:
                                hand_acceleration_commands_dict = OrderedDict(
                                    zip(
                                        hand_state_dict["velocities"].keys(),
                                        [0.0]
                                        * len(hand_state_dict["velocities"].keys()),
                                    )
                                )  # Initiate at 0 as accelerations are unknown
                                for (joint, acceleration,) in input_command_dict[
                                    "accelerations"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["velocities"]:
                                        hand_acceleration_commands_dict[
                                            joint
                                        ] = acceleration
                                arm_acceleration_commands = [0.0] * len(
                                    arm_control_msg.trajectory.joint_names
                                )  # Initiate at 0 as accelerations are unknown
                                hand_acceleration_commands = (
                                    hand_acceleration_commands_dict.values()
                                )
                            else:
                                # Make sure empty acceleration field stays empty
                                arm_acceleration_commands = []
                                hand_acceleration_commands = []

                            # Create effort command array
                            if len(waypoint.effort) != 0:
                                hand_effort_commands_dict = copy.deepcopy(
                                    hand_state_dict["efforts"]
                                )  # Start from the current states
                                for (joint, effort,) in input_command_dict[
                                    "efforts"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["efforts"]:
                                        hand_effort_commands_dict[joint] = effort
                                arm_effort_commands = arm_state_dict["efforts"].values()
                                hand_effort_commands = (
                                    hand_effort_commands_dict.values()
                                )
                            else:
                                # Make sure empty effort field stays empty
                                arm_effort_commands = []
                                hand_effort_commands = []
                        else:

                            # Add time_from_start variable if create_time_axis == TRUE
                            if input_msg.create_time_axis:
                                time_axis_tmp = rospy.Duration.from_sec(
                                    (idx * input_msg.time_axis_step)
                                    + input_msg.time_axis_step
                                )
                                arm_control_msg.trajectory.points[
                                    idx
                                ].time_from_start = time_axis_tmp

                            # Add time_from_start variable if create_time_axis == TRUE
                            if input_msg.create_time_axis:
                                time_axis_tmp = rospy.Duration.from_sec(
                                    (idx * input_msg.time_axis_step)
                                    + input_msg.time_axis_step
                                )
                                hand_control_msg.trajectory.points[
                                    idx
                                ].time_from_start = time_axis_tmp

                            # Create arm position control message
                            if len(waypoint.positions) != 0:
                                arm_position_commands_dict = copy.deepcopy(
                                    arm_state_dict["positions"]
                                )  # Start from the current states
                                for (joint, position,) in input_command_dict[
                                    "positions"
                                ].items():  # Add control commands
                                    if joint in arm_state_dict["positions"]:
                                        arm_position_commands_dict[joint] = position

                                # Create hand position control message
                                hand_position_commands_dict = copy.deepcopy(
                                    hand_state_dict["positions"]
                                )  # Start from the current states
                                for (joint, position,) in input_command_dict[
                                    "positions"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["positions"]:
                                        hand_position_commands_dict[joint] = position

                                # Retrieve commands from dictionaries
                                arm_position_commands = (
                                    arm_position_commands_dict.values()
                                )
                                hand_position_commands = (
                                    hand_position_commands_dict.values()
                                )
                            else:
                                # Make sure empty position field stays empty
                                if not self.autofill_traj_positions:
                                    arm_position_commands = []
                                    hand_position_commands = []

                            # Create arm velocity control message
                            if len(waypoint.velocities) != 0:
                                arm_velocity_commands_dict = copy.deepcopy(
                                    arm_state_dict["velocities"]
                                )  # Start from the current states
                                for (joint, velocity,) in input_command_dict[
                                    "velocities"
                                ].items():  # Add control commands
                                    if joint in arm_state_dict["velocities"]:
                                        arm_velocity_commands_dict[joint] = velocity

                                # Create hand velocity control message
                                hand_velocity_commands_dict = copy.deepcopy(
                                    hand_state_dict["velocities"]
                                )  # Start from the current states
                                for (joint, velocity,) in input_command_dict[
                                    "velocities"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["velocities"]:
                                        hand_velocity_commands_dict[joint] = velocity

                                # Retrieve commands from dictionaries
                                arm_velocity_commands = (
                                    arm_velocity_commands_dict.values()
                                )
                                hand_velocity_commands = (
                                    hand_velocity_commands_dict.values()
                                )
                            else:
                                # Make sure empty velocity field stays empty
                                arm_velocity_commands = []
                                hand_velocity_commands = []

                            # Create arm acceleration control message
                            if len(waypoint.accelerations) != 0:
                                arm_acceleration_commands_dict = OrderedDict(
                                    zip(
                                        arm_state_dict["velocities"].keys(),
                                        [0.0]
                                        * len(arm_state_dict["velocities"].keys()),
                                    )
                                )  # Initiate at 0 as accelerations are unknown
                                for (joint, acceleration,) in input_command_dict[
                                    "accelerations"
                                ].items():  # Add control commands
                                    if joint in arm_state_dict["velocities"]:
                                        arm_acceleration_commands_dict[
                                            joint
                                        ] = acceleration

                                # Create hand acceleration control message
                                hand_acceleration_commands_dict = OrderedDict(
                                    zip(
                                        hand_state_dict["velocities"].keys(),
                                        [0.0]
                                        * len(hand_state_dict["velocities"].keys()),
                                    )
                                )  # Initiate at 0 as accelerations are unknown
                                for (joint, acceleration,) in input_command_dict[
                                    "accelerations"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["velocities"]:
                                        hand_acceleration_commands_dict[
                                            joint
                                        ] = acceleration

                                # Retrieve commands from dictionaries
                                arm_acceleration_commands = (
                                    arm_acceleration_commands_dict.values()
                                )
                                hand_acceleration_commands = (
                                    hand_acceleration_commands_dict.values()
                                )
                            else:
                                # Make sure empty acceleration field stays empty
                                arm_acceleration_commands = []
                                hand_acceleration_commands = []

                            # Create arm effort control message
                            if len(waypoint.effort) != 0:
                                arm_effort_commands_dict = copy.deepcopy(
                                    arm_state_dict["efforts"]
                                )  # Start from the current states
                                for (joint, effort,) in input_command_dict[
                                    "efforts"
                                ].items():  # Add control commands
                                    if joint in arm_state_dict["efforts"]:
                                        arm_effort_commands_dict[joint] = effort

                                # Create hand effort control message
                                hand_effort_commands_dict = copy.deepcopy(
                                    hand_state_dict["efforts"]
                                )  # Start from the current states
                                for (joint, effort,) in input_command_dict[
                                    "efforts"
                                ].items():  # Add control commands
                                    if joint in hand_state_dict["efforts"]:
                                        hand_effort_commands_dict[joint] = effort
                                arm_effort_commands = arm_effort_commands_dict.values()
                                hand_effort_commands = (
                                    hand_effort_commands_dict.values()
                                )
                            else:
                                # Make sure empty effort field stays empty
                                arm_effort_commands = []
                                hand_effort_commands = []

                        # Add new control commands to joint trajectory control message
                        # waypoint

                        # Positions
                        arm_control_msg.trajectory.points[
                            idx
                        ].positions = arm_position_commands
                        hand_control_msg.trajectory.points[
                            idx
                        ].positions = hand_position_commands

                        # Velocities
                        arm_control_msg.trajectory.points[
                            idx
                        ].velocities = arm_velocity_commands
                        hand_control_msg.trajectory.points[
                            idx
                        ].velocities = hand_velocity_commands

                        # Accelerations
                        arm_control_msg.trajectory.points[
                            idx
                        ].accelerations = arm_acceleration_commands
                        hand_control_msg.trajectory.points[
                            idx
                        ].accelerations = hand_acceleration_commands

                        # Effort
                        arm_control_msg.trajectory.points[
                            idx
                        ].effort = arm_effort_commands
                        hand_control_msg.trajectory.points[
                            idx
                        ].effort = hand_effort_commands

                    # Return new action server messages
                    return {
                        "arm": panda_action_msg_2_control_msgs_action_msg(
                            arm_control_msg
                        ),
                        "hand": panda_action_msg_2_control_msgs_action_msg(
                            hand_control_msg
                        ),
                    }

    def _create_control_publisher_msg(
        self, input_msg, control_type, control_group, verbose=False
    ):
        """Converts the service input message into a control commands that is used by
        the control publishers. While doing this it also verifies whether the given
        input message is valid.

        Parameters
        ----------
        input_msg :
            The service input message we want to validate.
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are ``joint_effort_control`` and ``joint_position_control``.
        control_group : str
            The robot control group which is being controlled. Options are ``arm``,
            ``hand`` or ``both``.
        verbose : bool
            Boolean specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        (dict, dict)
            Two dictionaries. The first dictionary contains the Panda arm and hand
            control commands in the order which is are required by the publishers.
            The second dictionary contains the joints that are being controlled.

        Raises
        ----------
        panda_openai_sim.exceptions.InputMessageInvalidError
            Raised when the input_msg could not be converted into ``moveit_commander``
            arm hand joint position/effort commands.
        """

        # Validate control_group argument
        if control_group.lower() not in ["arm", "hand", "both"]:

            # Log message and return result
            logwarn_message = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_group '%s' invalid." % control_group.lower(),
                log_message=logwarn_message,
            )
        else:

            # De-capitalize input arguments
            control_group = control_group.lower()
            control_type = control_type.lower()

        # Validate control_type argument and extract information from input message
        if control_type == "joint_position_control":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_positions
        elif control_type == "joint_effort_control":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_efforts
        else:

            # Log message and return result
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['joint_position_control', 'joint_effort_control']")
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

            # Throw error if controlled joints could not be retrieved
            logwarn_message = (
                "The '%s' publisher messages could not be created as the '%s' %s "
                "are not initialized."
                % (
                    "effort control"
                    if control_type == "joint_effort_control"
                    else "position control",
                    self._effort_controllers
                    if control_type == "joint_effort_control"
                    else self._position_controllers,
                    "joint effort controllers"
                    if control_type == "joint_effort_control"
                    else "joint position controllers",
                )
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Required controllers not initialised.",
                details={"controlled_joints": self._effort_controllers},
                log_message=logwarn_message,
            )

        # Get needed contolled joints information out of dictionary
        if control_group == "arm":
            controlled_joints = controlled_joints_dict["arm"]
        elif control_group == "hand":
            controlled_joints = controlled_joints_dict["hand"]
        else:
            controlled_joints = controlled_joints_dict["both"]
        controlled_joints_size = len(controlled_joints)

        # Retrieve the current robot states
        state_dict = self._retrieve_state_dict(controlled_joints_dict)

        # Get control publisher message type and current joint states
        if control_type == "joint_position_control":
            arm_state_dict = state_dict["arm"]["positions"]
            hand_state_dict = state_dict["hand"]["positions"]
            arm_msg_type = self._arm_position_controller_msg_type
            hand_msg_type = self._hand_position_controller_msg_type
        elif control_type == "joint_effort_control":
            arm_state_dict = state_dict["arm"]["efforts"]
            hand_state_dict = state_dict["hand"]["efforts"]
            arm_msg_type = self._arm_effort_controller_msg_type
            hand_msg_type = self._hand_effort_controller_msg_type

        # Check service request input
        if len(joint_names) == 0:  # If not joint_names were given

            # Check if enough joint position commands were given otherwise give warning
            if len(control_input) != controlled_joints_size:

                # Create log message strings
                if control_type == "joint_position_control":
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

                    # Log message and raise exception
                    logwarn_message = "You specified %s while the Panda %s %s." % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        control_group + " control group has"
                        if control_group in ["arm", "hand"]
                        else "arm and hand control groups have",
                        "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
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

                    # Display warning message
                    logwarn_message = "You specified %s while the Panda %s %s." % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        control_group + " control group has"
                        if control_group in ["arm", "hand"]
                        else "arm and hand control groups have",
                        "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
                    ) + " As a result only joints %s will be controlled." % (
                        controlled_joints[: len(control_input)]
                    )
                    rospy.logwarn(logwarn_message)

            # Update current state dictionary with given joint_position commands
            if control_group == "arm":
                arm_position_commands = copy.deepcopy(arm_state_dict.values())
                arm_position_commands[: len(control_input)] = control_input
                hand_position_commands = hand_state_dict.values()
            elif control_group == "hand":
                hand_position_commands = copy.deepcopy(hand_state_dict)
                arm_position_commands = arm_state_dict.values()
            else:
                arm_position_commands = copy.deepcopy(arm_state_dict.values())
                hand_position_commands = copy.deepcopy(hand_state_dict.values())
                if self.joint_states.name[0] in controlled_joints_dict["arm"]:
                    if len(control_input) <= len(arm_position_commands):
                        arm_position_commands[: len(control_input)] = control_input
                    else:
                        arm_position_commands[:] = control_input[
                            : len(arm_position_commands)
                        ]
                        hand_position_commands[
                            : len(control_input) - len(arm_position_commands)
                        ] = control_input[len(arm_position_commands) :]
                else:
                    if len(control_input) <= len(hand_position_commands):
                        hand_position_commands[: len(control_input)] = control_input
                    else:
                        hand_position_commands[:] = control_input[
                            : len(hand_position_commands)
                        ]
                        arm_position_commands[
                            : len(control_input) - len(hand_position_commands)
                        ] = control_input[len(hand_position_commands) :]

            # Create publishers command dictionary
            if not self.use_group_controller:  # Non group controller
                control_commands = {
                    "arm": [arm_msg_type(item) for item in arm_position_commands],
                    "hand": [hand_msg_type(item) for item in hand_position_commands],
                }
            else:
                control_commands = {
                    "arm": arm_msg_type(data=arm_position_commands),
                    "hand": hand_msg_type(data=hand_position_commands),
                }

            # Return control commands
            return control_commands, controlled_joints_dict
        else:

            # Check if enough control values were given
            if len(joint_names) != len(control_input):

                # Create log message
                if control_type == "joint_position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "panda_openai_sim/SetJointPositions",
                        "joint" if len(joint_names) == 1 else "joints",
                        "joint position",
                        "joint_positions",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "panda_openai_sim/SetJointEfforts",
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

                    # Create joint names invalid warn message
                    if control_type == "joint_position_control":
                        logwarn_msg_strings = [
                            "Joint" if len(invalid_joint_names) == 1 else "Joints",
                            "was" if len(invalid_joint_names) == 1 else "were",
                            "panda_openai_sim/SetJointPositions",
                        ]
                    else:
                        logwarn_msg_strings = [
                            "Joint" if len(invalid_joint_names) == 1 else "Joints",
                            "was" if len(invalid_joint_names) == 1 else "were",
                            "panda_openai_sim/SetJointEfforts",
                        ]
                    logwarn_message = (
                        "%s that %s specified in the 'joint_names' field of the '%s' "
                        "message %s invalid. Valid joint names for controlling the "
                        "Panda %s are %s."
                        % (
                            "%s %s" % (logwarn_msg_strings[0], invalid_joint_names),
                            logwarn_msg_strings[1],
                            logwarn_msg_strings[2],
                            logwarn_msg_strings[1],
                            control_group
                            if control_group in ["hand", "arm"]
                            else "arm & hand",
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
                    if control_group == "arm":
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
                        hand_position_commands = hand_state_dict.values()
                    elif control_group == "hand":
                        hand_position_commands_dict = copy.deepcopy(
                            hand_state_dict
                        )  # Start from the current state
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Add control commands
                            if joint in hand_state_dict:
                                hand_position_commands_dict[joint] = position
                        arm_position_commands = arm_state_dict.values()
                        hand_position_commands = hand_position_commands_dict.values()
                    else:

                        # Create arm control message
                        arm_position_commands_dict = copy.deepcopy(
                            arm_state_dict
                        )  # Start from the current state
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Add control commands
                            if joint in arm_state_dict:
                                arm_position_commands_dict[joint] = position

                        # Create hand control message
                        hand_position_commands_dict = copy.deepcopy(
                            hand_state_dict
                        )  # Start from the current state
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Add control commands
                            if joint in hand_state_dict:
                                hand_position_commands_dict[joint] = position
                        arm_position_commands = arm_position_commands_dict.values()
                        hand_position_commands = hand_position_commands_dict.values()

                    # Create publishers command dictionary
                    if not self.use_group_controller:  # Non group controller
                        control_commands = {
                            "arm": [
                                arm_msg_type(item) for item in arm_position_commands
                            ],
                            "hand": [
                                hand_msg_type(item) for item in hand_position_commands
                            ],
                        }
                    else:
                        control_commands = {
                            "arm": arm_msg_type(data=arm_position_commands),
                            "hand": hand_msg_type(data=hand_position_commands),
                        }

                    # Return control publishers commands dictionary
                    return control_commands, controlled_joints_dict

    def _get_controlled_joints(self, control_type, verbose=False):
        """Returns the joints that are controlled by a given control type.

        Parameters
        ----------
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are ``joint_effort_control`` and ``joint_position_control``.
        verbose : bool
            Boolean specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        dict
            A dictionary containing the joints that are controlled when using a given
            control type, grouped by control group (``arm`` and ``hand``).

        Raises
        ----------
        panda_openai_sim.exceptions.InputMessageInvalidError
            Raised when the input_msg could not be converted into ``moveit_commander``
            arm hand joint position commands.
        """

        # Remove capital letters from control_type
        control_type = control_type.lower()

        # Get the joints which are contolled by a given control type
        if control_type == "joint_position_control":

            # Get contolled joints information
            controlled_joints_dict = OrderedDict(
                zip(["arm", "hand", "both"], [[], [], []])
            )
            try:
                for position_controller in self._position_controllers:
                    for claimed_resources in self._controllers[
                        position_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if position_controller in self.arm_position_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif position_controller in self.hand_position_controllers:
                                controlled_joints_dict["hand"].append(resource)
            except KeyError:  # Controllers not initialized

                # Log message and return result
                logwarn_message = (
                    "Could not retrieve the controllable joints for 'joint_position' "
                    "control as the '%s' position controllers are not initialized. "
                    "Initialization of these controllers is needed to retrieve "
                    "information about thejoints they control."
                    % (self._position_controllers)
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalidError(
                    message="Required controllers not initialised.",
                    details={"controlled_joints": self._position_controllers},
                    log_message=logwarn_message,
                )
        elif control_type == "joint_effort_control":

            # Get controlled joints information
            controlled_joints_dict = OrderedDict(
                zip(["arm", "hand", "both"], [[], [], []])
            )
            try:
                for effort_controller in self._effort_controllers:
                    for claimed_resources in self._controllers[
                        effort_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if effort_controller in self.arm_effort_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif effort_controller in self.hand_effort_controllers:
                                controlled_joints_dict["hand"].append(resource)
            except KeyError:  # Controllers not initialized

                # Log message and return result
                logwarn_message = (
                    "Could not retrieve the controllable joints for 'joint_effort' "
                    "control as the '%s' effort controllers are not initialized. "
                    "Initialization of these controllers is needed to retrieve "
                    "information about thejoints they control."
                    % (self._effort_controllers)
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalidError(
                    message="Required controllers not initialised.",
                    details={"controlled_joints": self._effort_controllers},
                    log_message=logwarn_message,
                )
        elif control_type == "traj_control" or control_type == "ee_control":

            # Get controlled joints information
            controlled_joints_dict = OrderedDict(
                zip(["arm", "hand", "both"], [[], [], []])
            )
            try:
                for traj_controller in self._traj_controllers:
                    for claimed_resources in self._controllers[
                        traj_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if traj_controller in self.arm_traj_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif traj_controller in self.hand_traj_controllers:
                                controlled_joints_dict["hand"].append(resource)
            except KeyError:  # Controllers not initialized

                # Log message and return result
                logwarn_message = (
                    "Could not retrieve the controllable joints for 'joint_traj' "
                    "control as the '%s' joint trajectory controllers are not "
                    "initialized. Initialization of these controllers is needed to "
                    "retrieve information about thejoints they control."
                    % (self._effort_controllers)
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalidError(
                    message="Required controllers not initialised.",
                    details={"controlled_joints": self._effort_controllers},
                    log_message=logwarn_message,
                )
        else:

            # Log message and return result
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['joint_position_control', 'joint_effort_control', 'traj_control']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Fill controlled joints dict
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

        # Return controlled joints dict
        return controlled_joints_dict

    def _get_joint_controllers(self):
        """Retrieves the controllers which are currently initialized to work with a
        given joint.

        Returns
        -------
        dict
            Dictionary containing each panda joint and the controllers that are able to
            control these joints.
        """

        # Loop through active controllers
        joint_contollers_dict = {}
        for (key, val) in self._controllers.items():
            for resources_item in val.claimed_resources:
                for resource in resources_item.resources:
                    if resource in joint_contollers_dict.keys():
                        joint_contollers_dict[resource].append(key)
                    else:
                        joint_contollers_dict[resource] = [key]

        # Return controller dict
        return joint_contollers_dict

    ###############################################
    # Subscribers callback functions ##############
    ###############################################
    def _joints_cb(self, data):
        """Callback function for the joint data subscriber."""

        # Update joint_states
        self.joint_states = data

    ###############################################
    # Control services callback functions #########
    ###############################################
    def _set_joint_positions_cb(self, set_joint_positions_req):
        """Request arm and hand joint position control.

        Parameters
        ----------
        set_joint_positions_req : panda_openai_sim.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot arm and hand
            joints.

        Returns
        -------
        panda_openai_sim.srv.SetJointPositionsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:

            # Print warning
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

        # Create required variables and messages
        controllers_missing = {"arm": False, "hand": False}
        resp = SetJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = {"arm": [], "hand": []}
        missing_controllers = {"arm": [], "hand": []}
        for (group, position_controllers) in {
            "arm": self.arm_position_controllers,
            "hand": self.hand_position_controllers,
        }.items():
            for position_controller in position_controllers:
                try:
                    if self._controllers[position_controller].state != "running":
                        stopped_controllers[group].append(position_controller)
                except KeyError:
                    missing_controllers[group].append(position_controller)

        # Return failed result if we miss a controller
        if len(flatten_list(missing_controllers.values())) >= 1:
            rospy.logwarn(
                "Panda arm and hand joint position command could not be send as the %s "
                "%s not initialized. Please make sure you load the controller "
                "parameters onto the ROS parameter server."
                % (
                    flatten_list(missing_controllers.values()),
                    "joint position controller is"
                    if len(flatten_list(missing_controllers.values())) == 1
                    else "joint position controllers are",
                )
            )
            missing_controllers_group_string = (
                "Arm and hand"
                if all(
                    [values != [] for (group, values) in missing_controllers.items()]
                )
                else ("Arm" if "arm" in dict_clean(missing_controllers) else "Hand")
            )
            resp.success = False
            resp.message = (
                "%s controllers not initialised." % missing_controllers_group_string
            )
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="joint_position_control",
                control_group="both",
            )
        except InputMessageInvalidError as e:

            # Print warning message and return result
            logwarn_msg = "Panda arm joint positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(flatten_list(stopped_controllers.values())) >= 1:

            # Check from which group the controllers are missing
            controllers_missing = {
                group: (True if (position_controllers) else False)
                for (group, position_controllers) in stopped_controllers.items()
            }

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_positions_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers.values())
            else:
                for joint in set_joint_positions_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers.values()
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda %s joint positions command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        "arm and hand"
                        if all(controllers_missing.values())
                        else ("arm" if controllers_missing["arm"] else "hand"),
                        flatten_list(stopped_controllers.values()),
                        "joint position controller is"
                        if len(flatten_list(stopped_controllers.values())) == 1
                        else "joint position controllers are",
                    )
                )

        # Save position control setpoint
        if self.use_group_controller:
            joint_positions_setpoint_dict = {
                group: [command for command in control_list.data]
                for (group, control_list) in control_pub_msgs.items()
            }
        else:
            joint_positions_setpoint_dict = {
                group: [command.data for command in control_list]
                for (group, control_list) in control_pub_msgs.items()
            }
        self.joint_positions_setpoint = joint_positions_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm and hand joint positions control message.")
        self._arm_position_pub.publish(control_pub_msgs["arm"])
        self._hand_position_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait and not all(controllers_missing.values()):
            wait_control_group = (
                "both"
                if all([not bool_val for bool_val in controllers_missing.values()])
                else ("arm" if not controllers_missing["arm"] else "hand")
            )
            self._wait_till_done(
                control_type="joint_position_control",
                control_group=wait_control_group,
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _set_joint_efforts_cb(self, set_joint_efforts_req):
        """Request arm and hand joint effort control.

        Parameters
        ----------
        set_joint_efforts_req : panda_openai_sim.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot arm and hand
            joints.

        Returns
        -------
        panda_openai_sim.srv.SetJointEffortsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "efforts."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/set_joint_efforts" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = {"arm": False, "hand": False}
        resp = SetJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = {"arm": [], "hand": []}
        missing_controllers = {"arm": [], "hand": []}
        for (group, effort_controllers) in {
            "arm": self.arm_effort_controllers,
            "hand": self.hand_effort_controllers,
        }.items():
            for effort_controller in effort_controllers:
                try:
                    if self._controllers[effort_controller].state != "running":
                        stopped_controllers[group].append(effort_controller)
                except KeyError:
                    missing_controllers[group].append(effort_controller)

        # Return failed result if we miss a controller
        if len(flatten_list(missing_controllers.values())) >= 1:
            rospy.logwarn(
                "Panda arm and hand joint effort command could not be send as the %s "
                "%s not initialized. Please make sure you load the controller "
                "parameters onto the ROS parameter server."
                % (
                    flatten_list(missing_controllers.values()),
                    "joint effort controller is"
                    if len(flatten_list(missing_controllers.values())) == 1
                    else "joint effort controllers are",
                )
            )
            missing_controllers_group_string = (
                "Arm and hand"
                if all(
                    [values != [] for (group, values) in missing_controllers.items()]
                )
                else ("Arm" if "arm" in dict_clean(missing_controllers) else "Hand")
            )
            resp.success = False
            resp.message = (
                "%s controllers not initialised." % missing_controllers_group_string
            )
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="joint_effort_control",
                control_group="both",
            )
        except InputMessageInvalidError as e:

            # Print warning message and return result
            logwarn_msg = "Panda arm joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(flatten_list(stopped_controllers.values())) >= 1:

            # Check from which group the controllers are missing
            controllers_missing = {
                group: (True if (effort_controllers) else False)
                for (group, effort_controllers) in stopped_controllers.items()
            }

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_efforts_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers.values())
            else:
                for joint in set_joint_efforts_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers.values()
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda %s joint efforts command send but probably not executed as "
                    "the %s %s not running."
                    % (
                        "arm and hand"
                        if all(controllers_missing.values())
                        else ("arm" if controllers_missing["arm"] else "hand"),
                        req_missing_controllers,
                        "joint effort controller is"
                        if len(req_missing_controllers) == 1
                        else "joint effort controllers are",
                    )
                )

        # Save effort control setpoint
        if self.use_group_controller:
            joint_efforts_setpoint_dict = {
                group: [command for command in control_list.data]
                for (group, control_list) in control_pub_msgs.items()
            }
        else:
            joint_efforts_setpoint_dict = {
                group: [command.data for command in control_list]
                for (group, control_list) in control_pub_msgs.items()
            }
        self.joint_efforts_setpoint = joint_efforts_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm and hand joint efforts control message.")
        self._arm_effort_pub.publish(control_pub_msgs["arm"])
        self._hand_effort_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait and not all(controllers_missing.values()):
            wait_control_group = (
                "both"
                if all([not bool_val for bool_val in controllers_missing.values()])
                else ("arm" if not controllers_missing["arm"] else "hand")
            )
            self._wait_till_done(
                control_type="joint_effort_control",
                control_group=wait_control_group,
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_positions_cb(self, set_joint_positions_req):
        """Request arm joint position control.

        Parameters
        ----------
        set_joint_positions_req : panda_openai_sim.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_openai_sim.srv.SetJointPositionsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:

            # Print warning
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

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for position_controller in self.arm_position_controllers:
            try:
                if self._controllers[position_controller].state != "running":
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
                control_type="joint_position_control",
                control_group="arm",
            )
        except InputMessageInvalidError as e:

            # Print warning message and return result
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

            # Log warning
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
        if self.use_group_controller:
            joint_positions_setpoint_dict = {
                group: [command for command in control_list.data]
                for (group, control_list) in control_pub_msgs.items()
            }
        else:
            joint_positions_setpoint_dict = {
                group: [command.data for command in control_list]
                for (group, control_list) in control_pub_msgs.items()
            }
        self.joint_positions_setpoint = joint_positions_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm joint positions control message.")
        self._arm_position_pub.publish(control_pub_msgs["arm"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait and not controllers_missing:
            self._wait_till_done(
                control_type="joint_position_control",
                control_group="arm",
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_efforts_cb(self, set_joint_efforts_req):
        """Request arm Joint effort control.

        Parameters
        ----------
        set_joint_efforts_req : panda_openai_sim.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_openai_sim.srv.SetJointEffortsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:

            # Print warning
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

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for effort_controller in self.arm_effort_controllers:
            try:
                if self._controllers[effort_controller].state != "running":
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
                control_type="joint_effort_control",
                control_group="arm",
            )
        except InputMessageInvalidError as e:

            # Print warning message and return result
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

            # Log warning
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
        if self.use_group_controller:
            joint_efforts_setpoint_dict = {
                group: [command for command in control_list.data]
                for (group, control_list) in control_pub_msgs.items()
            }
        else:
            joint_efforts_setpoint_dict = {
                group: [command.data for command in control_list]
                for (group, control_list) in control_pub_msgs.items()
            }
        self.joint_efforts_setpoint = joint_efforts_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm joint efforts control message.")
        self._arm_effort_pub.publish(control_pub_msgs["arm"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait and not controllers_missing:
            self._wait_till_done(
                control_type="joint_effort_control",
                control_group="arm",
                controlled_joints=controlled_joints,
            )

        # Return service response
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_positions_cb(self, set_joint_positions_req):
        """Request hand joint position control

        Parameters
        ----------
        set_joint_positions_req : panda_openai_sim.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot hand joints.

        Returns
        -------
        panda_openai_sim.srv.SetJointPositionsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:

            # Print warning
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

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for position_controller in self.hand_position_controllers:
            try:
                if self._controllers[position_controller].state != "running":
                    stopped_controllers.append(position_controller)
            except KeyError:
                missing_controllers.append(position_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda hand joint position command could not be send as the %s %s "
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
            resp.message = "Hand controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="joint_position_control",
                control_group="hand",
            )
        except InputMessageInvalidError as e:

            # Print warning message and return result
            logwarn_msg = "Panda hand joint positions not set as " + lower_first_char(
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

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda hand joint positions command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint position controller is"
                        if len(stopped_controllers) == 1
                        else "joint position controllers are",
                    )
                )
        # Save position control setpoint
        if self.use_group_controller:
            joint_positions_setpoint_dict = {
                group: [command for command in control_list.data]
                for (group, control_list) in control_pub_msgs.items()
            }
        else:
            joint_positions_setpoint_dict = {
                group: [command.data for command in control_list]
                for (group, control_list) in control_pub_msgs.items()
            }
        self.joint_positions_setpoint = joint_positions_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda hand joint positions control message.")
        self._hand_position_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait and not controllers_missing:
            self._wait_till_done(
                control_type="joint_position_control",
                control_group="hand",
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_efforts_cb(self, set_joint_efforts_req):
        """Request hand joint effort control.

        Parameters
        ----------
        set_joint_efforts_req : panda_openai_sim.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot hand joints.

        Returns
        -------
        panda_openai_sim.srv.SetJointEffortsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "efforts."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_hand/set_joint_efforts" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for effort_controller in self.hand_effort_controllers:
            try:
                if self._controllers[effort_controller].state != "running":
                    stopped_controllers.append(effort_controller)
            except KeyError:
                missing_controllers.append(effort_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda hand joint efforts command could not be send as the %s %s "
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
            resp.message = "Hand controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="joint_effort_control",
                control_group="hand",
            )
        except InputMessageInvalidError as e:

            # Print warning message and return result
            logwarn_msg = "Panda hand joint efforts not set as " + lower_first_char(
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

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda hand joint efforts command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint effort controller is"
                        if len(stopped_controllers) == 1
                        else "joint effort controllers are",
                    )
                )

        # Save effort control setpoint
        if self.use_group_controller:
            joint_efforts_setpoint_dict = {
                group: [command for command in control_list.data]
                for (group, control_list) in control_pub_msgs.items()
            }
        else:
            joint_efforts_setpoint_dict = {
                group: [command.data for command in control_list]
                for (group, control_list) in control_pub_msgs.items()
            }
        self.joint_efforts_setpoint = joint_efforts_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda hand joint efforts control message.")
        self._hand_effort_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait and not controllers_missing:
            self._wait_till_done(
                control_type="joint_effort_control",
                control_group="hand",
                controlled_joints=controlled_joints,
            )

        # Return service response
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _switch_control_type_cb(self, switch_control_type_req):
        """Request the controller type to switch from individual joint control to joint
        group control.

        Parameters
        ----------
        switch_control_type_req : panda_openai_sim.srv.SwitchControlTypeRequest
            Service request message to switch the control_type.

        Returns
        -------
        panda_openai_sim.srv.SwitchControlTypeResponse
            Service response.
        """

        # Switch use group controller bool
        self.use_group_controller = not self.use_group_controller

        # Switch controller publishers
        if not self.use_group_controller:
            self._arm_position_pub = self._arm_joint_position_pub
            self._arm_effort_pub = self._arm_joint_effort_pub
            self._hand_position_pub = self._hand_joint_position_pub
            self._hand_effort_pub = self._hand_joint_effort_pub
            self.arm_position_controllers = ARM_POSITION_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_CONTROLLERS
            self._arm_position_controller_msg_type = Float64
            self._arm_effort_controller_msg_type = Float64
            self._hand_position_controller_msg_type = Float64
            self._hand_effort_controller_msg_type = Float64
        else:
            self._arm_position_pub = self._arm_joint_positions_group_pub
            self._arm_effort_pub = self._arm_joint_efforts_group_pub
            self._hand_position_pub = self._hand_joint_positions_group_pub
            self._hand_effort_pub = self._hand_joint_efforts_group_pub
            self.arm_position_controllers = ARM_POSITION_GROUP_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_GROUP_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_GROUP_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_GROUP_CONTROLLERS
            self._arm_position_controller_msg_type = Float64MultiArray
            self._arm_effort_controller_msg_type = Float64MultiArray
            self._hand_position_controller_msg_type = Float64MultiArray
            self._hand_effort_controller_msg_type = Float64MultiArray

        # Update combined position/effort controllers lists
        self._position_controllers = flatten_list(
            [self.arm_position_controllers, self.hand_position_controllers]
        )
        self._effort_controllers = flatten_list(
            [self.arm_effort_controllers, self.hand_effort_controllers]
        )

        # Return success bool
        resp = SwitchControlTypeResponse()
        resp.success = True
        return resp

    def _list_control_type_cb(self, list_control_type_req):
        """Returns the current controller type the 'panda_control_server' is using.

        Parameters
        ----------
        list_control_type_req : panda_openai_sim.srv.ListControlTypeRequest
            Service request message to list the control_type.

        Returns
        -------
        panda_openai_sim.srv.ListControlTypeResponse
            Service response. Options: ``joint_group_control`` and ``joint_control``.
        """

        # Check if verbose was set to True
        verbose = list_control_type_req.verbose

        # Create response message
        resp = ListControlTypeResponse()

        # Get current control type
        if self.use_group_controller:
            if verbose:
                rospy.loginfo(
                    "'%s' is currently using the group controllers to control "
                    "the Panda robot" % rospy.get_name()
                )
            resp.control_type = "joint_group_control"
        else:
            if verbose:
                rospy.loginfo(
                    "'%s' is currently using the individual joint controllers to "
                    "control the Panda robot" % rospy.get_name()
                )
            resp.control_type = "joint_control"

        # Return control type
        return resp

    def _get_controlled_joints_cb(self, get_controlled_joints_req):
        """Returns the joints that are controlled when using a given control type.

        Parameters
        ----------
        get_controlled_joints_req : panda_openai_sim.srv.GetControlledJointsRequest
            The service request message specifying the control_type.

        Returns
        -------
        panda_openai_sim.srv.GetControlledJointsResponse
            The response message that contains the ``controlled_joints`` list that
            specifies the joints that are controlled.
        """

        # Return controlled joints
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

    ###############################################
    # Action server callback functions ############
    ###############################################
    def _arm_joint_traj_execute_cb(self, goal):
        """Goal execution callback function for the Panda arm joint trajectory action
        server.

        Parameters
        ----------
        Goal : control_msgs.msg.FollowJointTrajectoryGoal
            Goal execution action server goal message.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(goal.trajectory.joint_names)
        if duplicate_list:

            # Print warning
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

        # Create error boolean
        error_occurred = False

        # Check the input goal message for errors and if valid convert it to
        # the right format for the original panda joint trajectory action servers.
        try:

            # Convert input goal message to the right format
            goal_msg_dict = self._create_traj_action_server_msg(
                goal, control_group="arm"
            )
        except InputMessageInvalidError as e:

            # Create abort return message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = (
                -2 if e.args[0] == "Invalid joint_names were given." else -6
            )
            self._result.error_string = e.log_message

            # Abort trajectory goal execution
            self._arm_joint_traj_as.set_aborted(result=self._result)

            # Set error bool
            error_occurred = True

        # Abort joint trajectory control if original panda arm control server is not
        # available
        if not self._arm_joint_traj_connected:

            # Create abort return message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = -7
            self._result.error_string = (
                "Could not connect to original Panda arm action server."
            )

            # Abort trajectory goal execution
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

            # Return result
            if not (
                self._arm_joint_traj_as.is_preempt_requested()
                or self._state.state == self._state.state_dict["PREEMPTED"]
            ):
                if self._result.error_code == self._result.SUCCESSFUL:  # If successful
                    self._arm_joint_traj_as.set_succeeded(self._result)
                else:  # If not successful
                    self._arm_joint_traj_as.set_aborted(self._result)

    def _hand_joint_traj_execute_cb(self, goal):
        """Goal execution callback function for the Panda hand joint trajectory action
        server.

        Parameters
        ----------
        Goal : control_msgs.msg.FollowJointTrajectoryGoal
            Goal execution action server goal message.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(goal.trajectory.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "trajectory."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_hand/follow_joint_trajectory" % rospy.get_name(),
                )
            )

        # Create error boolean
        error_occurred = False

        # Check the input goal message for errors and if valid convert it to
        # the right format for the original panda joint trajectory action servers.
        try:

            # Convert input goal message to the right format
            goal_msg_dict = self._create_traj_action_server_msg(
                goal, control_group="hand"
            )
        except InputMessageInvalidError as e:

            # Create abort return message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = (
                -2 if e.args[0] == "Invalid joint_names were given." else -6
            )
            self._result.error_string = e.log_message

            # Abort trajectory goal execution
            self._hand_joint_traj_as.set_aborted(result=self._result)

            # Set error bool
            error_occurred = True

        # Abort joint trajectory control if original panda hand control server is not
        # available
        if not self._hand_joint_traj_connected:

            # Create abort return message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = -7
            self._result.error_string = (
                "Could not connect to original Panda hand action server."
            )

            # Abort trajectory goal execution
            self._hand_joint_traj_as.set_aborted(result=self._result)

        # Execute joint trajectory goal
        if not error_occurred:

            # Send goal to the original Panda hand action servers
            self._hand_joint_traj_client.send_goal(
                goal_msg_dict["hand"], feedback_cb=self._hand_joint_traj_feedback_cb
            )

            # Wait for the server to finish performing the action
            self._hand_joint_traj_client.wait_for_result()

            # Get result from action server
            self._result = self._hand_joint_traj_client.get_result()
            self._state = ActionClientState(self._hand_joint_traj_client)
            self._result.error_string = translate_actionclient_result_error_code(
                self._result
            )

            # Return result
            if not (
                self._hand_joint_traj_as.is_preempt_requested()
                or self._state.state == self._state.state_dict["PREEMPTED"]
            ):
                if self._result.error_code == self._result.SUCCESSFUL:  # If successful
                    self._hand_joint_traj_as.set_succeeded(self._result)
                else:  # If not successful
                    self._hand_joint_traj_as.set_aborted(self._result)

    def _joint_traj_execute_cb(self, goal):
        """Goal execution callback function for the combined Panda arm and hand joint
        trajectory action server.

        Parameters
        ----------
        Goal : control_msgs.msg.FollowJointTrajectoryGoal
            Goal execution action server goal message.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(goal.trajectory.joint_names)
        if duplicate_list:

            # Print warning
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

        # Create error boolean
        error_occurred = False

        # Check the input goal message for errors and if valid convert it to
        # the right format for the original panda joint trajectory action servers.
        try:

            # Convert input goal message to the right format
            goal_msg_dict = self._create_traj_action_server_msg(
                goal, control_group="both"
            )
        except InputMessageInvalidError as e:

            # Create abort return message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = (
                -2 if e.args[0] == "Invalid joint_names were given." else -6
            )
            self._result.error_string = e.log_message

            # Abort trajectory goal execution
            self._joint_traj_as.set_aborted(result=self._result)

            # Set error bool
            error_occurred = True

        # Abort joint trajectory control if original panda control servers are not
        # available
        if any(
            [not self._arm_joint_traj_connected, not self._hand_joint_traj_connected]
        ):

            # Create abort return message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = -7
            self._result.error_string = (
                "Could not connect to original Panda arm and hand action servers."
            )

            # Abort trajectory goal execution
            self._joint_traj_as.set_aborted(result=self._result)

        # Execute joint trajectory goal
        if not error_occurred:

            # Send goal to the original Panda arm and hand action servers
            self._full_joint_traj_control = True
            self._arm_joint_traj_client.send_goal(
                goal_msg_dict["arm"], feedback_cb=self._arm_joint_traj_feedback_cb
            )
            self._hand_joint_traj_client.send_goal(
                goal_msg_dict["hand"], feedback_cb=self._hand_joint_traj_feedback_cb
            )

            # Wait for the server to finish performing the action
            self._arm_joint_traj_client.wait_for_result()
            self._hand_joint_traj_client.wait_for_result()
            self._full_joint_traj_control = False

            # Get result and state from action server
            self._arm_result = self._arm_joint_traj_client.get_result()
            self._hand_result = self._hand_joint_traj_client.get_result()
            self._arm_result.error_string = translate_actionclient_result_error_code(
                self._arm_result
            )
            self._hand_result.error_string = translate_actionclient_result_error_code(
                self._hand_result
            )
            self._arm_state = ActionClientState(self._arm_joint_traj_client)
            self._hand_state = ActionClientState(self._hand_joint_traj_client)

            # Create combined result message
            self._result = FollowJointTrajectoryResult()
            self._result.error_code = int(
                "-"
                + "".join(
                    [
                        str(-1 * self._arm_result.error_code),
                        str(-1 * self._hand_result.error_code),
                    ]
                )
            )
            self._result.error_string = (
                "Arm: "
                + "status="
                + self._arm_state.state_value_dict[self._arm_state.state]
                + ", result="
                + self._arm_result.error_string
                + " Hand: "
                + "status="
                + self._hand_state.state_value_dict[self._hand_state.state]
                + ", result="
                + self._hand_result.error_string
            )

            # Return result
            if not (
                any(
                    [
                        self._arm_joint_traj_as.is_preempt_requested(),
                        self._hand_joint_traj_as.is_preempt_requested(),
                    ]
                )
                or any(
                    [
                        self._arm_state.state
                        == self._arm_state.state_dict["PREEMPTED"],
                        self._hand_state.state
                        == self._hand_state.state_dict["PREEMPTED"],
                    ]
                )
            ):
                if (
                    self._arm_result == self._arm_result.SUCCESSFUL
                    and self._hand_result == self._hand_result.SUCCESSFUL
                ):  # If successful
                    self._joint_traj_as.set_succeeded(self._result)
                else:  # If not successful
                    self._joint_traj_as.set_aborted(self._result)

    def _arm_joint_traj_feedback_cb(self, feedback):
        """Relays the feedback messages from the original
        ``panda_arm_controller/follow_joint_trajectory`` server to our to our
        ``panda_control_server/panda_arm/follow_joint_trajectory`` wrapper action
        server.

        Parameters
        ----------
        feedback : control_msgs.msg.FollowJointTrajectoryFeedback
            Goal execution action server feedback message.
        """

        # Store feedback for combined action server
        self._as_arm_feedback = feedback

        # Check if callback was called from the combined action server
        if self._full_joint_traj_control:

            # Combine feedback with hand feedback
            combined_feedback = self._get_combined_action_server_fb_msg()

            # Publish feedback to the combined action server
            self._joint_traj_as.publish_feedback(combined_feedback)

        else:

            # Publish feedback to hand action server
            self._arm_joint_traj_as.publish_feedback(feedback)

    def _hand_joint_traj_feedback_cb(self, feedback):
        """Relays the feedback messages from the original
        ``panda_hand_controller/follow_joint_trajectory`` server to our to our
        ``panda_control_server/panda_hand/follow_joint_trajectory`` wrapper action
        server.

        Parameters
        ----------
        feedback : control_msgs.msg.FollowJointTrajectoryFeedback
            Goal execution action server feedback message.
        """

        # Store feedback for combined action server
        self._as_hand_feedback = feedback

        # Check if callback was called from the combined action server
        if self._full_joint_traj_control:

            # Combine feedback with arm feedback
            combined_feedback = self._get_combined_action_server_fb_msg()

            # Publish feedback to the combined action server
            self._joint_traj_as.publish_feedback(combined_feedback)

        else:

            # Publish feedback to hand action server
            self._hand_joint_traj_as.publish_feedback(feedback)

    def _arm_joint_traj_preempt_cb(self):
        """Relays the preempt request made to the
        ``panda_control_server/panda_arm/follow_joint_trajectory`` action server wrapper
        to the original ``panda_arm_controller/follow_joint_trajectory`` action server.
        """

        # Stop panda_arm_controller action server
        self._arm_joint_traj_client.cancel_goal()
        self._arm_joint_traj_as.set_preempted()
        self._full_joint_traj_control = False

    def _hand_joint_traj_preempt_cb(self):
        """Relays the preempt request made to the
        ``panda_control_server/panda_hand/follow_joint_trajectory`` action server
        wrapper to the original ``panda_hand_controller/follow_joint_trajectory``
        action server.
        """

        # Stop panda_hand_controller action server
        self._hand_joint_traj_client.cancel_goal()
        self._hand_joint_traj_as.set_preempted()
        self._full_joint_traj_control = False

    def _joint_traj_preempt_cb(self):
        """Relays the preempt request made to the
        ``panda_control_server/follow_joint_trajectory`` action server wrapper
        to the original ``panda_arm_controller/follow_joint_trajectory`` and
        ``panda_hand_controller/follow_joint_trajectory`` action servers.
        """

        # Stop panda_arm_controller and panda_hand_controller action servers
        self._arm_joint_traj_client.cancel_goal()
        self._hand_joint_traj_client.cancel_goal()
        self._joint_traj_as.set_preempted()
        self._full_joint_traj_control = False
