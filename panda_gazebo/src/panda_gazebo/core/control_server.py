#! /usr/bin/env python3
"""This server is responsible for controlling the Panda arm. It created a number of
(action) services that can be used to send control commands to the Panda Robot arm and
hand.

Main services:
    * get_controlled_joints
    * follow_joint_trajectory
    * set_joint_commands
    * panda_arm/follow_joint_trajectory
    * panda_hand/set_gripper_width

Extra services:
    * panda_arm/set_joint_positions
    * panda_arm/set_joint_efforts
"""

import copy
import os
import sys
from collections import OrderedDict
from itertools import compress

import actionlib
import control_msgs.msg as control_msgs
import numpy as np
import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from franka_msgs.msg import FrankaState
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
from rospy.exceptions import ROSException, ROSInterruptException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Global script variables
GRASP_EPSILON = 0.003  # NOTE: Uses 'kGraspRestingThreshold' from 'franka_gripper.sim.h'
GRASP_SPEED = 0.1  # NOTE: Uses 'kDefaultGripperActionSpeed' from 'franka_gripper.sim.h'
GRASP_FORCE = 10  # Panda force information: {Continious force: 70N, max_force: 140 N}
ACTION_TIMEOUT = 5  # How long we should wait for a action.
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
HAND_CONTROLLERS = ["franka_gripper"]


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
    """

    def __init__(  # noqa: C901
        self,
        autofill_traj_positions=False,
        connection_timeout=10,
        load_extra_services=False,
        brute_force_grasping=False,
    ):
        """Initializes the PandaControlServer object.

        Args:
            autofill_traj_positions (bool, optional): Whether you want to automatically
                set the current states as positions when the positions field of the
                joint trajectory message is left empty. Defaults to ``False``.
            connection_timeout (int, optional): The timeout for connecting to the
                controller_manager services. Defaults to 3 sec.
            load_extra_services (bool, optional): Whether to load extra services that
                are not used by the `openai_ros <https://wiki.ros.org/openai_ros>`_
                package. Defaults to ``False``.
            brute_force_grasping (bool, optional): Disable the gripper width reached
                check when grasping. Defaults to ``False``.
        """
        self.arm_joint_positions_setpoint = []
        self.arm_joint_efforts_setpoint = []
        self.gripper_width_setpoint = None
        self.arm_joint_positions_threshold = 0.01
        self.hand_joint_positions_threshold = 0.001
        self.arm_joint_efforts_threshold = 0.01
        self.autofill_traj_positions = autofill_traj_positions
        self._wait_till_arm_control_done_timeout = rospy.Duration(ACTION_TIMEOUT)
        self._arm_joint_positions_grad_threshold = 0.01
        self._hand_joint_positions_grad_threshold = 0.001
        self._arm_joint_efforts_grad_threshold = 0.01
        self._gripper_move_client_connected = False
        self._gripper_grasp_client_connected = False
        self._arm_joint_traj_client_connected = False

        # Disable the gripper width reached check used in the
        # franka_gripper/gripper_action` when the `max_effort` is bigger than 0.0.
        if brute_force_grasping:
            rospy.set_param("/franka_gripper/gripper_action/width_tolerance", 0.1)

        ########################################
        # Create Panda control services ########
        ########################################

        # Create main PandaControlServer services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/get_controlled_joints' service." % rospy.get_name()
        )
        self._get_controlled_joints_srv = rospy.Service(
            "%s/get_controlled_joints" % rospy.get_name().split("/")[-1],
            GetControlledJoints,
            self._get_controlled_joints_cb,
        )
        rospy.logdebug("Creating '%s/set_joint_commands' service." % rospy.get_name())
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
            self._list_controllers_client = rospy.ServiceProxy(
                "controller_manager/list_controllers", ListControllers
            )
            rospy.logdebug(
                "Connected to 'controller_manager/list_controllers' service!"
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logerr(
                "Shutting down '%s' because no connection could be established "
                "with the 'controller_manager/list_controllers' service."
                % (rospy.get_name(),)
            )
            sys.exit(0)

        # Connect to the gripper command action server
        franka_gripper_command_topic = "franka_gripper/gripper_action"
        rospy.logdebug(
            "Connecting to '%s' action service." % franka_gripper_command_topic
        )
        self._gripper_command_client_connected = False
        if action_server_exists(franka_gripper_command_topic):
            # Connect to robot control action server
            self._gripper_command_client = actionlib.SimpleActionClient(
                franka_gripper_command_topic,
                GripperCommandAction,
            )

            # Waits until the action server has started up
            retval = self._gripper_command_client.wait_for_server(
                timeout=rospy.Duration(secs=5)
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot."
                    % (franka_gripper_command_topic)
                )
            else:
                self._gripper_command_client_connected = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot." % (franka_gripper_command_topic)
            )

        # Connect to franka_state message
        self._franka_states = None
        while self._franka_states is None and not rospy.is_shutdown():
            try:
                self._franka_states = rospy.wait_for_message(
                    "franka_state_controller/franka_states", FrankaState, timeout=1.0
                )
            except ROSException:
                rospy.logwarn(
                    "Current franka_states not ready yet, retrying for getting %s"
                    % "franka_States"
                )
        self._franka_states_sub = rospy.Subscriber(
            "franka_state_controller/franka_states",
            FrankaState,
            self._franka_states_cb,
            queue_size=1,
        )

        ########################################
        # Connect joint state subscriber #######
        ########################################

        # Retrieve current robot joint state and effort information
        self.joint_states = None
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    "joint_states", JointState, timeout=1.0
                )
            except ROSException:
                rospy.logwarn(
                    "Current joint_states not ready yet, retrying for getting %s"
                    % "joint_states"
                )

        # Create joint_state subscriber
        self._joint_states_sub = rospy.Subscriber(
            "joint_states", JointState, self._joints_cb, queue_size=1
        )

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
            "Connecting to '%s' action service."
            % "panda_arm_controller/follow_joint_trajectory"
        )
        if action_server_exists("panda_arm_controller/follow_joint_trajectory"):
            # Connect to robot control action server
            self._arm_joint_traj_client = actionlib.SimpleActionClient(
                "panda_arm_controller/follow_joint_trajectory",
                control_msgs.FollowJointTrajectoryAction,
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
                    % ("panda_arm_controller/follow_joint_trajectory")
                )
            else:
                self._arm_joint_traj_client_connected = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot."
                % ("panda_arm_controller/follow_joint_trajectory")
            )

        # Setup a new Panda arm joint trajectory action server
        rospy.logdebug(
            "Creating '%s/panda_arm/follow_joint_trajectory' service."
            % rospy.get_name()
        )
        self._arm_joint_traj_as = actionlib.SimpleActionServer(
            "%s/panda_arm/follow_joint_trajectory" % rospy.get_name().split("/")[-1],
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

    def _wait_till_arm_control_done(  # noqa: C901
        self,
        control_type,
        controlled_joints=None,
        timeout=None,
        check_gradient=True,
    ):
        """Wait till arm control is finished. Meaning the robot state is within range
        of the joint position and joint effort setpoints (or the velocity is zero).

        Args:
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort`` and ``position``.
            controlled_joints (list, optional): A dictionary containing the joints that
                are currently being controlled, these joints will be determined if no
                dictionary is given.
            timeout (int, optional): The timeout when waiting for the control
                to be done. Defaults to :attr:`_wait_till_arm_control_done_timeout`.
            check_gradient (boolean, optional): If enabled the script will also return
                when the gradients become zero. Defaults to ``True``.
        """
        # Validate control type and control group
        if control_type not in ["position", "effort"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['position', 'effort']")
            )
            return False
        else:
            control_type = control_type.lower()

        # Compute the state masks
        if controlled_joints:
            arm_states_mask = [
                joint in controlled_joints for joint in self.joint_states.name
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
            except InputMessageInvalidError:
                rospy.logwarn(
                    "Not waiting for control to be completed as no information could "
                    "be retrieved about which joints are controlled when using '%s' "
                    "control. Please make sure the '%s' controllers that are needed "
                    "for '%s' control are initialized."
                    % (
                        control_type,
                        ARM_POSITION_CONTROLLERS
                        if control_type == "position"
                        else ARM_EFFORT_CONTROLLERS,
                        control_type,
                    )
                )
                return False
            if not any(arm_states_mask):
                rospy.logwarn(
                    "Not waiting for control to be completed as no joints appear to be "
                    "controlled when using '%s' control. Please make sure the '%s' "
                    "controllers that are needed for '%s' control are initialized."
                    % (
                        control_type,
                        ARM_POSITION_CONTROLLERS
                        if control_type == "position"
                        else ARM_EFFORT_CONTROLLERS,
                        control_type,
                    )
                )
                return False

        # Get set input arguments
        if timeout:
            timeout = rospy.Duration(timeout)
        else:
            timeout = self._wait_till_arm_control_done_timeout

        # Wait till robot positions/efforts reach the setpoint or the velocities are
        # not changing anymore
        timeout_time = rospy.get_rostime() + timeout
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:
            # Wait till joint positions are within range or arm not changing anymore
            if control_type == "position":
                joint_setpoint = self.arm_joint_positions_setpoint
                joint_states = np.array(
                    list(compress(self.joint_states.position, arm_states_mask))
                )
                state_threshold = self.arm_joint_positions_threshold
                grad_threshold = self._arm_joint_positions_grad_threshold
            # Check if joint effort states are within setpoint
            elif control_type == "effort":
                joint_setpoint = self.arm_joint_efforts_setpoint
                joint_states = self.cur_command_torques
                state_threshold = self.arm_joint_efforts_threshold
                grad_threshold = self._arm_joint_efforts_grad_threshold

            # Add current state to state_buffer and delete oldest entry
            state_buffer = np.full((2, len(joint_states)), np.nan)
            grad_buffer = np.full((2, len(joint_states)), np.nan)
            state_buffer = np.delete(
                np.append(state_buffer, [joint_states], axis=0), 0, axis=0
            )
            grad_buffer = np.gradient(state_buffer, axis=0)

            # Check if setpoint is reached
            if check_gradient:  # Check position/effort and gradients
                if (
                    np.linalg.norm(
                        np.array(list(compress(joint_states, arm_states_mask)))
                        - np.array(list(compress(joint_setpoint, arm_states_mask)))
                    )
                    <= state_threshold  # Check if difference norm is within threshold
                ) or all(
                    [
                        (np.abs(val) <= grad_threshold and val != 0.0)
                        for val in list(compress(grad_buffer[-1], arm_states_mask))
                    ]  # Check if all velocities are close to zero
                ):
                    break
            else:  # Only check position/effort
                if (
                    np.linalg.norm(
                        np.array(list(compress(joint_states, arm_states_mask)))
                        - np.array(list(compress(joint_setpoint, arm_states_mask)))
                    )
                    <= state_threshold  # Check if difference norm is within threshold
                ):
                    break

        # Reset setpoints and return result
        if control_type == "position":
            self.arm_joint_positions_setpoint = []
        else:
            self.arm_joint_efforts_setpoint = []
        return True

    def _create_arm_traj_action_server_msg(  # noqa: C901
        self, input_msg, verbose=False
    ):
        """Converts the ``control_msgs.msg.FollowJointTrajectoryGoal`` message that
        is received by the ``panda_control_server`` follow joint trajectory wrapper
        action servers into the right format for the original ``panda_arm_controller``
        `follow_joint_trajectory <https://wiki.ros.org/joint_trajectory_action/>`_
        action server.

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
        controlled_joints_dict = self._get_controlled_joints(
            control_type="trajectory", verbose=verbose
        )

        # Get controlled joints, arm_state and joint_names
        controlled_joints = controlled_joints_dict["arm"]
        controlled_joints_size = len(controlled_joints)
        state_dict = self._retrieve_state_dict(controlled_joints_dict)
        arm_state_dict = state_dict["arm"]
        joint_names = input_msg.trajectory.joint_names

        # Initiate new arm action server messages using the current robot state
        arm_control_msg = copy.deepcopy(input_msg)
        arm_control_msg.trajectory.joint_names = controlled_joints_dict["arm"]
        for idx, waypoint in enumerate(input_msg.trajectory.points):
            arm_control_msg.trajectory.points[idx].positions = list(
                arm_state_dict["positions"].values()
            )
            arm_control_msg.trajectory.points[idx].velocities = list(
                arm_state_dict["velocities"].values()
            )
            arm_control_msg.trajectory.points[idx].accelerations = [0.0] * len(
                arm_control_msg.trajectory.joint_names
            )
            arm_control_msg.trajectory.points[idx].effort = list(
                arm_state_dict["efforts"].values()
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
            waypoints_lengths_not_equal_to_joints = [
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
            if any(waypoints_lengths_not_equal_to_joints):
                # Check if the number of joint positions/velocities/accelerations or
                # efforts exceeds the number of joints
                waypoints_lengths_to_big = {
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
                if any(waypoints_lengths_to_big.values()):
                    invalid_fields_string = list_2_human_text(
                        [key for key, val in waypoints_lengths_to_big.items() if val]
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
                        "panda_gazebo/SetJointPositions",
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
        """Converts the input message of the position/effort control service into a
        control commands that is used by the control publishers. While doing this it
        also verifies whether the given input message is valid.

        Args:
            input_msg (union[:obj:`panda_gazebo.msgs.SetJointPositions`,:obj:`panda_gazebo.msgs.SetJointEfforts`]):
                The service input message we want to convert and validate.
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort`` and ``position``.
            verbose (bool, optional): Boolean specifying whether you want to send
                warning and error message to the ROSlogger. Defaults to ``False``.

        Returns:
            dict: The Panda arm control commands in the order which is are required by
                the publishers.

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                input_msg could not be converted into ``moveit_commander`` arm joint
                position/effort commands.
        """  # noqa: E501
        # Validate control_type argument and extract information from input message
        if control_type == "position":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_positions
        elif control_type == "effort":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_efforts
        else:
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['position', 'effort']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Retrieve controlled joints
        controlled_joints_dict = self._get_controlled_joints(
            control_type=control_type, verbose=verbose
        )

        # Get controlled joints and arm state
        controlled_joints = controlled_joints_dict["arm"]
        controlled_joints_size = len(controlled_joints)
        state_dict = self._retrieve_state_dict(controlled_joints_dict)

        # Get control publisher message type and current joint states
        if control_type == "position":
            arm_state_dict = state_dict["arm"]["positions"]
        elif control_type == "effort":
            arm_state_dict = state_dict["arm"]["efforts"]

        # Check service request input
        if len(joint_names) == 0:  # If not joint_names were given
            # Check if enough joint position commands were given otherwise give warning
            if len(control_input) != controlled_joints_size:
                if control_type == "position":
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
            return control_commands
        else:
            # Check if enough control values were given
            if len(joint_names) != len(control_input):
                if control_type == "position":
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
                    if control_type == "position":
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
                    control_commands = [Float64(item) for item in arm_position_commands]
                    return control_commands

    def _split_joint_commands_req(
        self, joint_commands_req, control_type, verbose=False
    ):
        """Splits the combined :obj:`~panda_gazebo.msg.SetJointControlCommandRequest`
        message into separate arm and gripper messages.

        Args:
            joint_commands_req :obj:`~panda_gazebo.msg.SetJointControlCommandRequest`):
                The joint control command message.
            control_type (str): The type of control that is being executed. Options
                are ``effort``, ``position`` and ``trajectory``.
            verbose (bool, optional): Boolean specifying whether you want to send a
                warning message to the ROS logger. Defaults to ``False``.

        Returns:
            (tuple): tuple containing:
                - union[:obj:`~panda_gazebo.msg.SetJointPositionsRequest`, :obj:`~panda_gazebo.msg.SetJointEffortsRequest`, ``None``]:
                    The arm set joint position/effort message or ``None`` if no
                    joint positions/efforts were found in the joint control command.
                - union[:obj:`~panda_gazebo.msg.SetGripperWidth`, ``None``]: The gripper
                    action goal message or ``None`` if 'gripper_width' was not found in
                    the joint control command.
        """  # noqa: E501
        # Retrieve controlled joints
        controlled_arm_joints = self._get_controlled_joints(
            control_type=control_type, verbose=verbose
        )["arm"]
        controlled_arm_joints = (
            PANDA_JOINTS["arm"] if not controlled_arm_joints else controlled_arm_joints
        )

        # Split control message
        joint_commands_dict = dict(
            zip(joint_commands_req.joint_names, joint_commands_req.joint_commands)
        )
        arm_joint_commands_dict = {
            key: val
            for key, val in joint_commands_dict.items()
            if key in controlled_arm_joints
        }
        gripper_width_command = [
            val
            for key, val in joint_commands_dict.items()
            if key.lower() == "gripper_width"
        ]

        # Create arm control message
        if arm_joint_commands_dict:
            if control_type == "position":
                arm_req = SetJointPositions()
                arm_req.joint_names = list(arm_joint_commands_dict.keys())
                arm_req.joint_positions = list(arm_joint_commands_dict.values())
                arm_req.wait = joint_commands_req.wait
            else:
                arm_req = SetJointEfforts()
                arm_req.joint_names = list(arm_joint_commands_dict.keys())
                arm_req.joint_efforts = list(arm_joint_commands_dict.values())
                arm_req.wait = joint_commands_req.wait
        else:
            arm_req = None

        # Create hand control message
        # NOTE: Use 'kDefaultGripperActionSpeed' see 'franka_gripper.sim.h'
        if gripper_width_command:
            gripper_req = SetGripperWidthRequest()
            gripper_req.width = gripper_width_command[0]
            gripper_req.grasping = joint_commands_req.grasping
            gripper_req.wait = joint_commands_req.wait
        else:
            gripper_req = None

        return arm_req, gripper_req

    def _get_controlled_joints(self, control_type, verbose=False):  # noqa: C901
        """Returns the joints that can be controlled by a given control type.

        Args:
            control_type (str): The type of control that is being executed. Options
                are ``effort``, ``position`` and ``trajectory``.
            verbose (bool, optional): Boolean specifying whether you want to send a
                warning message to the ROS logger. Defaults to ``False``.

        Returns:
            dict: A dictionary containing the joints that are controlled when using a
                given control type, grouped by control group (``arm`` and ``hand``).

        Raises:
            :obj:`panda_gazebo.exceptions.InputMessageInvalidError`: Raised when the
                the ``control_type`` is invalid.
        """
        control_type = control_type.lower()

        # Get the joints which are contolled by a given control type
        controlled_joints_dict = OrderedDict(zip(["arm", "hand", "both"], [[], [], []]))
        if control_type == "position":
            for controller in ARM_POSITION_CONTROLLERS + HAND_CONTROLLERS:
                try:
                    for claimed_resources in self.controllers[
                        controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if controller in ARM_POSITION_CONTROLLERS:
                                controlled_joints_dict["arm"].append(resource)
                            elif controller in HAND_CONTROLLERS:
                                controlled_joints_dict["hand"].append(resource)
                except KeyError:  # Controller not initialized
                    pass
        elif control_type == "effort":
            for controller in ARM_EFFORT_CONTROLLERS + HAND_CONTROLLERS:
                try:
                    for claimed_resources in self.controllers[
                        controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if controller in ARM_EFFORT_CONTROLLERS:
                                controlled_joints_dict["arm"].append(resource)
                            elif controller in HAND_CONTROLLERS:
                                controlled_joints_dict["hand"].append(resource)
                except KeyError:  # Controller not initialized
                    pass
        elif control_type == "trajectory" or control_type == "end_effector":
            for controller in ARM_TRAJ_CONTROLLERS + HAND_CONTROLLERS:
                try:
                    for claimed_resources in self.controllers[
                        controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if controller in ARM_TRAJ_CONTROLLERS:
                                controlled_joints_dict["arm"].append(resource)
                            elif controller in HAND_CONTROLLERS:
                                controlled_joints_dict["hand"].append(resource)
                except KeyError:  # Controller not initialized
                    pass
        else:
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['position', 'effort', 'trajectory']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalidError(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Return controlled joints dict
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
        missing_controllers = []
        stopped_controllers = []
        for controller in controllers:
            try:
                if self.controllers[controller].state != "running":
                    stopped_controllers.append(controller)
            except KeyError:
                missing_controllers.append(controller)
        return missing_controllers, stopped_controllers

    @property
    def controllers(self):
        """Retrieves information about the currently running controllers."""
        return controller_list_array_2_dict(
            self._list_controllers_client.call(ListControllersRequest())
        )

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

    @property
    def cur_command_torques(self):
        """Retrieve currently achieved torque commands. Calculated by subtracting the
        external torque and gravity torque from the effort that is supplied by gazebo.

        Returns:
            np.ndarray: The current torque commands.
        """
        tau_tot = np.array(self._franka_states.tau_J)
        tau_command = np.array(self._franka_states.tau_J_d)
        tau_ext = np.array(self._franka_states.tau_ext_hat_filtered)
        gravity_torque = tau_ext - tau_tot + tau_command
        return tau_tot - gravity_torque - tau_ext

    ################################################
    # Subscribers callback functions ###############
    ################################################
    def _joints_cb(self, data):
        """Callback function for the joint data subscriber."""
        self.joint_states = data

    def _franka_states_cb(self, data):
        """Callback function for the franka states data subscriber."""
        self._franka_states = data

    ################################################
    # Control services callback functions ##########
    ################################################
    def _set_joint_commands_cb(self, set_joint_commands_req):  # noqa: C901
        """Request arm and hand joint command control.

        Args:
            set_joint_commands_req (:obj:`panda_gazebo.srv.SetJointPositionsRequest`):
                Service request message specifying the joint position/effort commands
                for the robot arm and hand joints.

        Returns:
            :obj:`panda_gazebo.srv.SetJointCommandsResponse`: Service response.
        """
        # Validate the control type
        resp = SetJointCommandsResponse()
        if set_joint_commands_req.control_type.lower() not in [
            "position",
            "effort",
        ]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['position', 'effort']")
            )
            resp.success = False
            resp.message = "Specified 'control_type' invalid."
            return resp
        else:
            control_type = set_joint_commands_req.control_type.lower()

        # Split input message and send arm and gripper control commands
        arm_command_msg, gripper_command_msg = self._split_joint_commands_req(
            set_joint_commands_req, control_type
        )

        # Send control commands
        if arm_command_msg is not None:
            if control_type == "position":
                arm_resp = self._arm_set_joint_positions_cb(arm_command_msg)
            else:
                arm_resp = self._arm_set_joint_efforts_cb(arm_command_msg)
        if gripper_command_msg is not None:
            gripper_result = self._set_gripper_width_cb(gripper_command_msg)

        # Return result
        resp = SetJointCommandsResponse()
        if all([gripper_result, arm_resp.success]):
            resp.success = True
            resp.message = "Everything went OK"
        elif all([not item for item in [gripper_result, arm_resp.success]]):
            resp.success = False
            resp.message = "Joint control failed"
        elif not arm_resp.success:
            resp.success = False
            resp.message = "Arm control failed"
        elif not gripper_result:
            resp.success = False
            resp.message = "Gripper control failed"
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

        # Check if all controllers are available and running
        resp = SetJointPositionsResponse()
        missing_controllers, stopped_controllers = self._controllers_running(
            ARM_POSITION_CONTROLLERS
        )

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
            control_pub_msgs = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="position",
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

        # Save setpoint and publish request
        self.arm_joint_positions_setpoint = [
            command.data for command in control_pub_msgs
        ]
        rospy.logdebug("Publishing Panda arm joint positions control message.")
        self._arm_joint_position_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait and not len(stopped_controllers) >= 1:
            self._wait_till_arm_control_done(
                control_type="position",
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

        # Check if all controllers are available and running
        resp = SetJointEffortsResponse()
        missing_controllers, stopped_controllers = self._controllers_running(
            ARM_EFFORT_CONTROLLERS
        )

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
            control_pub_msgs = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="effort",
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

        # Save setpoint and publish request
        self.arm_joint_efforts_setpoint = [command.data for command in control_pub_msgs]
        rospy.logdebug("Publishing Panda arm joint efforts control message.")
        self._arm_joint_effort_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        # NOTE: We currently do not have to wait since the FrankaHWSim does not yet
        # implement control latency. Torques are therefore applied instantly. A feature
        # request for this can be found on
        # https://github.com/frankaemika/franka_ros/issues/208.
        # if set_joint_efforts_req.wait and not len(stopped_controllers) >= 1:
        #     self._wait_till_arm_control_done(
        #         control_type="effort",
        #     )

        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _set_gripper_width_cb(self, set_gripper_width_req):
        """Request gripper width.

        Args:
            set_gripper_width_req (:obj:`panda_gazebo.srv.SetGripperWidth`):
                Service request message specifying the gripper width for the robot hand.

        Returns:
            :obj:`panda_gazebo.srv.SetGripperWidthResponse`: Service response.
        """
        resp = SetGripperWidthResponse()

        # Check if gripper width is within boundaries
        if set_gripper_width_req.width < 0.0 or set_gripper_width_req.width > 0.08:
            rospy.logwarn(
                "Gripper width was clipped as it was not within bounds [0, 0.8]."
            )
            gripper_width = np.clip(set_gripper_width_req.width, 0, 0.08)
        else:
            gripper_width = set_gripper_width_req.width

        # Create gripper command message
        req = GripperCommandGoal()
        req.command.position = gripper_width
        req.command.max_effort = GRASP_FORCE if set_gripper_width_req.grasping else 0

        # Invoke 'franka_gripper' action service
        if self._gripper_command_client_connected:
            self.gripper_width_setpoint = gripper_width
            self._gripper_command_client.send_goal(req)

            # Wait for result
            if set_gripper_width_req.wait:
                gripper_result = self._gripper_command_client.wait_for_result(
                    timeout=rospy.Duration.from_sec(ACTION_TIMEOUT)
                )
                resp.success = gripper_result
                self.gripper_width_setpoint = None
            else:
                resp.success = True
        else:
            rospy.logwarn(
                "Cloud not connect to franka_gripper/{} service.".format(
                    "grasp" if set_gripper_width_req.grasping else "move"
                )
            )
            resp.success = False

        # Add result message and return result
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
            goal (:obj:`panda_gazebo.msg.FollowJointTrajectoryGoal`): Goal execution
                action server goal message.
        """
        # Check if joint goal.joint_names contains duplicates
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
        if not self._arm_joint_traj_client_connected:
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
                goal_msg_dict, feedback_cb=self._arm_joint_traj_feedback_cb
            )

            # Wait for the server to finish performing the action
            self._arm_joint_traj_client.wait_for_result(
                timeout=rospy.Duration.from_sec(ACTION_TIMEOUT)
            )

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
        self._arm_joint_traj_as.publish_feedback(feedback)

    def _arm_joint_traj_preempt_cb(self):
        """Relays the preempt request made to the
        ``panda_control_server/panda_arm/follow_joint_trajectory`` action server wrapper
        to the original ``panda_arm_controller/follow_joint_trajectory`` action server.
        """
        # Stop panda_arm_controller action server
        self._arm_joint_traj_client.cancel_goal()
        self._arm_joint_traj_as.set_preempted()
