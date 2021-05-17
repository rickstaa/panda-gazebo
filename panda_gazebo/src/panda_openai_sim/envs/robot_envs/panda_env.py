"""This module contains the Robot environment class which is responsible
for interaction with the Panda robot (Control and Sensors).
"""

# Main python imports
import sys
import os
from collections import OrderedDict
import numpy as np
import yaml

from panda_openai_sim.envs.robot_gazebo_goal_env import RobotGazeboGoalEnv
from panda_openai_sim.exceptions import EePoseLookupError, EeRpyLookupError
from panda_openai_sim.functions import (
    action_server_exists,
    lower_first_char,
    get_orientation_euler,
    flatten_list,
    joint_positions_2_follow_joint_trajectory_goal,
)
from panda_openai_sim.extras import EulerAngles, Quaternion
from panda_openai_sim.core import PandaControlSwitcher

# ROS python imports
import rospy
import actionlib
import tf2_ros
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose

from panda_openai_sim.msg import FollowJointTrajectoryAction
from panda_openai_sim.srv import (
    GetEe,
    GetEeRequest,
    GetEePose,
    GetEePoseRequest,
    GetEeRpy,
    GetEeRpyRequest,
    SetEe,
    SetEeRequest,
    SetEePose,
    SetEePoseRequest,
    SetJointEfforts,
    SetJointEffortsRequest,
    SetJointPositions,
    SetJointPositionsRequest,
)

# General script parameters
DIRNAME = os.path.dirname(__file__)
PARAMS_CONFIG_PATH = os.path.abspath(
    os.path.join(DIRNAME, "../../../../cfg/_cfg/parms_config.yaml")
)
MOVEIT_SET_EE_POSE_TOPIC = "panda_moveit_planner_server/panda_arm/set_ee_pose"
MOVEIT_GET_EE_POSE_TOPIC = "panda_moveit_planner_server/panda_arm/get_ee_pose"
MOVEIT_GET_EE_RPY_TOPIC = "panda_moveit_planner_server/panda_arm/get_ee_rpy"
MOVEIT_SET_EE_TOPIC = "panda_moveit_planner_server/panda_arm/set_ee"
MOVEIT_GET_EE_TOPIC = "panda_moveit_planner_server/panda_arm/get_ee"
MOVEIT_SET_JOINT_POSITIONS_TOPIC = "panda_moveit_planner_server/set_joint_positions"
SET_JOINT_POSITIONS_TOPIC = "panda_control_server/set_joint_positions"
SET_JOINT_EFFORTS_TOPIC = "panda_control_server/set_joint_efforts"
SET_JOINT_TRAJECTORY_TOPIC = "panda_control_server/follow_joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"
REQUIRED_SERVICES_DICT = {
    "joint_trajectory_control": [SET_JOINT_TRAJECTORY_TOPIC],
    "joint_position_control": [
        [
            SET_JOINT_POSITIONS_TOPIC,
            SET_JOINT_TRAJECTORY_TOPIC,
            MOVEIT_SET_JOINT_POSITIONS_TOPIC,
        ]
    ],
    "joint_effort_control": [SET_JOINT_EFFORTS_TOPIC],
    "joint_group_position_control": [
        [
            SET_JOINT_POSITIONS_TOPIC,
            SET_JOINT_TRAJECTORY_TOPIC,
            MOVEIT_SET_JOINT_POSITIONS_TOPIC,
        ]
    ],
    "joint_group_effort_control": [SET_JOINT_EFFORTS_TOPIC],
    "ee_control": [[MOVEIT_SET_EE_POSE_TOPIC, SET_JOINT_TRAJECTORY_TOPIC]],
}


#################################################
# Panda Robot Environment Class #################
#################################################
class PandaRobotEnv(RobotGazeboGoalEnv):
    """Used for controlling the panda robot and retrieving sensor data.

    Attributes
    ----------
    robot_name_space : str
        The robot name space.
    robot_EE_link : str
        The robot end effector link.
    robot_arm_control_type : str
        The robot arm control type.
    robot_hand_control_type : str
        The robot hand control type.
    joint_states : sensor_msgs.msg.JointState
        The current joint states.

    Methods
    ----------
    get_ee_pose():
        Get end effector pose.
    get_ee_rpy():
        Get end effector orientation.
    set_ee_pose(action):
        Set end effector pose.
    set_joint_efforts(joint_efforts, wait):
        Set joint efforts.
    set_joint_positions(joint_positions, wait):
        Set joint positions.
    set_joint_trajectory(joint_trajectory_msg, wait):
        Set joint trajectory.
    """

    def __init__(
        self,
        robot_EE_link="panda_hand",
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
        robot_name_space="",
        reset_robot_pose=True,
        reset_controls=False,
        reset_control_list=[],
    ):
        """Initializes a new Panda Robot environment.

        Parameters
        ----------
        robot_EE_link : str, optional
            Robot end effector link name, by default panda_hand.
        robot_arm_control_type : str, optional
            The type of control you want to use for the robot arm. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``, by default
            joint_position_control.
        robot_hand_control_type : str, optional
            The type of control you want to use for the robot hand. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``, by default
            joint_position_control.
        robot_name_space : str, optional
            Robot namespace, by default ``""``.
        reset_robot_pose : bool
            Boolean specifying whether to reset the robot pose when the simulation is
            reset.
        reset_controls : bool
            Boolean specifying whether to reset the controllers when the simulation
            is reset.
        reset_control_list : list, optional
            List containing the robot controllers you want to reset each time
            the simulation is reset, by default ``[]``.
        """

        # Environment initiation message
        rospy.loginfo("Initializing Panda Robot environment.")

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
        robot_control_types = parms_config["robot_control_types"]

        # Check if control_types are valid
        control_type_invalid = {
            control_group: control_type
            for control_group, control_type in {
                "arm": robot_arm_control_type,
                "hand": robot_hand_control_type,
            }.items()
            if control_type not in robot_control_types[control_group]
        }
        if control_type_invalid:
            rospy.logerr(
                "Shutting down '%s' because the control %s %s that %s specified for "
                "the Panda robot %s %s invalid. Please use one of the following robot "
                " control types: "
                "%s."
                % (
                    rospy.get_name(),
                    "types" if len(control_type_invalid.keys()) > 1 else "type",
                    list(control_type_invalid.values())
                    if len(control_type_invalid.keys()) > 1
                    else "'" + list(control_type_invalid.values())[0] + "'",
                    "were" if len(control_type_invalid.keys()) > 1 else "was",
                    "hand and arm"
                    if len(control_type_invalid.keys()) > 1
                    else list(control_type_invalid.keys())[0],
                    "are" if len(control_type_invalid.keys()) > 1 else "is",
                    robot_control_types,
                )
            )
            sys.exit(0)

        # Set class attributes
        self.robot_name_space = robot_name_space
        self.robot_EE_link = robot_EE_link
        self.robot_arm_control_type = robot_arm_control_type.lower()
        self.robot_hand_control_type = robot_hand_control_type.lower()
        self._panda_moveit_server_connection_timeout = 5
        self._panda_controller_server_connection_timeout = 5
        self._traj_control_connection_timeout = rospy.Duration(secs=10)
        self._controller_switcher_connection_timeout = 5
        self._joint_traj_action_server_default_step_size = 1  # Time from send [sec]

        # Initialize parent Class to setup the Gazebo environment)
        super(PandaRobotEnv, self).__init__(
            robot_name_space=self.robot_name_space,
            reset_robot_pose=reset_robot_pose,
            reset_controls=reset_controls,
            reset_control_list=reset_control_list,
        )

        #########################################
        # Create controlswitcher and tf #########
        # and tf listener #######################
        #########################################

        # Create controller switcher
        self._controller_switcher = PandaControlSwitcher(
            connection_timeout=self._controller_switcher_connection_timeout
        )

        # Create transform listener
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)

        #########################################
        # Connect to control services ###########
        #########################################
        # NOTE: Depending on the type of control we require the Panda moveit services,
        # the joint trajectory action service, the joint effort control service or the
        # joint position control service. All these services are initiated here if they
        # are available.
        rospy.loginfo("Connecting to robot control services.")

        #################################
        # Moveit Control services #######
        #################################

        # Connect to moveit 'set_ee_pose' topic
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_SET_EE_POSE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_SET_EE_POSE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_set_ee_pose_client = rospy.ServiceProxy(
                MOVEIT_SET_EE_POSE_TOPIC, SetEePose
            )
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_SET_EE_POSE_TOPIC)
            self._services_connection_status[MOVEIT_SET_EE_POSE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_SET_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_SET_EE_POSE_TOPIC] = False

        # Connect to Moveit 'get_ee_pose' service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_GET_EE_POSE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_GET_EE_POSE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_ee_pose_client = rospy.ServiceProxy(
                MOVEIT_GET_EE_POSE_TOPIC, GetEePose
            )
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_GET_EE_POSE_TOPIC)
            self._services_connection_status[MOVEIT_GET_EE_POSE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_GET_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_EE_POSE_TOPIC] = False

        # Connect to Moveit 'get_ee_rpy' service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_GET_EE_RPY_TOPIC)
            rospy.wait_for_service(
                MOVEIT_GET_EE_RPY_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_ee_rpy_client = rospy.ServiceProxy(
                MOVEIT_GET_EE_RPY_TOPIC, GetEeRpy
            )
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_GET_EE_RPY_TOPIC)
            self._services_connection_status[MOVEIT_GET_EE_RPY_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_GET_EE_RPY_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_EE_RPY_TOPIC] = False

        # Connect to Moveit 'set_ee' service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_SET_EE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_SET_EE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_set_ee_client = rospy.ServiceProxy(MOVEIT_SET_EE_TOPIC, SetEe)
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_SET_EE_TOPIC)
            self._services_connection_status[MOVEIT_SET_EE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn("Failed to connect to '%s' service!" % MOVEIT_SET_EE_TOPIC)
            self._services_connection_status[MOVEIT_SET_EE_TOPIC] = False

        # Connect to Moveit 'get_ee' service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_GET_EE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_GET_EE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_ee_client = rospy.ServiceProxy(MOVEIT_GET_EE_TOPIC, GetEe)
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_GET_EE_TOPIC)

            # Validate class set EE with moveit end effector
            self._moveit_ee = self._moveit_get_ee_client(GetEeRequest()).ee_name
            if (
                self._moveit_ee != self.robot_EE_link
            ) and self.robot_arm_control_type == "ee_control":
                rospy.logwarn(
                    "The Moveit control end effector was set to '%s' while "
                    "the Panda robot environment class end effector was set to '%s'."
                    "Changing Moveit end effector to '%s' as the EE for these two "
                    "classes needs to be equal when using 'ee_control'."
                    % (self._moveit_ee, self.robot_EE_link, self._moveit_ee)
                )

                # Set moveit EE
                resp = self._moveit_set_ee_client(
                    SetEeRequest(ee_name=self.robot_EE_link)
                )

                # Check ee set result
                # Only shutdown if ee_control is used
                if not resp.success:
                    rospy.logerr(
                        "Shutting down '%s' because '%s' could not be set as "
                        "the Moveit move_group end effector (EE). Please "
                        "check that the EE you initiate the PandaRobotEnv "
                        "class with is valid." % (rospy.get_name(), self.robot_EE_link)
                    )
                    sys.exit(0)
            else:
                self._services_connection_status[MOVEIT_GET_EE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn("Failed to connect to '%s' service!" % MOVEIT_GET_EE_TOPIC)
            self._services_connection_status[MOVEIT_GET_EE_TOPIC] = False

        # Connect to Moveit 'set_joint_positions' service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_SET_JOINT_POSITIONS_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_SET_JOINT_POSITIONS_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_set_joint_positions_client = rospy.ServiceProxy(
                MOVEIT_SET_JOINT_POSITIONS_TOPIC, SetJointPositions
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[MOVEIT_SET_JOINT_POSITIONS_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[MOVEIT_SET_JOINT_POSITIONS_TOPIC] = False

        #################################
        # Trajectory action service #####
        #################################

        # Connect to Joint Trajectory Panda Control (action) service
        rospy.logdebug(
            "Connecting to '%s' action service." % SET_JOINT_TRAJECTORY_TOPIC
        )
        if action_server_exists(SET_JOINT_TRAJECTORY_TOPIC):  # Check if exists

            # Connect to robot control action server
            self._joint_traj_control_client = actionlib.SimpleActionClient(
                SET_JOINT_TRAJECTORY_TOPIC, FollowJointTrajectoryAction
            )
            # Waits until the action server has started up
            retval = self._joint_traj_control_client.wait_for_server(
                timeout=self._traj_control_connection_timeout
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot." % (SET_JOINT_TRAJECTORY_TOPIC)
                )
                self._services_connection_status[SET_JOINT_TRAJECTORY_TOPIC] = False
            else:
                self._services_connection_status[SET_JOINT_TRAJECTORY_TOPIC] = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot." % (SET_JOINT_TRAJECTORY_TOPIC)
            )
            self._services_connection_status[SET_JOINT_TRAJECTORY_TOPIC] = False

        #################################
        # Panda control services ########
        #################################

        # Connect to Panda Control server 'set_joint_positions' service
        try:
            rospy.logdebug("Connecting to '%s' service." % SET_JOINT_POSITIONS_TOPIC)
            rospy.wait_for_service(
                SET_JOINT_POSITIONS_TOPIC,
                timeout=self._panda_controller_server_connection_timeout,
            )
            self._set_joint_positions_client = rospy.ServiceProxy(
                SET_JOINT_POSITIONS_TOPIC, SetJointPositions
            )
            rospy.logdebug("Connected to '%s' service!" % SET_JOINT_POSITIONS_TOPIC)
            self._services_connection_status[SET_JOINT_POSITIONS_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[SET_JOINT_POSITIONS_TOPIC] = False

        # Connect to Panda Control server 'set_joint_efforts' service
        try:
            rospy.logdebug("Connecting to '%s' service." % SET_JOINT_EFFORTS_TOPIC)
            rospy.wait_for_service(
                SET_JOINT_EFFORTS_TOPIC,
                timeout=self._panda_controller_server_connection_timeout,
            )
            self._set_joint_efforts_client = rospy.ServiceProxy(
                SET_JOINT_EFFORTS_TOPIC, SetJointEfforts
            )
            rospy.logdebug("Connected to '%s' service!" % SET_JOINT_EFFORTS_TOPIC)
            self._services_connection_status[SET_JOINT_EFFORTS_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % SET_JOINT_EFFORTS_TOPIC
            )
            self._services_connection_status[SET_JOINT_EFFORTS_TOPIC] = False

        #########################################
        # Validate service control connections ##
        #########################################
        # NOTE: Shut down ROS node if the services that are needed for a given control
        # type are not available.
        (retval, missing_services) = self._required_services_available()

        # Shut down ROS node if required services are not available
        if not retval:

            # Remove all but the first service when a nested list is given
            # NOTE: Only give the user an warning about the main service they are
            # missing not the possible alternatives
            missing_services_sparse = {"arm": [], "hand": []}
            for key, services in missing_services.items():
                if isinstance(services, list):
                    missing_services_sparse[key].append(services[0])
                else:
                    missing_services_sparse[key].append(services)

            # Create error message and send log
            control_types = [
                self.robot_arm_control_type
                if key == "arm"
                else self.robot_hand_control_type
                for (key, val) in missing_services.items()
            ]
            logerror_msg_strings = [
                (
                    "{} and {}".format(*missing_services.keys())
                    if len(missing_services.keys()) > 1
                    else "{}".format(*missing_services.keys())
                ),
                (
                    "'{}' and '{}'".format(*control_types)
                    if len(control_types) > 1
                    else "'{}'".format(*control_types)
                ),
            ]
            rospy.logerr(
                "Shutting down '{}' node because the Panda robot services '{}' which "
                "are needed for controlling the Panda {} using {} control are not "
                "available.".format(
                    rospy.get_name(),
                    flatten_list(list(missing_services_sparse.values())),
                    logerror_msg_strings[0],
                    logerror_msg_strings[1],
                )
            )
            sys.exit(0)

        #########################################
        # Switch to right controller ############
        #########################################

        # Switch to the right controller
        rospy.loginfo("Switching to required controller.")
        arm_switch_resp = self._controller_switcher.switch(
            control_group="arm", control_type=self.robot_arm_control_type
        )
        hand_switch_resp = self._controller_switcher.switch(
            control_group="hand", control_type=self.robot_hand_control_type
        )

        # Check whether the controller was available
        control_switch_success = [arm_switch_resp.success, hand_switch_resp.success]
        if not all(control_switch_success):
            logerror_msg_strings = [
                "hand and arm control types"
                if not all(control_switch_success)
                else (
                    "arm control type"
                    if control_switch_success[0]
                    else "hand control type"
                ),
                [arm_switch_resp.prev_control_type, hand_switch_resp.prev_control_type],
                [self.robot_arm_control_type, self.robot_hand_control_type],
            ]
            rospy.logerr(
                "Shutting down '%s' node because the Panda %s could could not be "
                "switched from %s to %s."
                % (
                    rospy.get_name(),
                    logerror_msg_strings[0],
                    logerror_msg_strings[1],
                    logerror_msg_strings[2],
                )
            )
            sys.exit(0)

        # Environment initiation complete message
        rospy.loginfo("Panda Robot environment initialized.")

    #############################################
    # Overload Gazebo env virtual methods #######
    #############################################
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Returns
        -------
        bool
            Boolean specifying whether reset was successful.
        """
        self._check_all_sensors_ready()
        return True

    #############################################
    # Panda Robot env main methods ##############
    #############################################
    def get_ee_pose(self):
        """Returns the end effector EE pose.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            The current end effector pose.
        """

        # Retrieve end effector pose
        try:

            # Retrieve EE pose using tf2
            grip_site_trans = self._tfBuffer.lookup_transform(
                "world", self.robot_EE_link, rospy.Time()
            )

            # Transform trans to pose
            gripper_pose = PoseStamped()
            gripper_pose.header = grip_site_trans.header
            gripper_pose.pose.orientation = grip_site_trans.transform.rotation
            gripper_pose.pose.position = grip_site_trans.transform.translation
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:

            # Retrieve end effector pose using moveit
            if self._services_connection_status[MOVEIT_GET_EE_POSE_TOPIC]:
                gripper_pose_req = GetEePoseRequest()
                gripper_pose = self._moveit_get_ee_pose_client(gripper_pose_req)
            else:
                logwarn_msg = (
                    "End effector pose could not be retrieved as "
                    + lower_first_char(e.args[0])
                )
                raise EePoseLookupError(
                    message="End effector pose could not be retrieved.",
                    log_message=logwarn_msg,
                )

        # return EE pose
        return gripper_pose

    def get_ee_rpy(self):
        """Returns the end effector EE orientation.

        Returns
        -------
        panda_openai_sim.srv.GetEeRpyResponse
            Object containing the roll (x), yaw (z), pitch (y) euler angles.
        """

        # Retrieve end effector pose
        try:

            # Retrieve EE pose using tf2
            grip_site_trans = self._tfBuffer.lookup_transform(
                "world", self.robot_EE_link, rospy.Time()
            )

            # Transform trans to pose
            gripper_pose = PoseStamped()
            gripper_pose.header = grip_site_trans.header
            gripper_pose.pose.orientation = grip_site_trans.transform.rotation
            gripper_pose.pose.position = grip_site_trans.transform.translation

            # Convert EE pose to rpy
            gripper_rpy = get_orientation_euler(gripper_pose.pose)  # Yaw, Pitch Roll

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:

            # Retrieve end effector pose using moveit
            if self._services_connection_status[MOVEIT_GET_EE_POSE_TOPIC]:
                gripper_rpy_req = GetEeRpyRequest()
                gripper_rpy_resp = self._moveit_get_ee_rpy_client(
                    gripper_rpy_req
                )  # Roll Pitch yaw

                # Convert to Yaw (z), Pitch (y) and Roll (x) representation
                gripper_rpy = EulerAngles()
                gripper_rpy.y = gripper_rpy_resp.p
                gripper_rpy.p = gripper_rpy_resp.y
                gripper_rpy.r = gripper_rpy_resp.r
            else:
                logwarn_msg = (
                    "End effector orientation (rpy) could not be retrieved as "
                    + lower_first_char(e.args[0])
                )
                raise EeRpyLookupError(
                    message="End effector orientation (rpy) could not be retrieved.",
                    log_message=logwarn_msg,
                )

        # Return EE orientation (RPY)
        return gripper_rpy

    def set_ee_pose(self, ee_pose):
        """Sets the Panda end effector pose.

        Parameters
        ----------
        ee_pose :  geometry_msgs.msg.Pose, panda_openai_sim.msg.SetEePoseRequest
            The end effector pose.
        dict, list, tuple, int, float or numpy.ndarray
            A list or pose message containing the end effector position (x, y, z)
            and orientation (x, y, z, w).
        """

        # Try to setting EE pose if service is available
        if self._services_connection_status[MOVEIT_SET_EE_POSE_TOPIC]:

            # Retrieve the current EE pose and convert to pose dict
            cur_ee_pose = self.get_ee_pose()
            cur_ee_pose_dict = OrderedDict(
                zip(
                    ["x", "y", "z", "rx", "ry", "rz", "rw"],
                    [
                        cur_ee_pose.pose.position.x,
                        cur_ee_pose.pose.position.y,
                        cur_ee_pose.pose.position.z,
                        cur_ee_pose.pose.orientation.x,
                        cur_ee_pose.pose.orientation.y,
                        cur_ee_pose.pose.orientation.z,
                        cur_ee_pose.pose.orientation.w,
                    ],
                )
            )

            # Print log message
            rospy.logdebug(
                "Setting end effector pose using the 'panda_moveit_server/set_ee_pose' "
                "service."
            )

            # Convert list and dict to pose message
            if (
                isinstance(ee_pose, list)
                or isinstance(ee_pose, np.ndarray)
                or isinstance(ee_pose, tuple)
                or isinstance(ee_pose, int)
                or isinstance(ee_pose, float)
            ):

                # Convert int and float to list
                if isinstance(ee_pose, int) or isinstance(ee_pose, float):
                    ee_pose = [ee_pose]

                # Create ee_pose_list containing the current pose
                ee_pose_list = list(cur_ee_pose_dict.values())

                # Overwrite ee_pose_list values with the values of the ee_pose input arg
                for idx, val in enumerate(ee_pose):
                    if idx <= (len(cur_ee_pose_dict.values()) - 1):
                        ee_pose_list[idx] = val
                    else:
                        rospy.logwarn_once(
                            "End effector setpoint contains %s values while it can "
                            "only contain %s. As a result only the first %s values are "
                            "used when setting the end effector setpoint."
                            % (len(ee_pose), len(ee_pose_list), len(ee_pose_list))
                        )

                # Create ee_pose dictionary
                ee_pose = OrderedDict(
                    zip(["x", "y", "z", "rx", "ry", "rz", "rw"], ee_pose_list)
                )
            elif isinstance(ee_pose, dict):

                # Add current state for missing keys in the ee_pose argument
                cur_ee_pose_dict.update(ee_pose)
                ee_pose = cur_ee_pose_dict

            # Create SetJointPositions message
            if isinstance(ee_pose, dict):
                ee_target_pose = Pose()
                ee_target_pose.position.x = ee_pose["x"]
                ee_target_pose.position.y = ee_pose["y"]
                ee_target_pose.position.z = ee_pose["z"]
                ee_target_pose.orientation.x = ee_pose["rx"]
                ee_target_pose.orientation.y = ee_pose["ry"]
                ee_target_pose.orientation.z = ee_pose["rz"]
                ee_target_pose.orientation.w = ee_pose["rw"]
            elif isinstance(ee_pose, PoseStamped):

                # Create Pose message
                ee_target_pose = Pose()
                ee_target_pose.position = ee_pose.pose.position
                ee_target_pose.orientation = ee_pose.pose.orientation
            elif isinstance(ee_pose, Pose):
                ee_target_pose = ee_pose
            else:  # If the ee_pose format is not valid

                # Display warning message and return success bool
                rospy.logwarn(
                    "Setting end effector pose failed since the ee_pose you specified "
                    "was given as a %s while the 'set_ee_pose' function only accepts "
                    "a list, Pose and a PoseStamped." % type(ee_pose)
                )
                return False

            # Make sure the quaternion is normalized
            ee_target_pose.orientation = Quaternion.normalize_quaternion(
                ee_target_pose.orientation
            )

            # Set up a trajectory message to publish.
            if isinstance(ee_pose, SetEePoseRequest):
                ee_target = ee_pose
            else:
                ee_target = SetEePoseRequest()
                ee_target.pose = ee_target_pose

            # Switch to joint_trajectory controllers
            resp_arm = self._controller_switcher.switch(
                control_group="arm", control_type="joint_trajectory_control"
            )
            resp_hand = self._controller_switcher.switch(
                control_group="hand", control_type="joint_trajectory_control"
            )

            # Send control command if switch was successfull
            if all([resp_arm.success, resp_hand.success]):

                # Send control command
                retval = self._moveit_set_ee_pose_client(ee_target)

                # Switch controllers back
                resp_arm = self._controller_switcher.switch(
                    control_group="arm", control_type=resp_arm.prev_control_type
                )
                resp_hand = self._controller_switcher.switch(
                    control_group="hand", control_type=resp_hand.prev_control_type
                )

                # Check if switching back was successfull
                if not all([resp_arm.success, resp_hand.success]):
                    logerr_msg_strings = (
                        "arm and hand control types"
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            "arm control type "
                            if not resp_arm.success
                            else "hand control type"
                        ),
                        "'%s' and '%s'"
                        % (resp_arm.prev_control_type, resp_hand.prev_control_type)
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            resp_arm.prev_control_type
                            if not resp_arm.success
                            else resp_hand.prev_control_type
                        ),
                    )
                    rospy.logerr(
                        "Shutting down '%s' because the Panda %s could not be "
                        "switched back to '%s'."
                        % (
                            rospy.get_name(),
                            logerr_msg_strings[0],
                            logerr_msg_strings[1],
                        )
                    )
                    sys.exit(0)

                # Return success bool
                if not retval.success:
                    logdebug_msg = "End effector pose not set as " + lower_first_char(
                        retval.message
                    )
                    rospy.logdebug(logdebug_msg)
                    return False
                else:
                    return True
        else:

            # Display warning message and return success bool if failed
            rospy.logwarn(
                "Setting end effector pose failed since the required service '%s' was "
                "not available." % MOVEIT_SET_EE_POSE_TOPIC
            )
            return False

    def set_joint_positions(self, joint_positions, wait=None):
        """Sets the Panda arm and hand joint positions.

        Parameters
        ----------
        joint_positions : SetJointPositionsRequest, dict, list, float or numpy.ndarray
            The joint positions of each of the robot joints.
        wait : bool, optional
            Wait till the control has finished, by default False

        Returns
        -------
        bool
            Boolean specifying if the joint positions were set successfully.
        """
        # NOTE: Setting the joint position is done by using the panda_control_server and
        # panda_moveit_server services. First we try to use the
        # 'panda_control_server/set_joint_positions' service then the
        # 'panda_control_server/follow_joint_trajectory' action service and lastly the
        # 'panda_moveit_server/set_joint_positions' service.

        # Convert joint_positions to dictionary if type is list or numpy.ndarray
        if (
            isinstance(joint_positions, list)
            or isinstance(joint_positions, np.ndarray)
            or isinstance(joint_positions, tuple)
            or isinstance(joint_positions, int)
            or isinstance(joint_positions, float)
        ):

            # Convert int and float to list
            if isinstance(joint_positions, int) or isinstance(joint_positions, float):
                joint_positions = [joint_positions]

            # Create joint positions list
            joint_positions_list = list(self.joint_states.position)

            # Overwrite ee_pose values with the values of the ee_pose argument
            for idx, val in enumerate(joint_positions):
                if idx <= (len(joint_positions_list) - 1):
                    joint_positions_list[idx] = val
                else:
                    rospy.logwarn_once(
                        "Joint positions setpoint contains %s values while it "
                        "can only contain %s. As a result only the first %s "
                        "values are used when setting the end effector "
                        "setpoint."
                        % (
                            len(joint_positions),
                            len(joint_positions_list),
                            len(joint_positions_list),
                        )
                    )

            # Create joint_positions dictionary
            joint_positions = OrderedDict(
                zip(self.joint_states.name, joint_positions_list)
            )

        # Create SetJointPositions message
        if isinstance(joint_positions, dict):
            req = SetJointPositionsRequest()
            req.joint_names = list(joint_positions.keys())
            req.joint_positions = list(joint_positions.values())
        elif isinstance(joint_positions, SetJointPositionsRequest):
            req = joint_positions
        else:

            # Display warning message and return success bool
            rospy.logwarn(
                "Setting joint positions failed since the joint_positions argument has "
                "is of the %s type while the 'set_joint_positions' function only "
                "accepts a dictionary or a SetJointPositions message."
                % type(joint_positions)
            )
            return False

        # Add wait variable to SetJointPositions message
        req.wait = True  # FIXME: Quick fix wait should be fixed in the service
        # if wait:  # If not it will take the msg default value of False
        # req.wait = wait

        #########################################
        # Try using set_joint_positions service #
        #########################################
        if self._services_connection_status[SET_JOINT_POSITIONS_TOPIC]:

            # Print log message
            rospy.logdebug(
                "Setting joint positions using the "
                "'panda_control_server/set_joint_positions' service."
            )

            # Switch to joint position controllers
            resp_arm = self._controller_switcher.switch(
                control_group="arm", control_type="joint_position_control"
            )
            resp_hand = self._controller_switcher.switch(
                control_group="hand", control_type="joint_position_control"
            )

            # Send control command if switch was successfull
            if all([resp_arm.success, resp_hand.success]):

                # Send control command
                self._set_joint_positions_client.call(req)

                # Switch controllers back
                resp_arm = self._controller_switcher.switch(
                    control_group="arm", control_type=resp_arm.prev_control_type
                )
                resp_hand = self._controller_switcher.switch(
                    control_group="hand", control_type=resp_hand.prev_control_type
                )

                # Check if switching back was successfull
                if not all([resp_arm.success, resp_hand.success]):
                    logerr_msg_strings = (
                        "arm and hand control types"
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            "arm control type "
                            if not resp_arm.success
                            else "hand control type"
                        ),
                        "'%s' and '%s'"
                        % (resp_arm.prev_control_type, resp_hand.prev_control_type)
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            resp_arm.prev_control_type
                            if not resp_arm.success
                            else resp_hand.prev_control_type
                        ),
                    )
                    rospy.logerr(
                        "Shutting down '%s' because the Panda %s could not be "
                        "switched back to '%s'."
                        % (
                            rospy.get_name(),
                            logerr_msg_strings[0],
                            logerr_msg_strings[1],
                        )
                    )
                    sys.exit(0)
                else:
                    # Set success bool
                    return True

        #########################################
        # Try using traj action server ##########
        #########################################
        if self._services_connection_status[SET_JOINT_TRAJECTORY_TOPIC]:

            # Print log message
            rospy.logdebug(
                "Setting joint positions using the "
                "'panda_control_server/follow_joint_trajectory' service."
            )

            # Switch to joint trajectory controllers
            resp_arm = self._controller_switcher.switch(
                control_group="arm", control_type="joint_trajectory_control"
            )
            resp_hand = self._controller_switcher.switch(
                control_group="hand", control_type="joint_trajectory_control"
            )

            # Send control command if switch was successfull
            if all([resp_arm.success, resp_hand.success]):

                # Create follow joint trajectory control command message
                goal_req = joint_positions_2_follow_joint_trajectory_goal(
                    req,
                    time_from_start=self._joint_traj_action_server_default_step_size,
                )

                # Send control command
                self._joint_traj_control_client.send_goal(goal_req)

                # Wait for trajectory completion
                req.wait = True  # FIXME: Quick fix wait should be fixed in the service
                if req.wait:
                    self._joint_traj_control_client.wait_for_result()

                # Switch controllers back
                resp_arm = self._controller_switcher.switch(
                    control_group="arm", control_type=resp_arm.prev_control_type
                )
                resp_hand = self._controller_switcher.switch(
                    control_group="hand", control_type=resp_hand.prev_control_type
                )

                # Check if switching back was successfull
                if not all([resp_arm.success, resp_hand.success]):
                    logerr_msg_strings = (
                        "arm and hand control types"
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            "arm control type "
                            if not resp_arm.success
                            else "hand control type"
                        ),
                        "'%s' and '%s'"
                        % (resp_arm.prev_control_type, resp_hand.prev_control_type)
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            resp_arm.prev_control_type
                            if not resp_arm.success
                            else resp_hand.prev_control_type
                        ),
                    )
                    rospy.logerr(
                        "Shutting down '%s' because the Panda %s could not be "
                        "switched back to '%s'."
                        % (
                            rospy.get_name(),
                            logerr_msg_strings[0],
                            logerr_msg_strings[1],
                        )
                    )
                    sys.exit(0)
                else:
                    # Set success bool
                    return True

        #########################################
        # Use moveit server #####################
        #########################################
        if self._services_connection_status[MOVEIT_SET_JOINT_POSITIONS_TOPIC]:

            # Print log message
            rospy.logdebug(
                "Setting joint positions using the "
                "'panda_moveit_server/set_joint_positions' service."
            )

            # Print wait warning message
            # FIXME: THIS SHOULD BE TRUE
            if req.wait:
                rospy.logwarn(
                    "You set wait to False but his option is neglected since "
                    "the '%s' service is used." % MOVEIT_SET_JOINT_POSITIONS_TOPIC
                )

            # Switch to joint trajectory controllers
            resp_arm = self._controller_switcher.switch(
                control_group="arm", control_type="joint_trajectory_control"
            )
            resp_hand = self._controller_switcher.switch(
                control_group="hand", control_type="joint_trajectory_control"
            )

            # Send control command if switch was successfull
            if all([resp_arm.success, resp_hand.success]):

                # Send control command
                self._moveit_set_joint_positions_client.call(req)

                # Switch controllers back
                resp_arm = self._controller_switcher.switch(
                    control_group="arm", control_type=resp_arm.prev_control_type
                )
                resp_hand = self._controller_switcher.switch(
                    control_group="hand", control_type=resp_hand.prev_control_type
                )

                # Check if switching back was successfull
                if not all([resp_arm.success, resp_hand.success]):
                    logerr_msg_strings = (
                        "arm and hand control types"
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            "arm control type "
                            if not resp_arm.success
                            else "hand control type"
                        ),
                        "'%s' and '%s'"
                        % (resp_arm.prev_control_type, resp_hand.prev_control_type)
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            resp_arm.prev_control_type
                            if not resp_arm.success
                            else resp_hand.prev_control_type
                        ),
                    )
                    rospy.logerr(
                        "Shutting down '%s' because the Panda %s could not be "
                        "switched back to '%s'."
                        % (
                            rospy.get_name(),
                            logerr_msg_strings[0],
                            logerr_msg_strings[1],
                        )
                    )
                    sys.exit(0)
                else:
                    # Set success bool
                    return True
        else:

            # Display warning message and return success bool if failed
            rospy.logwarn(
                "Setting joints positions failed since no of the required services "
                "'%s' were available."
                % [
                    SET_JOINT_POSITIONS_TOPIC,
                    SET_JOINT_TRAJECTORY_TOPIC,
                    MOVEIT_SET_JOINT_POSITIONS_TOPIC,
                ]
            )
            return False

    def set_joint_efforts(self, joint_efforts, wait=None):
        """Sets the Panda arm and hand joint efforts.

        Parameters
        ----------
        joint_efforts : SetJointEffortsRequest, dict, list, float or numpy.ndarray
            The joint efforts of each of the robot joints. Can also be supplied as
        wait : bool, optional
            Wait till the control has finished, by default False

        Returns
        -------
        bool
            Boolean specifying if the joint efforts were set successfully.
        """

        # Convert joint_efforts to dictionary if type is list or numpy.ndarray
        if (
            isinstance(joint_efforts, list)
            or isinstance(joint_efforts, np.ndarray)
            or isinstance(joint_efforts, tuple)
            or isinstance(joint_efforts, int)
            or isinstance(joint_efforts, float)
        ):

            # Convert int and float to list
            if isinstance(joint_efforts, int) or isinstance(joint_efforts, float):
                joint_efforts = [joint_efforts]

            # Create joint efforts list
            joint_efforts_list = list(self.joint_states.effort)

            # Overwrite ee_pose values with the values of the ee_pose argument
            for idx, val in enumerate(joint_efforts):
                if idx <= (len(joint_efforts_list) - 1):
                    joint_efforts_list[idx] = val
                else:
                    rospy.logwarn_once(
                        "Joint efforts setpoint contains %s values while it "
                        "can only contain %s. As a result only the first %s "
                        "values are used when setting the end effector "
                        "setpoint."
                        % (
                            len(joint_efforts),
                            len(joint_efforts_list),
                            len(joint_efforts_list),
                        )
                    )

            # Create joint_efforts dictionary
            joint_efforts = OrderedDict(zip(self.joint_states.name, joint_efforts_list))

        # Create SetJointEfforts message
        if isinstance(joint_efforts, dict):
            req = SetJointEffortsRequest()
            req.joint_names = list(joint_efforts.keys())
            req.joint_efforts = list(joint_efforts.values())
        elif isinstance(joint_efforts, SetJointEffortsRequest):
            req = joint_efforts
        else:

            # Display warning message and return success bool
            rospy.logwarn(
                "Setting joint efforts failed since the joint_efforts argument has "
                "is of the %s type while the 'set_joint_efforts' function only "
                "accepts a dictionary or a SetJointEfforts message."
                % type(joint_efforts)
            )
            return False

        # Add wait variable to SetJointEfforts message
        req.wait = True  # FIXME: Quick fix wait should be fixed in the service
        # if wait:  # If not it will take the msg default value of False
        #     req.wait = wait

        # Try to setting joint efforts if service is available
        if self._services_connection_status[SET_JOINT_EFFORTS_TOPIC]:

            # Print log message
            rospy.logdebug(
                "Setting joint efforts using the "
                "'panda_control_server/set_joint_efforts' service."
            )

            # Switch to joint effort controllers
            resp_arm = self._controller_switcher.switch(
                control_group="arm", control_type="joint_effort_control"
            )
            resp_hand = self._controller_switcher.switch(
                control_group="hand", control_type="joint_effort_control"
            )

            # Send control command if switch was successfull
            if all([resp_arm.success, resp_hand.success]):

                # Send control command
                self._set_joint_efforts_client.call(req)

                # Switch controllers back
                resp_arm = self._controller_switcher.switch(
                    control_group="arm", control_type=resp_arm.prev_control_type
                )
                resp_hand = self._controller_switcher.switch(
                    control_group="hand", control_type=resp_hand.prev_control_type
                )

                # Check if switching back was successfull
                if not all([resp_arm.success, resp_hand.success]):
                    logerr_msg_strings = (
                        "arm and hand control types"
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            "arm control type "
                            if not resp_arm.success
                            else "hand control type"
                        ),
                        "'%s' and '%s'"
                        % (resp_arm.prev_control_type, resp_hand.prev_control_type)
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            resp_arm.prev_control_type
                            if not resp_arm.success
                            else resp_hand.prev_control_type
                        ),
                    )
                    rospy.logerr(
                        "Shutting down '%s' because the Panda %s could not be "
                        "switched back t '%s'."
                        % (
                            rospy.get_name(),
                            logerr_msg_strings[0],
                            logerr_msg_strings[1],
                        )
                    )
                    sys.exit(0)
                else:
                    # Set success bool
                    return True
        else:

            # Display warning message and return success bool if failed
            rospy.logwarn(
                "Setting joints efforts failed since the required service '%s' was not "
                "available." % SET_JOINT_EFFORTS_TOPIC
            )
            return False

    def set_joint_trajectory(self, joint_trajectory_msg, wait=False):
        """Sets the joint trajectory the Panda robot should follow.

        Parameters
        ----------
        joint_trajectory_msg : panda_openai_sim.msg.FollowJointTrajectoryGoal
            The joint trajectory goal message.
        wait : bool, optional
            Wait till the control has finished, by default False

        Returns
        -------
        bool
            Boolean specifying if the joint trajectory was set successfully.
        """

        # Try setting a joint trajectory if service is available
        if self._services_connection_status[SET_JOINT_TRAJECTORY_TOPIC]:

            # Switch to joint trajectory controllers
            resp_arm = self._controller_switcher.switch(
                control_group="arm", control_type="joint_trajectory_control"
            )
            resp_hand = self._controller_switcher.switch(
                control_group="hand", control_type="joint_trajectory_control"
            )

            # Send control command if switch was successfull
            if all([resp_arm.success, resp_hand.success]):

                # Send control command
                self._joint_traj_control_client.send_goal(joint_trajectory_msg)
                if wait:
                    self._joint_traj_control_client.wait_for_result()

                # Switch controllers back
                resp_arm = self._controller_switcher.switch(
                    control_group="arm", control_type=resp_arm.prev_control_type
                )
                resp_hand = self._controller_switcher.switch(
                    control_group="hand", control_type=resp_hand.prev_control_type
                )

                # Check if switching back was successfull
                if not all([resp_arm.success, resp_hand.success]):
                    logerr_msg_strings = (
                        "arm and hand control types"
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            "arm control type "
                            if not resp_arm.success
                            else "hand control type"
                        ),
                        "'%s' and '%s'"
                        % (resp_arm.prev_control_type, resp_hand.prev_control_type)
                        if all(
                            [
                                not bool_item
                                for bool_item in [resp_arm.success, resp_hand.success]
                            ]
                        )
                        else (
                            resp_arm.prev_control_type
                            if not resp_arm.success
                            else resp_hand.prev_control_type
                        ),
                    )
                    rospy.logerr(
                        "Shutting down '%s' because the Panda %s could not be "
                        "switched back to '%s'."
                        % (
                            rospy.get_name(),
                            logerr_msg_strings[0],
                            logerr_msg_strings[1],
                        )
                    )
                    sys.exit(0)
                else:
                    # Set success bool
                    return True
        else:

            # Display warning message and return success bool if failed
            rospy.logwarn(
                "Setting joints trajectory failed since the required service '%s' was "
                "not available." % SET_JOINT_TRAJECTORY_TOPIC
            )
            return False

    #############################################
    # Panda Robot env helper methods ############
    #############################################
    def _required_services_available(self):
        """Checks if all services required for the current 'robot_arm_control_type' are
        available.

        Returns
        -------
        bool
            Boolean specifying if the required services are available.
        dict
            Dictionary containing the missing services grouped by control_group.
        """

        # Setup required, connected and missing services dictionaries
        required_services = {
            "arm": REQUIRED_SERVICES_DICT[self.robot_arm_control_type],
            "hand": REQUIRED_SERVICES_DICT[self.robot_hand_control_type],
        }
        connected_services = [
            key for (key, val) in self._services_connection_status.items() if val
        ]
        missing_services = {}

        # Loop through required services and check if they are missing
        for (key, req_srvs_list) in required_services.items():
            for req_srvs in req_srvs_list:
                if isinstance(req_srvs, list):  # If nested list
                    # NOTE: If nested list only one of the services has to be connected
                    # otherwise a missing service is directly added to the missing
                    # services dictionary
                    missing_req_srvs = []
                    for req_srv in req_srvs:
                        if req_srv not in connected_services:
                            missing_req_srvs.append(req_srv)

                    # Add the missing services to the dictionary
                    if len(missing_req_srvs) == len(req_srvs):
                        missing_services.update({key: missing_req_srvs})
                else:
                    if req_srvs not in connected_services:
                        missing_services.update({key: req_srvs})

        # Check whether any services are missing and return result
        if len(missing_services.values()) >= 1:
            return (False, missing_services)
        else:
            return (True, missing_services)

    def _check_all_sensors_ready(self):
        """Checks whether we are receiving sensor data."""
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        """Checks if we are receiving joint state
        sensor data.

        Returns
        -------
        sensor_msgs.msgs.JointState
            Array containing the joint states.
        """

        self.__joint_states = None
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    "/joint_states", JointState, timeout=1.0
                )
                rospy.logdebug("Current /joint_states READY=>" + str(self.joint_states))

            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting "
                    "joint_states"
                )
        return self.joint_states

    #############################################
    # Callback functions ########################
    #############################################
    # NOTE: Implemented as a class property since using subscribers without the
    # ros.spin() command did not work.
    @property
    def joint_states(self):
        """Function for retrieving the joint_state data."""
        return rospy.wait_for_message(JOINT_STATES_TOPIC, JointState)

    @joint_states.setter
    def joint_states(self, val):
        """Function for setting the joint_state data."""
        self.__joint_states = val

    #############################################
    # Setup virtual methods #####################
    #############################################
    # NOTE: These virtual methods NEED to be overloaded by the Task env

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Returns the observation.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Indicates whether or not the episode is done. The goal was achieved or the
        robot has fallen.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _sample_goal(self):
        """Samples a new goal and returns it.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _set_init_pose(self):
        """Sets the Robot in its init pose.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _set_init_obj_pose(self):
        """Sets the Object to its init pose.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()
