"""This is the main Panda openai_ros gym environment. It provided all the methods
necessary for training an RL algorithm on the Panda robot.
"""

# Main python imports
import os
import sys
from collections import OrderedDict
import yaml
import numpy as np
from gym import utils
from gym import spaces

from panda_openai_sim.envs.robot_envs import PandaRobotEnv
from panda_openai_sim.extras import TargetMarker, SampleRegionMarker
from panda_openai_sim.exceptions import (
    EePoseLookupError,
    RandomJointPositionsError,
    RandomEePoseError,
    SpawnModelError,
    SetModelStateError,
)
from panda_openai_sim.errors import (
    arg_type_error,
    arg_keys_error,
    arg_value_error,
)
from panda_openai_sim.functions import (
    flatten_list,
    get_orientation_euler,
    lower_first_char,
    has_invalid_type,
    contains_keys,
    has_invalid_value,
    pose_dict_2_pose_msg,
    pose_msg_2_pose_dict,
    split_bounds_dict,
    split_pose_dict,
    translate_gripper_width_2_finger_joint_commands,
    log_pose_dict,
    action_dict_2_joint_trajectory_msg,
    list_2_human_text,
    merge_two_dicts,
)

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose

from panda_openai_sim.msg import BoundingRegion, JointLimits
from panda_openai_sim.srv import (
    GetRandomEePose,
    GetRandomEePoseRequest,
    GetRandomJointPositions,
    GetRandomJointPositionsRequest,
    GetControlledJoints,
    GetControlledJointsRequest,
)

# TODO: Add current target as ros param
# TODO: Change reward topic
# TODO: Check panda_task env with gym and openai_ros
# TODO: Wait for joint trajectory control
# TODO: Add fix gripper rotation see self._set_Action of gym makes sure that init pose is set
# TODO: Check log message
# TODO: Check control switcher
# TODO: Check why finger joints can not be controlled
# TODO: Add wait variable to moveit and joint_trajectory control
# TODO: ADD wait between commands config variable.
# TODO: Check ee pose not set warning when using panda_training scripts

# Script Parameters
DIRNAME = os.path.dirname(__file__)
PARAMS_CONFIG_PATH = os.path.abspath(
    os.path.join(DIRNAME, "../../../../cfg/_cfg/parms_config.yaml")
)
ENV_CONFIG_PATH = os.path.abspath(
    os.path.join(DIRNAME, "../../../../cfg/env_config.yaml")
)
MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC = (
    "panda_moveit_planner_server/get_random_joint_positions"
)
MOVEIT_GET_RANDOM_EE_POSE_TOPIC = "panda_moveit_planner_server/get_random_ee_pose"
GET_CONTROLLED_JOINTS_TOPIC = "panda_control_server/get_controlled_joints"
POSITION_CONTROL_TYPES = [
    "joint_position_control",
    "joint_group_position_control",
    "joint_trajectory_control",
]
EFFORT_CONTROL_TYPES = [
    "joint_effort_control",
    "joint_group_effort_control",
]
REWARD_TYPES = ["sparse", "dense"]
TARGET_SAMPLING_STRATEGIES = ["local", "global"]
GOAL_SAMPLING_STRATEGIES = ["global", "local"]
GOAL_SAMPLING_REFERENCES = ["initial", "current"]
INIT_POSE_TYPES = ["ee_pose", "qpose"]
GRASP_OBJECT_NAME = "grasp_object_0"


#################################################
# Panda Task environment Class ##################
#################################################
class PandaTaskEnv(PandaRobotEnv, utils.EzPickle):
    """Class that provides all the methods used for the algorithm training.

    Attributes
    ----------
    action_space : gym.spaces.box.Box
        Gym action space object.
    observation_space : gym.spaces.dict.Dict
        Gym observation space object.
    goal : geometry_msgs.PoseStamped
        The current goal.
    initial_ee_pos : numpy.ndarray
        The initial robot pose for the current episode.
    initial_qpose : collections.OrderedDict
        The initial generalized joint positions of the robot for the current episode.
    reward_type : str, optional
        The reward type, i.e. ``sparse`` or ``dense``, by default None.
    """

    def __init__(
        self,
        reward_type=None,
        distance_threshold=None,
        has_object=None,
        block_gripper=None,
        target_in_the_air=None,
        gripper_extra_height=None,
        target_offset=None,
        target_bounds=None,
        target_sampling_strategy=None,
        init_pose=None,
        init_pose_bounds=None,
        init_obj_pose=None,
        obj_bounds=None,
        n_actions=None,
        use_gripper_width=None,
        action_space_joints=None,
        robot_arm_control_type=None,
        robot_hand_control_type=None,
    ):
        """Initializes a Panda Task Environment.


        Parameters
        ----------
        reward_type : str, optional
            The reward type, i.e. ``sparse`` or ``dense``, by default None.
        distance_threshold : float, optional
            The threshold after which a goal is considered achieved, by default None.
        has_object : bool, optional
            Whether or not the environment has an object.
        block_gripper : bool, optional
            Whether or not the gripper is blocked (i.e. not movable) or not, by default
            None.
        target_in_the_air : bool, optional
            Whether or not the target should be in the air above the table or on the
            table surface, by default None.
        gripper_extra_height : float, optional
            Additional height above the table when positioning the gripper.
        target_offset : dict, optional
            A dictionary in which the offset of the target is defined i.e. {x,y,z}, by
            default None.
        target_bounds : dict, optional
            A dictionary with the bounds from which the target is sampled i.e.
            ``{x_min, y_min, z_min, x_max, y_max, x_max}``, by default None.
        target_sampling_strategy : str, optional
            Whether the target bounds from which we sample the target goal position are
            relative to the global frame 'global' or relative to the current
            end-effector pose 'local', by default None.
        init_pose : dict, optional
            A dictionary of names and values that define the initial configuration i.e.
            ``{x, y, z, rx, ry, rz, rw, panda_finger_joint1, panda_fingerjoint_2}``, by
            default None.
        init_pose_bounds : dict, optional
            A dictionary with the bounds from which the initial robot pose is sampled
            i.e. ``{x_min, y_min, z_min, x_max, y_max, x_max}``, by default None.
        init_obj_pose : dict, optional
            A dictionary that contains the initial object pose i.e.
            ``{x, y, z, rx, ry, rz, rw}``, by default None. The object will be spawned
            relative to this pose in a region defined by the obj_bounds.
        obj_bounds : dict, optional
            A dictionary in which the bounds for sampling the object
            positions is defined ``{x_min, y_min, x_max, y_max}``, by default None.
            These bounds are relative to the init_obj_pose.
        n_actions : int, optional
            The size of the action space you want to use. When the 'action_space_joints'
            variable is supplied this variable is ignored and the length of the
            controlled joints variable is used as the action space size, by default
            None.
        use_gripper_width : bool, optional
            Whether you want to use the gripper_width instead of the individual panda
            finger joint commands during the training, by default None. Consequently,
            using the gripper width reduces the action space by 1.
        action_space_joints : list, optional
            A list containing the joints which you want to use in the action space.
            If this variable is supplied the length of the action space will be set
            to be equal to the length of the 'action_space_joints' list, by default
            None.
        robot_arm_control_type : str, optional
            The type of control you want to use for the robot arm. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``, by default None.
        robot_hand_control_type : str, optional
            The type of control you want to use for the robot hand. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``, by default None.

        Note
        -------
            If the default value for a argument is set to None this means the default
            values are used. These values can be set in the
            :env_config:`env_config.yaml <>` file.
        """

        # Log message
        rospy.loginfo("Initializing Panda task environment.")

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
        self._robot_control_types = parms_config["robot_control_types"]
        self._panda_joints = parms_config["panda_joints"]

        # Load task environment configuration settings as class attributes
        self._get_config()

        # Process input arguments
        self._process_input_arguments(
            gripper_extra_height=gripper_extra_height,
            block_gripper=block_gripper,
            has_object=has_object,
            target_in_the_air=target_in_the_air,
            target_sampling_strategy=target_sampling_strategy,
            target_offset=target_offset,
            target_bounds=target_bounds,
            distance_threshold=distance_threshold,
            init_pose=init_pose,
            init_pose_bounds=init_pose_bounds,
            init_obj_pose=init_obj_pose,
            obj_bounds=obj_bounds,
            reward_type=reward_type,
            robot_arm_control_type=robot_arm_control_type,
            robot_hand_control_type=robot_hand_control_type,
            n_actions=n_actions,
            use_gripper_width=use_gripper_width,
            action_space_joints=action_space_joints,
        )

        #########################################
        # Create class attributes and initiate #
        # Robot and Gazebo Goal environments ####
        #########################################

        # Create other class attributes
        self._sim_time = rospy.get_time()
        self._prev_grip_pos = np.zeros(3)
        self._prev_object_pos = np.zeros(3)
        self._prev_object_rot = np.zeros(3)
        self._services_connection_status = {}

        # Wait for the simulation to be started
        wait_for_sim_timeout = 60
        simulation_check_timeout_time = rospy.get_rostime() + rospy.Duration(
            wait_for_sim_timeout
        )
        while (
            not rospy.is_shutdown()
            and rospy.get_rostime() < simulation_check_timeout_time
        ):
            if any(
                [
                    "/gazebo" in topic
                    for topic in flatten_list(rospy.get_published_topics())
                ]
            ):
                break
            else:
                rospy.logwarn_once(
                    "Waiting for the Panda Gazebo robot simulation to be " "started."
                )
        else:
            rospy.logerr(
                "Shutting down '%s' since the Panda Gazebo simulation was not "
                "started within the set timeout period of %s seconds."
                % (rospy.get_name(), wait_for_sim_timeout)
            )
            sys.exit(0)

        # Initialize parent Class to setup the Robot environment
        super(PandaTaskEnv, self).__init__(
            robot_EE_link=self._ee_link,
            robot_arm_control_type=self._robot_arm_control_type,
            robot_hand_control_type=self._robot_hand_control_type,
        )
        utils.EzPickle.__init__(self)

        #########################################
        # Connect to required services, #########
        # subscribers and publishers. ###########
        #########################################

        # Connect to Moveit 'get_random_joint_positions' service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_random_joint_positions_client = rospy.ServiceProxy(
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC, GetRandomJointPositions
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            ] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            ] = False

        # Connect to Moveit 'get_random_ee_pose' service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_GET_RANDOM_EE_POSE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_random_ee_pose_client = rospy.ServiceProxy(
                MOVEIT_GET_RANDOM_EE_POSE_TOPIC, GetRandomEePose
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_RANDOM_EE_POSE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_RANDOM_EE_POSE_TOPIC] = False

        # Connect to Panda control server 'get_controlled_joints' service
        try:
            rospy.logdebug("Connecting to '%s' service." % GET_CONTROLLED_JOINTS_TOPIC)
            rospy.wait_for_service(
                GET_CONTROLLED_JOINTS_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_controlled_joints_client = rospy.ServiceProxy(
                GET_CONTROLLED_JOINTS_TOPIC, GetControlledJoints
            )
            rospy.logdebug("Connected to '%s' service!" % GET_CONTROLLED_JOINTS_TOPIC)
            self._services_connection_status[GET_CONTROLLED_JOINTS_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GET_CONTROLLED_JOINTS_TOPIC
            )
            self._services_connection_status[GET_CONTROLLED_JOINTS_TOPIC] = False

        # Create current target publisher
        rospy.logdebug("Creating target pose publisher.")
        self._target_pose_pub = rospy.Publisher(
            "panda_openai_sim/current_target", Marker, queue_size=10
        )
        rospy.logdebug("Goal target publisher created.")

        # Create target bounding region publisher
        rospy.logdebug("Creating target bounding region publisher.")
        self._target_sample_region_pub = rospy.Publisher(
            "panda_openai_sim/target_sample_region", Marker, queue_size=10
        )
        rospy.logdebug("Target bounding region publisher created.")

        # Create initial pose bounding region publisher
        rospy.logdebug("Creating initial pose sample region publisher.")
        self._init_pose_sample_region_pub = rospy.Publisher(
            "panda_openai_sim/init_pose_sample_region", Marker, queue_size=10
        )
        rospy.logdebug("Initial pose sample region publisher created.")

        # Create initial pose bounding region publisher
        rospy.logdebug("Creating initial object pose bounding region publisher.")
        self._obj_pose_sample_region_pub = rospy.Publisher(
            "panda_openai_sim/obj_pose_sample_region", Marker, queue_size=10
        )
        rospy.logdebug("Initial object pose bounding region publisher created.")

        #########################################
        # Initialize rviz and Gazebo envs #######
        #########################################

        # Initiate task environment
        # TODO: In gym it is in the gazebo goal env?
        rospy.logdebug("Setup initial environment state.")
        self._env_setup()

        # Get observations
        rospy.logdebug("Get initial observation.")
        obs = self._get_obs()

        # Display goal target sample region in rviz
        if self._visualize_target_bounds:

            # Create goal sampling region marker
            goal_sample_region_marker_msg = SampleRegionMarker(
                x_min=self._target_bounds["global"]["x_min"],
                y_min=self._target_bounds["global"]["y_min"],
                z_min=self._target_bounds["global"]["z_min"],
                x_max=self._target_bounds["global"]["x_max"],
                y_max=self._target_bounds["global"]["y_max"],
                z_max=self._target_bounds["global"]["z_max"],
            )

            # Publish goal sample region marker for rviz visualization
            self._target_sample_region_pub.publish(goal_sample_region_marker_msg)

        # Display object sampling region in rviz
        if self._visualize_obj_bounds:

            # Create goal sampling region marker
            obj_sample_region_marker_msg = SampleRegionMarker(
                x_min=self._obj_bounds["x_min"],
                y_min=self._obj_bounds["y_min"],
                z_min=self._init_obj_pose.position.z,
                x_max=self._obj_bounds["x_max"],
                y_max=self._obj_bounds["y_max"],
                z_max=self._init_obj_pose.position.z + 10 ** -19,
            )
            obj_region_color = ColorRGBA()
            obj_region_color.a = 0.15
            obj_region_color.r = 0.0
            obj_region_color.g = 1.0
            obj_region_color.b = 0.0
            obj_sample_region_marker_msg.color = obj_region_color

            # Publish goal sample region marker for rviz visualization
            self._obj_pose_sample_region_pub.publish(obj_sample_region_marker_msg)

        # Display object sampling region in rviz
        if self._visualize_init_pose_bounds and hasattr(self, "init_pose_bounds"):

            # Create goal sampling region marker
            init_pose_sample_region_marker_msg = SampleRegionMarker(
                x_min=self._init_pose_bounds["x_min"],
                y_min=self._init_pose_bounds["y_min"],
                z_min=self._init_pose_bounds["z_min"],
                x_max=self._init_pose_bounds["x_max"],
                y_max=self._init_pose_bounds["y_max"],
                z_max=self._init_pose_bounds["z_max"],
            )
            init_pose_region_color = ColorRGBA()
            init_pose_region_color.a = 0.15
            init_pose_region_color.r = 0.0
            init_pose_region_color.g = 0.0
            init_pose_region_color.b = 1.0
            init_pose_sample_region_marker_msg.color = init_pose_region_color

            # Publish goal sample region marker for rviz visualization
            self._init_pose_sample_region_pub.publish(
                init_pose_sample_region_marker_msg
            )

        #########################################
        # Retrieve the joints that are being ####
        # controlled ############################
        #########################################
        self._controlled_joints = self._get_controlled_joints()

        #########################################
        # Create action and observation space ###
        #########################################
        rospy.logdebug("Setup gym action and observation space.")

        # Validate action space size
        self._validate_action_space_size()

        # Validate or create action space joints
        if self._action_space_joints:  # If exists
            self._validate_action_space_joints()
        else:
            self._action_space_joints = self._get_action_space_joints()

        # Create action space
        self.action_space = self._create_action_space()

        # Create observation space
        self.observation_space = spaces.Dict(
            dict(
                desired_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype="float32"
                ),
                achieved_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype="float32"
                ),
                observation=spaces.Box(
                    -np.inf, np.inf, shape=obs["observation"].shape, dtype="float32"
                ),
            )
        )

        # Environment initiation complete message
        rospy.loginfo("Panda Panda task environment initialized.")

    #############################################
    # Panda Task env main methods ###############
    #############################################
    # NOTE: Overloads virtual methods that were defined in the Robot and Gazebo Goal
    # environments

    def _compute_reward(self, observations, done):
        """Compute the reward.

        Parameters
        ----------
        observations : dict
            Dictionary containing the observations
        done : bool
            Boolean specifying whether an episode is terminated.

        Returns
        -------
        :obj:`numpy.float32`
            Reward that is received by the agent.
        """

        # Calculate the rewards based on the distance from the goal
        d = self._goal_distance(observations["achieved_goal"], self.goal)
        if self.reward_type == "sparse":

            # Print Debug info
            rospy.logdebug("=Reward info=")
            rospy.logdebug("Reward type: Non sparse")
            rospy.logdebug("Goal: %s", self.goal)
            rospy.logdebug("Achieved goal: %s", observations["achieved_goal"])
            rospy.logdebug("Perpendicular distance: %s", d)
            rospy.logdebug("Threshold: %s", self._distance_threshold)
            rospy.logdebug(
                "Received reward: %s",
                -(d > self._distance_threshold).astype(np.float32),
            )

            # Return result
            return -(d > self._distance_threshold).astype(np.float32)
        else:

            # Print Debug info
            rospy.logdebug("=Reward info=")
            rospy.logdebug("Reward type: Sparse")
            rospy.logdebug("Goal: %s", self.goal)
            rospy.logdebug("Achieved goal: %s", observations["achieved_goal"])
            rospy.logdebug("Perpendicular distance: %s", d)
            rospy.logdebug("Threshold: %s", self._distance_threshold)
            rospy.logdebug("Received reward: %s", -d)

            # Return result
            return -d

    def _get_obs(self):
        """Get robot state observation.

        Returns
        -------
        dict
            A dictionary containing the {observation, achieved_goal, desired_goal,
            info):

            observations : list (22x1)
                - End effector x position
                - End effector y position
                - End effector z position
                - Object x position
                - Object y position
                - Object z position
                - Object/gripper rel. x position
                - Object/gripper rel. y position
                - Object/gripper rel. z position
                - EE joints positions
                - Object pitch (y)
                - Object yaw (z)
                - Object roll (x)
                - Object x velocity
                - Object y velocity
                - Object z velocity
                - Object x angular velocity
                - Object y angular velocity
                - Object z angular velocity
                - EE joints velocities
            achieved_goal : object
                The goal that was achieved during execution.
            desired_goal : object
                The desired goal that we asked the agent to attempt to achieve.
            info : dict
                An info dictionary with additional information
        """

        # Retrieve robot end effector pose and orientation
        ee_pose = self.get_ee_pose()
        ee_pos = np.array(
            [ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z]
        )

        # Retrieve robot joint pose and velocity
        # IMPROVE: Retrieve real velocity from gazebo
        dt = self._get_elapsed_time()
        grip_velp = (
            ee_pos - self._prev_grip_pos
        ) / dt  # Velocity(position) = Distance/Time
        robot_qpos, robot_qvel = self._robot_get_obs(self.joint_states)

        # Get ee joint positions and velocities
        # NOTE: For the parallel jaw gripper this are the finger joints
        ee_state = robot_qpos[0:2]
        ee_vel = robot_qvel[0:2]

        # Get object pose and (angular)velocity
        if self._has_object:

            # Get object pose
            object_pos = np.array(
                [
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.x,
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.y,
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.z,
                ]
            )

            # Get object orientation
            object_rot_resp = get_orientation_euler(
                self.model_states[GRASP_OBJECT_NAME]["pose"]
            )
            object_rot = np.array(
                [object_rot_resp.y, object_rot_resp.p, object_rot_resp.r]
            )

            # Get object velocity
            object_velp = (
                object_pos - self._prev_object_pos
            ) / dt  # Velocity(position) = Distance/Time
            object_velr = (
                object_rot - self._prev_object_rot
            ) / dt  # Velocity(rotation) = Rotation/Time

            # Get relative position and velocity
            object_rel_pos = object_pos - ee_pos
            object_velp -= grip_velp
        else:
            object_pos = (
                object_rot
            ) = object_velp = object_velr = object_rel_pos = np.zeros(0)

        # Get achieved goal
        achieved_goal = self._sample_achieved_goal(ee_pos, object_pos)

        # Concatenate observations
        obs = np.concatenate(
            [
                ee_pos,
                object_pos.ravel(),
                object_rel_pos.ravel(),
                ee_state,
                object_rot.ravel(),
                object_velp.ravel(),
                object_velr.ravel(),
                ee_vel,
            ]
        )

        # Save current gripper and object positions
        self._prev_grip_pos = ee_pos
        self._prev_object_pos = object_pos
        self._prev_object_rot = object_rot

        # Return goal env observation dictionary
        return {
            "observation": obs.copy(),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": self.goal.copy(),
            "info": {},
        }

    def _set_action(self, action):
        """Take robot action.

        Parameters
        ----------
        action : list
            List containing joint or ee action commands.
        """

        # Throw error and shutdown if action space is not the right size
        if not action.shape == self.action_space.shape:
            rospy.logerr(
                "Shutting down '%s' since the shape of the supplied action %s while "
                "the gym action space has shape %s."
                % (rospy.get_name(), action.shape, self.action_space.shape)
            )
            sys.exit(0)

        # ensure that we don't change the action outside of this scope
        action = action.copy()

        # Send action commands to the controllers based on control type
        if self._robot_arm_control_type == "ee_control":

            # Create action dictionary
            action_dict = OrderedDict(zip(self._action_space_joints, action))

            # Convert gripper_width command into finger joints commands
            if (
                self._robot_hand_control_type in POSITION_CONTROL_TYPES
                and self._use_gripper_width
            ):
                action_dict = translate_gripper_width_2_finger_joint_commands(
                    action_dict
                )

            # Make sure the gripper is not moved if block_gripper == True
            # Note: When the gripper joint_commands are not present the current gripper
            # state will be used as a setpoint and thus the gripper is locked
            if self._block_gripper:
                for joint in self._controlled_joints["hand"]:
                    action_dict.pop(joint)

            # Print Debug info
            rospy.logdebug("=Action set info=")
            rospy.logdebug("Action that is set:")
            rospy.logdebug(list(action_dict.values()))

            # Split action_dict into hand and arm control_msgs
            arm_action_dict = {
                key: val
                for key, val in action_dict.items()
                if key in ["x", "y", "z", "rx", "ry", "rz", "rw"]
            }
            hand_action_dict = {
                key: val
                for key, val in action_dict.items()
                if key in self._controlled_joints["hand"]
            }

            # Take action
            self.set_ee_pose(arm_action_dict)
            if self._robot_hand_control_type in POSITION_CONTROL_TYPES:
                self.set_joint_positions(hand_action_dict)
            elif EFFORT_CONTROL_TYPES:  # If hand uses effort control
                self.set_joint_efforts(hand_action_dict)
            else:  # If hand uses joint_trajectory control
                hand_traj_msg = action_dict_2_joint_trajectory_msg(hand_action_dict)
                self.set_joint_trajectory(hand_traj_msg)
        else:

            # Create action dictionary
            action_dict = OrderedDict(zip(self._action_space_joints, action))

            # Convert gripper_width command into finger joints commands
            if (
                self._robot_hand_control_type in POSITION_CONTROL_TYPES
                and self._use_gripper_width
            ):
                action_dict = translate_gripper_width_2_finger_joint_commands(
                    action_dict
                )

            # Make sure the gripper is not moved if block_gripper == True
            # Note: When the gripper joint_commands are not present the current gripper
            # state will be used as a setpoint and thus the gripper is locked
            if self._block_gripper:
                for joint in self._controlled_joints["hand"]:
                    action_dict.pop(joint)

            # Print Debug info
            rospy.logdebug("=Action set info=")
            rospy.logdebug("Action that is set:")
            rospy.logdebug(list(action_dict.values()))

            # Take action
            if (
                self._robot_arm_control_type in POSITION_CONTROL_TYPES
                and self._robot_hand_control_type in POSITION_CONTROL_TYPES
            ):
                self.set_joint_positions(action_dict)
            elif (
                self._robot_arm_control_type in EFFORT_CONTROL_TYPES
                and self._robot_hand_control_type in EFFORT_CONTROL_TYPES
            ):
                self.set_joint_efforts(action_dict)
            elif (
                self._robot_arm_control_type == "joint_trajectory_control"
                and self._robot_hand_control_type == "joint_trajectory_control"
            ):
                traj_msg = action_dict_2_joint_trajectory_msg(action_dict)
                self.set_joint_trajectory(traj_msg)
            else:  # If arm and hand have different control types

                # Split action_dict into hand and arm control_msgs
                arm_action_dict = {
                    key: val
                    for key, val in action_dict.items()
                    if key in self._controlled_joints["arm"]
                }
                hand_action_dict = {
                    key: val
                    for key, val in action_dict.items()
                    if key in self._controlled_joints["hand"]
                }

                # Take arm action
                if self._robot_arm_control_type in POSITION_CONTROL_TYPES:
                    self.set_joint_positions(arm_action_dict)
                elif self._robot_arm_control_type in EFFORT_CONTROL_TYPES:
                    self.set_joint_efforts(arm_action_dict)
                else:
                    arm_traj_msg = action_dict_2_joint_trajectory_msg(arm_action_dict)
                    self.set_joint_trajectory(arm_traj_msg)

                # Take hand action
                if self._robot_hand_control_type in POSITION_CONTROL_TYPES:
                    self.set_joint_positions(hand_action_dict)
                elif self._robot_hand_control_type in EFFORT_CONTROL_TYPES:
                    self.set_joint_efforts(hand_action_dict)
                else:
                    hand_traj_msg = action_dict_2_joint_trajectory_msg(hand_action_dict)
                    self.set_joint_trajectory(hand_traj_msg)

    def _is_done(self, observations):
        """Check if task is done.

        Parameters
        ----------
        observations : dict
            Dictionary containing the observations

        Returns
        -------
        bool
            Boolean specifying whether the episode is done (e.i. distance to the goal is
            within the distance threshold, robot has fallen etc.).
        """

        # Check if gripper is within range of the goal
        d = self._goal_distance(observations["achieved_goal"], self.goal)

        # Print Debug info
        rospy.logdebug("=Task is done info=")
        if (d < self._distance_threshold).astype(np.float32):
            rospy.logdebug("Taks is done.")
        else:
            rospy.logdebug("Task is not done.")

        # Return result
        return (d < self._distance_threshold).astype(np.float32)

    def _sample_goal(self):
        """Sample a grasping goal. Sample from objects if environment has object
        otherwise create random goal.

        Returns
        -------
        geometry_msgs.PoseStamped
            A goal pose.

        Raises
        ------
        EePoseLookupError
            Error thrown when error occurred while trying to retrieve the EE pose using
            the 'get_ee_pose' service.
        """

        # Sample goal from goal region based on the target_sampling_strategy
        if self._target_sampling_strategy == "global":

            # Sample goal within the global bounds
            goal = self.np_random.uniform(
                [
                    self._target_bounds["global"]["x_min"],
                    self._target_bounds["global"]["y_min"],
                    self._target_bounds["global"]["z_min"],
                ],
                [
                    self._target_bounds["global"]["x_max"],
                    self._target_bounds["global"]["y_max"],
                    self._target_bounds["global"]["z_max"],
                ],
                size=3,
            )
        elif self._target_sampling_strategy == "local":  # Rel to current EE pose

            # Retrieve current end effector pose
            try:
                cur_ee_pose = self.get_ee_pose()  # Get ee pose
                cur_ee_pos = np.array(
                    [
                        cur_ee_pose.pose.position.x,
                        cur_ee_pose.pose.position.y,
                        cur_ee_pose.pose.position.z,
                    ]
                )  # Retrieve position
            except EePoseLookupError:
                rospy.logerr(
                    "Shutting down '%s' since the current end effector pose "
                    "which is needed for sampling the goals could not be "
                    "retrieved." % (rospy.get_name())
                )
                sys.exit(0)

            # Sample goal relative to end effector pose
            goal = cur_ee_pos + self.np_random.uniform(
                [
                    self._target_bounds["local"]["x_min"],
                    self._target_bounds["local"]["y_min"],
                    self._target_bounds["local"]["z_min"],
                ],
                [
                    self._target_bounds["local"]["x_max"],
                    self._target_bounds["local"]["y_max"],
                    self._target_bounds["local"]["z_max"],
                ],
                size=3,
            )
        else:  # Thrown error if goal could not be sampled
            rospy.logerr(
                "Shutting down '%s' since no goal could be sampled as '%s' is not "
                "a valid goal sampling strategy. Options are 'global' and 'local'."
                % (rospy.get_name(), self._target_sampling_strategy)
            )
            sys.exit(0)

        # Apply offsets if the task environment has an object
        if self._has_object:  # Environment has object

            # Apply target offset (Required in the panda push task)
            goal += [
                self._target_offset["x"],
                self._target_offset["y"],
                self._target_offset["z"],
            ]
            goal[2] = self._object_height_offset

            # If object is in the air add an additional height offset
            if self._target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0, 0.45)

        # Make sure the goal is always within the global goal sampling region
        goal = self._clip_goal_position(goal)

        # Visualize goal marker
        if self._visualize_target:

            # Generate Rviz marker
            goal_maker_pose = Pose()
            goal_maker_pose.position.x = goal[0]
            goal_maker_pose.position.y = goal[1]
            goal_maker_pose.position.z = goal[2]
            goal_marker_msg = TargetMarker(pose=goal_maker_pose)

            # Publish goal marker for rviz visualization
            self._target_pose_pub.publish(goal_marker_msg)

        # return goal.copy()
        return goal

    def _env_setup(self):
        """Sets up initial configuration of the environment. Can be used to configure
        the initial robot state and extract information from the simulation.
        """

        # Move robot joints into their initial position
        self._set_init_pose()

        # Spawn grasp object, set initial pose and calculate the gripper height offset
        if self._has_object:

            # Spawn the object
            rospy.loginfo("Spawning '%s' object." % GRASP_OBJECT_NAME)
            try:
                self._spawn_object(
                    GRASP_OBJECT_NAME, "grasp_cube", pose=self._init_obj_pose
                )
            except SpawnModelError:
                rospy.logerr(
                    "Shutting down '%s' since the grasp object could not be spawned."
                    % (rospy.get_name(),)
                )
                sys.exit(0)

            # Set initial object pose
            self._set_init_obj_pose()

            # Retrieve the object height
            self._object_height_offset = self.model_states[GRASP_OBJECT_NAME][
                "pose"
            ].position.y

        # Store initial EE and qpose
        try:
            cur_ee_pose = self.get_ee_pose()
        except EePoseLookupError:
            rospy.logwarn(
                "Initial EE pose not stored since it could not be retrieved."
                % (rospy.get_name())
            )
        cur_ee_pos = np.array(
            [
                cur_ee_pose.pose.position.x,
                cur_ee_pose.pose.position.y,
                cur_ee_pose.pose.position.z,
            ]
        )
        self.initial_ee_pos = cur_ee_pos.copy()
        try:
            cur_qpose = OrderedDict(
                zip(list(self.joint_states.name), list(self.joint_states.position))
            )
        except EePoseLookupError:
            rospy.logwarn(
                "Initial generalized robot pose (qpose) not stored since it could not "
                "be retrieved." % (rospy.get_name())
            )
        self.initial_qpose = cur_qpose.copy()

        # Sample a reaching goal
        self.goal = self._sample_goal()
        self._get_obs()

    def _set_init_pose(self):
        """Sets the Robot in its init pose.

        Returns
        -------
        bool
            Boolean specifying whether reset was successful.
        """

        # Retrieve initial pose (Random or fixed)
        if (
            self._randomize_first_episode or self.episode_num != 0
        ) and self._random_init_pose:

            # Retrieve random ee pose
            rospy.logdebug("Retrieve random ee_pose.")
            try:

                # Retrieve random ee and gripper pose
                init_ee_pose = self._get_random_ee_pose()
                init_ee_pose = pose_msg_2_pose_dict(init_ee_pose)  # Conv. to pose dict
            except (RandomEePoseError) as e:
                # Change warn message
                logwarn_msg = (
                    "Random ee_pose could not be retrieved as %s. Initial pose used "
                    "instead." % lower_first_char(e.args[0])
                )
                rospy.logwarn(logwarn_msg)
                init_ee_pose, _ = split_pose_dict(self._init_pose)

            # Retrieve random gripper pose
            rospy.logdebug("Retrieve random gripper pose.")
            try:

                # Retrieve random gripper pose
                init_joint_pose = self._get_random_joint_positions()
                init_grip_joints_pose = {
                    key: val
                    for key, val in init_joint_pose.items()
                    if key
                    in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
                }
            except (RandomJointPositionsError) as e:

                # Change warn message
                logwarn_msg = (
                    "Random gripper pose could not be retrieved as %s. Initial pose "
                    "used instead." % lower_first_char(e.args[0])
                )
                rospy.logwarn(logwarn_msg)
                _, init_joint_pose = split_pose_dict(self._init_pose)
                init_grip_joints_pose = {
                    key: val
                    for key, val in init_joint_pose.items()
                    if key
                    in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
                }

        else:

            # Clip init pose within bounds if requested
            init_ee_pose, init_joint_pose = split_pose_dict(self._init_pose)
            if hasattr(self, "init_pose_bounds"):
                init_ee_pose = self._clip_init_pose(init_ee_pose)
                init_joint_pose = self._clip_init_pose(init_joint_pose)
            init_grip_joints_pose = {
                key: val
                for key, val in init_joint_pose.items()
                if key
                in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
            }

        # Put finger joints in the right position if use_gripper_width is True
        if self._use_gripper_width:
            if "gripper_width" not in init_grip_joints_pose.keys():
                rospy.loginfo(
                    "The joint position of 'panda_finger_joint2' was set to be "
                    "be equal to the value of 'panda_finger_joint1' as the "
                    "'use_gripper_width' variable is set to True."
                )
                init_grip_joints_pose["panda_finger_joint2"] = init_grip_joints_pose[
                    "panda_finger_joint1"
                ]
            else:
                init_grip_joints_pose = translate_gripper_width_2_finger_joint_commands(
                    init_grip_joints_pose
                )

        # Log messages
        rospy.loginfo("Setting initial robot pose.")
        rospy.logdebug("Init ee pose:")
        rospy.logdebug(pose_dict_2_pose_msg(init_ee_pose))
        rospy.logdebug("init gripper pose:")
        log_pose_dict(init_grip_joints_pose, header="gripper_pose")

        # Make sure the end effector z position is higher than the extra gripper height
        # 'gripper_extra_height'
        # TODO: Validate wheter this pose is valid with the extra height
        if init_ee_pose["z"] <= self._gripper_extra_height:
            init_ee_pose["z"] = self._gripper_extra_height

        # Set initial pose
        # TODO: Why only wait true for joint_positions
        self.gazebo.unpauseSim()
        ee_pose_retval = self.set_ee_pose(init_ee_pose)
        gripper_pose_retval = self.set_joint_positions(init_grip_joints_pose, wait=True)

        # Throw warning and return result
        if not ee_pose_retval and not gripper_pose_retval:
            rospy.logwarn("Setting initial robot pose failed.")
        elif not ee_pose_retval:
            rospy.logwarn("Setting initial ee pose failed.")
        elif not gripper_pose_retval:
            rospy.logwarn("Setting initial gripper pose failed.")
        return any([ee_pose_retval, gripper_pose_retval])

    def _set_init_obj_pose(self):
        """Sets the grasp object to its initial pose.

        Returns
        -------
        bool
            Success boolean.
        """

        # Set the grasp object pose
        rospy.loginfo("Setting initial object position.")
        if self._has_object:

            # Retrieve x,y positions of the current and initial object pose
            obj_pose = self.model_states[GRASP_OBJECT_NAME]["pose"]
            obj_xy_positions = np.array(
                [
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.x,
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.y,
                ]
            )
            init_obj_xy_positions = np.array(
                [self._init_obj_pose.position.x, self._init_obj_pose.position.y]
            )

            # Sample an object initial object (x, y) position
            # NOTE: This is done relative to the 'init_obj_pose' that is set in the
            # that is supplied through the class constructor
            while (
                np.linalg.norm(
                    np.array([obj_pose.position.x, obj_pose.position.y])
                    - obj_xy_positions
                )
                < self._obj_sampling_distance_threshold
            ):  # Sample till is different enough from the current object pose
                obj_xy_positions = init_obj_xy_positions + self.np_random.uniform(
                    [self._obj_bounds["x_min"], self._obj_bounds["y_min"]],
                    [self._obj_bounds["x_max"], self._obj_bounds["y_max"]],
                    size=2,
                )

            # Set the sampled object x and y positions to the object pose
            obj_pose.position.x = obj_xy_positions[0]
            obj_pose.position.y = obj_xy_positions[1]

            # Set init object pose
            rospy.logdebug("Init object pose:")
            rospy.logdebug(obj_pose)
            try:
                retval = self._set_model_state(GRASP_OBJECT_NAME, obj_pose)
            except SetModelStateError:
                rospy.logerr(
                    "Shutting down '%s' since the state of the grasp object could not "
                    "be set." % (rospy.get_name())
                )
                sys.exit(0)

            # Return result
            if not retval:
                rospy.logwarn("setting initial object position failed.")
            return retval

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        pass

    #############################################
    # Panda Task env extension methods ##########
    #############################################
    # NOTE: Overloads virtual methods that were defined in the Robot and Gazebo Goal
    # environments

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Used
        to enforce additional constraints on the simulation state.
        """

        # Make sure the gripper joint positions are equal to the initial positions
        if self._block_gripper:
            self.set_joint_positions(
                {
                    key: val
                    for key, val in self.initial_qpose.items()
                    if key in ["panda_finger_joint1", "panda_finger_joint2"]
                },
            )

    #############################################
    # Task env helper methods ###################
    #############################################

    def _robot_get_obs(self, data):
        """Returns all joint positions and velocities associated with a robot.

        Parameters
        ----------
        param : sensor_msgs/JointState
            Joint states message.

        Returns
        -------
        numpy.array
           Robot Positions, Robot Velocities
        """

        # Retrieve positions and velocity out of sensor_msgs/JointState msgs
        if data.position is not None and data.name:
            names = [n for n in data.name]
            return (
                np.array([data.position[i] for i in range(len(names))]),
                np.array([data.velocity[i] for i in range(len(names))]),
            )
        else:
            return np.zeros(0), np.zeros(0)

    def _goal_distance(self, goal_a, goal_b):
        """Calculates the perpendicular distance to the goal.

        Parameters
        ----------
        goal_a : numpy.ndarray
            List containing a gripper and object pose.
        goal_b : numpy.ndarray
            List containing a gripper and object pose.

        Returns
        -------
        numpyp.float32
            Perpendicular distance to the goal.
        """
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _sample_achieved_goal(self, ee_pos, object_pos):
        """Retrieve currently achieved goal. If gripper has object return object
        position.

        Parameters
        ----------
        ee_pos : numpy.ndarray
            Gripper position.
        object_pos : numpy.ndarray
            Object position.

        Returns
        -------
        geometry_msgs.PoseStamped
            The achieved pose.
        """

        # Retrieve gripper end pose
        if not self._has_object:  # Environment has no object
            achieved_goal = np.squeeze(ee_pos.copy())
        else:  # Has object
            achieved_goal = np.squeeze(object_pos.copy())

        # return achieved_goal.copy()
        return achieved_goal

    def _get_config(self):
        """Retrieve default values from the defaults configuration file."""

        # Try to load default values from default configuration file
        try:
            with open(ENV_CONFIG_PATH, "r") as stream:
                try:
                    config = yaml.safe_load(stream)
                except yaml.YAMLError as e:
                    rospy.logwarn(
                        "Shutting down '%s' as the task environment configuration "
                        "values could not be loaded from the configuration file '%s' "
                        "as the following error was thrown: %s"
                        % (rospy.get_name(), ENV_CONFIG_PATH, e)
                    )
                    sys.exit(0)
        except FileNotFoundError:
            rospy.logwarn(
                "Shutting down '%s' as the task environment configuration values could "
                "not be loaded since the configuration file '%s' was not found. Please "
                "make sure the configuration file is present."
                % (rospy.get_name(), ENV_CONFIG_PATH)
            )
            sys.exit(0)

        # configuration values that can be overloaded using the constructor
        self._gripper_extra_height = config["simulation"]["control"][
            "gripper_extra_height"
        ]
        self._block_gripper = config["simulation"]["control"]["block_gripper"]
        self._has_object = config["training"]["has_object"]
        self._target_in_the_air = config["training"]["target_sampling"][
            "target_in_the_air"
        ]
        self._target_offset = config["training"]["target_sampling"]["target_offset"]
        self._target_sampling_strategy = config["training"]["target_sampling"][
            "strategy"
        ].lower()
        self._distance_threshold = config["training"]["distance_threshold"]
        self._init_pose = config["simulation"]["init_pose_sampling"]["init_robot_pose"]
        self._init_obj_pose = config["training"]["object_sampling"]["init_obj_pose"]
        self._obj_bounds = config["training"]["object_sampling"]["bounds"]
        self.reward_type = config["training"]["reward_type"].lower()
        self._robot_arm_control_type = config["simulation"]["control"][
            "robot_arm_control_type"
        ].lower()
        self._robot_hand_control_type = config["simulation"]["control"][
            "robot_hand_control_type"
        ].lower()
        self._use_gripper_width = config["action_space"]["use_gripper_width"]

        # Other configuration values
        self._obj_sampling_distance_threshold = config["training"]["object_sampling"][
            "distance_threshold"
        ]
        self._reset_robot_pose = config["simulation"]["reset_robot_pose"]
        self._random_init_pose = config["simulation"]["init_pose_sampling"][
            "random_init_pose"
        ]
        self._randomize_first_episode = config["simulation"]["init_pose_sampling"][
            "randomize_first_episode"
        ]
        self._target_bounds = config["training"]["target_sampling"]["bounds"]
        self._visualize_target = config["training"]["target_sampling"][
            "visualize_target"
        ]
        self._visualize_target_bounds = config["training"]["target_sampling"][
            "visualize_bounds"
        ]
        self._visualize_obj_bounds = config["training"]["object_sampling"][
            "visualize_bounds"
        ]
        self._visualize_init_pose_bounds = config["simulation"]["init_pose_sampling"][
            "visualize_bounds"
        ]
        self._ee_link = config["simulation"]["control"]["ee_link"]
        self._action_bounds = config["action_space"]["bounds"]

    def _get_elapsed_time(self):
        """Returns the elapsed time since the last time this function was called."""
        current_time = rospy.get_time()
        dt = self._sim_time - current_time
        self._sim_time = current_time
        return dt

    def _get_random_joint_positions(self):
        """Get valid joint position commands for the Panda arm and hand.

        Returns
        -------
        dict
            Dictionary containing a valid joint position for each joint.

        Raises
        ------
        RandomJointPositionsError
            Error thrown when 'get_random_joint_positions' service is not available.
        """

        # Create GetRandomJointPositionsRequest message
        req = GetRandomJointPositionsRequest()
        if hasattr(self, "init_pose_bounds"):  # If the user supplied bounding region

            # Retrieve joint positions bounding regions
            _, joint_pose_bound_region = split_bounds_dict(self._init_pose_bounds)

            # Translate gripper_with bounds to finger bounds
            joint_pose_bound_region = translate_gripper_width_2_finger_joint_commands(
                joint_pose_bound_region
            )

            # Add bounding region to GetRandomEePoseRequest message
            joint_limits = JointLimits()
            joint_limits.names = list(joint_pose_bound_region.keys())
            joint_limits.values = list(joint_pose_bound_region.values())
            req.joint_limits = joint_limits
        if self._services_connection_status[MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC]:

            # Request random pose
            resp = self._moveit_get_random_joint_positions_client(req)

            # Convert to joint_position dictionary and return
            if resp.success:
                joint_positions_dict = OrderedDict(
                    zip(resp.joint_names, resp.joint_positions)
                )
                return joint_positions_dict
            else:
                raise RandomJointPositionsError(
                    message=(
                        "A MoveItCommanderException error occurred in the '%s' service "
                        "when trying to retrieve random (valid) joint positions."
                        % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
                    ),
                )
        else:
            raise RandomJointPositionsError(
                message="No random (valid) joint positions could not be retrieved as "
                "the '%s' service is not available."
                % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC,
            )

    def _get_random_ee_pose(self):
        """Get a valid ee pose command for controlling the Panda Arm end effector.

        Returns
        -------
        geometry_msgs.msg.Pose
            Pose message containing a valid ee pose.

        Raises
        ------
        RandomEePoseError
            Error thrown when 'get_random_ee_pose' service is not available.
        """

        # Create GetRandomEePoseRequest message
        req = GetRandomEePoseRequest()
        if hasattr(self, "init_pose_bounds"):  # If the user supplied bounding region

            # Retrieve end effector pose bounding regions
            ee_pose_bound_region, _ = split_bounds_dict(self._init_pose_bounds)

            # Add bounding region to GetRandomEePoseRequest message
            req.bounding_region = BoundingRegion(**ee_pose_bound_region)

        # Get random pose using moveit get_random_pose service
        if self._services_connection_status[MOVEIT_GET_RANDOM_EE_POSE_TOPIC]:

            # Request random pose
            resp = self._moveit_get_random_ee_pose_client(req)

            # Convert to joint_position dictionary and return
            if resp.success:
                return resp.ee_pose
            else:
                raise RandomEePoseError(
                    message=(
                        "A MoveItCommanderException error occurred in the '%s' service "
                        "when trying to retrieve a random (valid) end effector pose."
                        % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
                    ),
                )
        else:
            raise RandomEePoseError(
                message="A random (valid) end effector pose could not be retrieved as "
                "the '%s' service is not available." % MOVEIT_GET_RANDOM_EE_POSE_TOPIC,
            )

    def _clip_goal_position(self, goal_pose):
        """Limit the possible goal position x, y and z values to a certian range.

        Parameters
        ----------
        goal_pose : numpy.ndarray
            A numpy array containing the goal x,y and z values.
        """

        # Clip goal using the goal bounds
        goal_pose[0] = np.clip(
            goal_pose[0],
            self._target_bounds[self._target_sampling_strategy]["x_min"],
            self._target_bounds[self._target_sampling_strategy]["x_max"],
        )
        goal_pose[1] = np.clip(
            goal_pose[1],
            self._target_bounds[self._target_sampling_strategy]["y_min"],
            self._target_bounds[self._target_sampling_strategy]["y_max"],
        )
        goal_pose[2] = np.clip(
            goal_pose[2],
            self._target_bounds[self._target_sampling_strategy]["z_min"],
            self._target_bounds[self._target_sampling_strategy]["z_max"],
        )

        # Return goal
        return goal_pose

    def _clip_init_pose(self, init_pose):
        """Limit the possible initial end effector (EE) and Robot joint positions to
        a certain range.

        Parameters
        ----------
        init_pose : dict
            A dictionary containing the init pose values for each of the Robot or
            end effector joints.
        """

        # Clip goal
        if hasattr(self, "init_pose_bounds"):
            init_pose_clipped = {}
            for key, val in init_pose.items():
                try:
                    init_pose_clipped[key] = np.clip(
                        val,
                        self._init_pose_bounds[key + "_min"],
                        self._init_pose_bounds[key + "_max"],
                    )
                except KeyError:
                    init_pose_clipped[key] = val
        else:
            rospy.logwarn(
                "Init pose could not be clipped since no 'init_pose_bounds' were given."
            )
            init_pose_clipped = init_pose

        # Return goal
        return init_pose_clipped

    def _get_controlled_joints(self):
        """Get the joints that can be controlled when using the current Panda arm and
        hand control types.

        Returns
        -------
        dict
            Arm and Hand joints that are available for control.
        """

        # Retrieve all the joints that can be controlled by the control types
        controlled_joints = {}
        resp_arm = self._moveit_get_controlled_joints_client(
            GetControlledJointsRequest(control_type=self._robot_arm_control_type)
        )
        resp_hand = self._moveit_get_controlled_joints_client(
            GetControlledJointsRequest(control_type=self._robot_hand_control_type)
        )
        controlled_joints["arm"] = (
            resp_arm.controlled_joints_arm
            if resp_arm.success
            else self._panda_joints["arm"]
        )
        controlled_joints["hand"] = (
            resp_hand.controlled_joints_hand
            if resp_hand.success
            else self._panda_joints["hand"]
        )
        controlled_joints["both"] = (
            flatten_list([controlled_joints["arm"], controlled_joints["hand"]])
            if self.joint_states.name[0] in controlled_joints["arm"]
            else flatten_list([controlled_joints["hand"], controlled_joints["arm"]])
        )

        # Return currently controlled joints
        return controlled_joints

    def _validate_action_space_size(self):
        """Checks if the action space size 'n_actions' is valid and adjust it if this
        is not the case.
        """

        # Calculate the maximum action space size
        if self._robot_arm_control_type == "ee_control":

            # Retrieve max action space and create action space description string
            if (
                self._robot_hand_control_type in POSITION_CONTROL_TYPES
            ):  # Hand position control
                max_action_space = 9 if not self._use_gripper_width else 8
                action_space_description = (
                    "EE position(x,y,z), EE-orientation(x,y,z,w), %s"
                    % (
                        "panda_finger_joint1 position, panda_finger_joint2 position"
                        if not self._use_gripper_width
                        else "gripper_width"
                    )
                )
            else:  # Other hand control types
                max_action_space = 9
                action_space_description = (
                    "EE position(x,y,z), EE-orientation(x,y,z,w), panda_finger_joint1 "
                    "effort, panda_finger_joint2 effort"
                )
        else:  # Other control types

            # Retrieve max action space
            if self._robot_hand_control_type in POSITION_CONTROL_TYPES:
                max_action_space = (
                    len(self.joint_states.name)
                    if not self._use_gripper_width
                    else len(self.joint_states.name) - 1
                )
                action_space_description = (
                    (
                        " position, ".join(
                            map(
                                str,
                                [
                                    item
                                    for item in self.joint_states.name
                                    if item
                                    not in [
                                        "panda_finger_joint1",
                                        "panda_finger_joint2",
                                    ]
                                ],
                            )
                        )
                        + " position, gripper_width"
                        if self._use_gripper_width
                        else " position, ".join(map(str, self.joint_states.name))
                        + " position"
                    )
                    if self._use_gripper_width
                    else str(self.joint_states.name)[1:-1].replace("'", "")
                )
            else:
                max_action_space = len(self.joint_states.name)
                action_space_description = (
                    str(self.joint_states.name)[1:-1]
                    .replace("'", "")
                    .replace(",", " effort,")
                    + " effort,"
                    + ("panda_finger_joint1 effort, " "panda_finger_joint2 effort")
                )

        # Adjust n_actions based on the action_space_joints
        if not hasattr(self, "n_actions") and not self._action_space_joints:
            self._n_actions = max_action_space  # Use max action space
        elif not hasattr(
            self, "n_actions"
        ):  # If only action space joints were supplied
            self._n_actions = len(self._action_space_joints)  # Use action space joints
        else:  # If both were supplied

            # Check if action space size 'n_actions' is greater than 1 and equal to the
            # size of the 'action_space_joints' argument.
            if self._n_actions <= 0:
                if len(self._action_space_joints) > 0:
                    rospy.logwarn(
                        "Action space size 'n_actions' can not be smaller than 1. The "
                        "size of the 'action_space_joints' argument (%s) will be used "
                        "instead." % len(self._action_space_joints)
                    )
                    self._n_actions = len(self._action_space_joints)
                else:
                    rospy.logerr(
                        "Shutting down '%s' since the action space size 'n_actions' "
                        "can not be smaller than 1." % rospy.get_name()
                    )
                    sys.exit(0)
            else:

                # Verify whether 'action_space_joints' is equal to 'n_actions' and
                # adjust 'n_actions' when this if this is not the case.
                if self._action_space_joints:  # If action_space_joints were given
                    if len(self._action_space_joints) != self._n_actions:
                        rospy.logwarn(
                            "The length of the 'action_space_joints' argument (%s) is "
                            "not equal to the requested action space size 'n_actions' "
                            "(%s). As a result the action space size 'n_actions' was "
                            "adjusted to the size of the 'action_space_joints' "
                            "argument (%s)."
                            % (
                                len(self._action_space_joints),
                                self._n_actions,
                                len(self._action_space_joints),
                            )
                        )
                        self._n_actions = len(self._action_space_joints)

        # If actionspace is to big throw warning and adjust action size
        if self._n_actions > max_action_space:

            # Log warning message
            rospy.logwarn(
                "An action space of size %s was specified while the maximum size "
                "of the action space while using '%s' for the arm and '%s' "
                "for the hand is %s %s([%s]). Because of this an action space of "
                "%s will be used during training."
                % (
                    self._n_actions,
                    self._robot_arm_control_type,
                    self._robot_hand_control_type,
                    max_action_space,
                    "when 'gripper_width' is used " if self._use_gripper_width else "",
                    action_space_description,
                    max_action_space,
                )
            )

            # Set actionspace to max_actionspace
            self._n_actions = max_action_space

    def _process_input_arguments(
        self,
        gripper_extra_height,
        block_gripper,
        has_object,
        target_in_the_air,
        target_sampling_strategy,
        target_offset,
        target_bounds,
        distance_threshold,
        init_pose,
        init_pose_bounds,
        init_obj_pose,
        obj_bounds,
        reward_type,
        robot_arm_control_type,
        robot_hand_control_type,
        n_actions,
        use_gripper_width,
        action_space_joints,
    ):
        """Function used for processing the constructor input arguments. It validates
        whether the input arguments have the right type, value and/or contain the right
        keys. If this is the case they get added as class attributes otherwise an error
        is thrown.

        Parameters
        ----------
        reward_type : str
            The reward type, i.e. sparse or dense.
        distance_threshold : float
            The threshold after which a goal is considered achieved.
        has_object : bool
            Whether or not the environment has an object.
        block_gripper : bool
            Whether or not the gripper is blocked (i.e. not movable) or not, by default
            None.
        target_in_the_air : bool
            Whether or not the target should be in the air above the table or on the
            table surface.
        gripper_extra_height : float
            Additional height above the table when positioning the gripper.
        target_offset : dict
            A dictionary in which the offset of the target is defined i.e. {x,y,z}, by
            default None.
        target_bounds : dict
            A dictionary with the bounds from which the target is sampled i.e.
            ``{x_min, y_min, z_min, x_max, y_max, x_max}``.
        target_sampling_strategy : str
            Whether the target bounds from which we sample the target goal position are
            relative to the global frame 'global' or relative to the current
            end-effector pose 'local'.
        init_pose : dict
            A dictionary of names and values that define the initial configuration i.e.
            ``{x, y, z, rx, ry, rz, rw, panda_finger_joint1, panda_fingerjoint_2}``, by
            default None.
        init_pose_bounds : dict
            A dictionary with the bounds from which the initial robot pose is sampled
            i.e. ``{x_min, y_min, z_min, x_max, y_max, x_max}``.
        init_obj_pose : dict
            A dictionary that contains the initial object pose i.e.
            ``{x, y, z, rx, ry, rz, rw}``. The object will be spawned
            relative to this pose in a region defined by the obj_bounds.
        obj_bounds : dict
            A dictionary in which the bounds for sampling the object
            positions is defined {x_min, y_min, x_max, y_max}. These
            bounds are relative to the init_obj_pose.
        n_actions : int
            The size of the action space you want to use. When the 'action_space_joints'
            variable is supplied this variable is ignored and the length of the
            controlled joints variable is used as the action space size, by default
            None.
        use_gripper_width : bool
            Whether you want to use the gripper_width instead of the individual panda
            finger joint commands during the training. Consequently,
            using the gripper width reduces the action space by 1.
        action_space_joints : list
            A list containing the joints which you want to use in the action space.
            If this variable is supplied the length of the action space will be set
            to be equal to the length of the 'action_space_joints' list.
        robot_arm_control_type : str
            The type of control you want to use for the robot arm. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``.
        robot_hand_control_type : str
            The type of control you want to use for the robot hand. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``.
        """

        # Overload configuration with input argument if supplied
        if gripper_extra_height:
            self._gripper_extra_height = gripper_extra_height
        if block_gripper is not None:
            self._block_gripper = block_gripper
        if has_object is not None:
            self._has_object = has_object
        if target_in_the_air is not None:
            self._target_in_the_air = target_in_the_air
        if target_sampling_strategy:
            self._target_sampling_strategy = target_sampling_strategy.lower()

            # Validate target_sampling strategy
            # NOTE: Placed here since target bounds variable depends on it
            valid_types = (str,)
            valid_values = TARGET_SAMPLING_STRATEGIES
            retval, depth, invalid_types = has_invalid_type(
                self._target_sampling_strategy, variable_types=valid_types
            )
            if retval:  # Validate type
                arg_type_error(
                    "target_sampling_strategy",
                    depth,
                    invalid_types,
                    valid_types,
                )
            retval, invalid_values = has_invalid_value(
                self._target_sampling_strategy, valid_values=valid_values
            )
            if retval:  # Validate values
                arg_value_error(
                    "target_sampling_strategy",
                    invalid_values=invalid_values,
                    valid_values=valid_values,
                )
        if target_offset:

            # If list convert to dictionary
            if isinstance(target_offset, list):
                self._target_offset = {
                    "x": target_offset[0],
                    "y": target_offset[1],
                    "z": target_offset[2],
                }
        if target_bounds:
            self._target_bounds[self._target_sampling_strategy] = target_bounds
        if distance_threshold:
            self._distance_threshold = distance_threshold
        if init_pose:
            self._init_pose = init_pose
        if init_pose_bounds:
            self._init_pose_bounds = init_pose_bounds
        if init_obj_pose:
            self._init_obj_pose = init_obj_pose
        if obj_bounds:
            self._obj_bounds = obj_bounds
        if reward_type:
            self.reward_type = reward_type.lower()
        if robot_arm_control_type:
            self._robot_arm_control_type = robot_arm_control_type.lower()
        if robot_hand_control_type:
            self._robot_hand_control_type = robot_hand_control_type.lower()
        if n_actions:
            self._n_actions = n_actions
        if use_gripper_width is not None:
            self._use_gripper_width = use_gripper_width
        if action_space_joints:
            self._action_space_joints = action_space_joints

            # Change use_gripper_width based on action_space_joints
            if not all(
                [
                    item
                    in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
                    for item in action_space_joints
                ]
            ):  # If NOT both finger joints and 'gripper_with' are given
                if any(
                    [
                        item in ["panda_finger_joint1", "panda_finger_joint2"]
                        for item in action_space_joints
                    ]
                ):  # If contains finger joints
                    rospy.logwarn(
                        "'use_gripper_width' argument set to False since the "
                        "'action_space_joints' argument contains finger joints instead "
                        "of 'gripper_width'."
                    )
                    self._use_gripper_width = False
                elif (
                    "gripper_width" in action_space_joints
                ):  # If contains gripper_width
                    rospy.logwarn(
                        "'use_gripper_width' argument set to True since the "
                        "'action_space_joints' argument contains 'gripper_with' "
                        "instead of finger joints."
                    )
                    self._use_gripper_width = True
        else:
            self._action_space_joints = []  # Initialize as list

        # Validate input arguments
        self._validate_input_args()

        # Convert init pose and init pose bounds dictionaries into the right format
        self._init_obj_pose = pose_dict_2_pose_msg(self._init_obj_pose)
        self._init_pose = translate_gripper_width_2_finger_joint_commands(
            self._init_pose
        )
        if hasattr(self, "init_pose_bounds"):
            self._init_pose_bounds = translate_gripper_width_2_finger_joint_commands(
                self._init_pose_bounds
            )

    def _validate_action_space_joints(self):
        """Checks whether the joints in the ``action_space_joints`` arguments are valid.
        If this is not the case a ROS error will be thrown and the node will be
        shutdown.
        """

        # Validate if the number of action_space_joints is equal to the action space
        if self._n_actions != len(self._action_space_joints):
            rospy.logerr(
                "Shutting down '%s' since the action space size 'n_actions' (%s) is "
                "unequal to the number of joints that was specified in the "
                "'action_space_joints' argument (%s). Please only supply one of these "
                "arguments or make sure that their size is equal."
                % (
                    rospy.get_name(),
                    len(self._n_actions),
                    len(self._action_space_joints),
                )
            )
            sys.exit(0)

        # Validate if the joints in the action_space_joints list exist
        invalid_joints = []
        for joint in self._action_space_joints:

            # Check if joint_name is vallid based on the control type
            if self._robot_arm_control_type == "ee_control":  # arm end-effector control
                if (
                    self._robot_hand_control_type in POSITION_CONTROL_TYPES
                ):  # Hand position control
                    if self._use_gripper_width:  # If gripper_width is used
                        valid_joints = [
                            "x",
                            "y",
                            "z",
                            "rx",
                            "ry",
                            "rz",
                            "rw",
                            "gripper_width",
                        ]
                        if joint not in flatten_list(valid_joints):
                            invalid_joints.append(joint)
                    else:
                        valid_joints = [
                            "x",
                            "y",
                            "z",
                            "rx",
                            "ry",
                            "rz",
                            "rw",
                            "panda_finger_joint1",
                            "panda_finger_joint2",
                        ]
                        if joint not in flatten_list(valid_joints):
                            invalid_joints.append(joint)
                else:  # All other hand control types
                    valid_joints = [
                        "x",
                        "y",
                        "z",
                        "rx",
                        "ry",
                        "rz",
                        "rw",
                        "panda_finger_joint1",
                        "panda_finger_joint2",
                    ]
                    if joint not in flatten_list(valid_joints):
                        invalid_joints.append(joint)
            else:  # All other arm control types
                if (
                    self._robot_hand_control_type in POSITION_CONTROL_TYPES
                ):  # Hand position control
                    if self._use_gripper_width:  # If gripper_width is used
                        valid_joints = [self.joint_states.name, "gripper_width"]
                        if joint not in flatten_list(valid_joints):
                            invalid_joints.append(joint)
                    else:
                        valid_joints = [
                            self.joint_states.name,
                            "panda_finger_joint1",
                            "panda_finger_joint2",
                        ]
                        if joint not in flatten_list(valid_joints):
                            invalid_joints.append(joint)
                else:
                    valid_joints = [
                        self.joint_states.name,
                        "panda_finger_joint1",
                        "panda_finger_joint2",
                    ]
                    if joint not in flatten_list(valid_joints):
                        invalid_joints.append(joint)

        # Throw error and shutdown node if invalid joint was found
        if invalid_joints:
            rospy.logerr(
                "Shutting down '%s' since the %s %s in the 'action_space_joints' "
                "%s when using '%s' for the arm and '%s' for the hand. Valid joints "
                "are: %s."
                % (
                    rospy.get_name(),
                    list_2_human_text(
                        ["'" + item + "'" for item in invalid_joints],
                        end_seperator="and",
                    ),
                    "joint that was specified"
                    if len(invalid_joints) == 1
                    else "joints that were specified",
                    "is not a valid" if len(invalid_joints) == 1 else "are not valid ",
                    self._robot_arm_control_type,
                    self._robot_hand_control_type,
                    str(flatten_list(valid_joints)),
                )
            )
            sys.exit(0)

    def _get_action_space_joints(self):
        """Retrieves the joints that are being controlled when we sample from the action
        space.

        Returns
        -------
        list
            Joints that are controlled.
        """

        # Retrieve controlled joints given the action space size and control_type
        if self._robot_arm_control_type == "ee_control":
            if (
                self._robot_hand_control_type in POSITION_CONTROL_TYPES
            ):  # If hand is not position controlled check if gripper_width is used
                if self._use_gripper_width:

                    # Replace gripper_width item with panda finger joints
                    action_space_joints = flatten_list(
                        [
                            ["x", "y", "z", "rx", "ry", "rz", "rw"][
                                0 : (self._n_actions - 1)
                            ],
                            "gripper_width",
                        ]
                    )
                else:
                    action_space_joints = flatten_list(
                        [
                            ["x", "y", "z", "rx", "ry", "rz", "rw"],
                            self._controlled_joints["hand"],
                        ]
                    )[0 : self._n_actions]
            else:  # If hand is not position controlled
                action_space_joints = flatten_list(
                    [
                        ["x", "y", "z", "rx", "ry", "rz", "rw"],
                        self._controlled_joints["hand"],
                    ]
                )[0 : self._n_actions]
        else:  # All other control types
            if (
                self._robot_hand_control_type in POSITION_CONTROL_TYPES
            ):  # If hand is not position controlled check if gripper_width is used
                if self._use_gripper_width:

                    # Replace gripper_width item with panda finger joints
                    action_space_joints = flatten_list(
                        [
                            "gripper_width",
                            self._controlled_joints["arm"][0 : (self._n_actions - 1)],
                        ]
                        if self._controlled_joints["both"][0]
                        in self._controlled_joints["hand"]
                        else [
                            self._controlled_joints["arm"][0 : (self._n_actions - 1)],
                            "gripper_width",
                        ]
                    )
                else:
                    action_space_joints = self._controlled_joints["both"][
                        0 : self._n_actions
                    ]
            else:  # If hand is not position controlled
                action_space_joints = self._controlled_joints["both"][
                    0 : self._n_actions
                ]

        # Return action space joints
        return action_space_joints

    def _create_action_space(self):
        """Create the action space based on the action space size and the action bounds.

        Returns
        -------
        gym.spaces.Box
            The gym action space.
        """

        # Retrieve action space bounds based on the chosen control_types and the given
        # action_space_joints

        # Get lower bounds
        arm_bounds_key = (
            "ee_pose"
            if self._robot_arm_control_type == "ee_control"
            else (
                "joint_efforts"
                if self._robot_arm_control_type
                in ["joint_effort_control", "joint_group_effort_control"]
                else "joint_positions"
            )
        )
        hand_bounds_key = (
            "joint_efforts"
            if self._robot_hand_control_type
            in ["joint_effort_control", "joint_group_effort_control"]
            else "joint_positions"
        )
        arm_action_bounds_low = {
            joint: val
            for joint, val in self._action_bounds[arm_bounds_key]["low"].items()
        }
        hand_action_bounds_low = {
            joint: val
            for joint, val in self._action_bounds[hand_bounds_key]["low"].items()
        }
        action_bounds_low = merge_two_dicts(
            arm_action_bounds_low, hand_action_bounds_low
        )
        action_bound_low_filtered = np.array(
            [
                val
                for key, val in action_bounds_low.items()
                if key in self._action_space_joints
            ]
        )

        # Get upper bounds
        arm_action_bounds_high = {
            joint: val
            for joint, val in self._action_bounds[arm_bounds_key]["high"].items()
        }
        hand_action_bounds_high = {
            joint: val
            for joint, val in self._action_bounds[hand_bounds_key]["high"].items()
        }
        action_bounds_high = merge_two_dicts(
            arm_action_bounds_high, hand_action_bounds_high
        )
        action_bound_high_filtered = np.array(
            [
                val
                for key, val in action_bounds_high.items()
                if key in self._action_space_joints
            ]
        )

        # Create action space based on action space size and bounds
        return spaces.Box(
            action_bound_low_filtered,
            action_bound_high_filtered,
            shape=(self._n_actions,),
            dtype="float32",
        )

    def _validate_input_args(self):
        """Checks wether the input arguments are valid. If this is not the case a ROS error
        will be thrown and the node will be shutdown.
        """

        # Gripper_extra_height
        valid_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self._gripper_extra_height, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "gripper_extra_height",
                depth,
                invalid_types,
                valid_types,
            )

        # Block_gripper
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self._block_gripper, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "block_gripper",
                depth,
                invalid_types,
                valid_types,
            )

        # Has_object
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self._has_object, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "has_object",
                depth,
                invalid_types,
                valid_types,
            )

        # Target in the air
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self._target_in_the_air, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "target_in_the_air",
                depth,
                invalid_types,
                valid_types,
            )

        # Target offset
        valid_types = dict
        valid_items_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self._target_offset,
            variable_types=valid_types,
        )
        if retval:  # Validate type
            arg_type_error(
                "target_offset",
                depth,
                invalid_types,
                valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self._target_offset,
            required_keys=["x", "y", "z"],
            exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "target_offset",
                missing_keys=missing_keys,
                extra_keys=extra_keys,
            )

        # Target sampling strategy
        valid_types = (str,)
        valid_values = GOAL_SAMPLING_STRATEGIES
        retval, depth, invalid_types = has_invalid_type(
            self._target_sampling_strategy, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "target_sampling_strategy",
                depth,
                invalid_types,
                valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self._target_sampling_strategy, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "target_sampling_strategy",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Target bounds
        valid_types = (dict,)
        valid_items_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self._target_bounds[self._target_sampling_strategy],
            variable_types=valid_types,
        )
        if retval:  # Validate type
            arg_type_error(
                "target_bounds",
                depth,
                invalid_types,
                valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self._target_bounds[self._target_sampling_strategy],
            required_keys=["x_min", "y_min", "z_min", "x_max", "y_max", "z_max"],
            exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "target_bounds",
                missing_keys=missing_keys,
                extra_keys=extra_keys,
            )

        # Distance threshold
        valid_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self._distance_threshold, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "distance_threshold",
                depth,
                invalid_types,
                valid_types,
            )

        # Init pose
        valid_types = (dict,)
        valid_items_types = (float, int)
        required_keys = [
            "x",
            "y",
            "z",
            "rx",
            "ry",
            "rz",
            "rw",
            ["gripper_width", "panda_finger_joint1"],
            ["gripper_width", "panda_finger_joint2"],
        ]
        retval, depth, invalid_types = has_invalid_type(
            self._init_pose, variable_types=valid_types, items_types=valid_items_types
        )
        if retval:  # Validate type
            arg_type_error(
                "init_pose",
                depth,
                invalid_types,
                valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self._init_pose,
            required_keys=required_keys,
            exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "init_pose",
                missing_keys=missing_keys,
                extra_keys=extra_keys,
            )

        # Init pose bounds
        if hasattr(self, "init_pose_bounds"):  # If init_pose_bounds were supplied
            valid_types = (dict,)
            valid_items_types = (float, int)
            required_keys = [
                "x_min",
                "y_min",
                "z_min",
                "x_max",
                "y_max",
                "z_max",
                ["gripper_width_min", "panda_finger_joint1_min"],
                ["gripper_width_min", "panda_finger_joint2_min"],
                ["gripper_width_max", "panda_finger_joint1_max"],
                ["gripper_width_max", "panda_finger_joint2_max"],
            ]
            retval, depth, invalid_types = has_invalid_type(
                self._init_pose_bounds,
                variable_types=valid_types,
                items_types=valid_items_types,
            )
            if retval:  # Validate type
                arg_type_error(
                    "init_pose_bounds",
                    depth,
                    invalid_types,
                    valid_types,
                )
            retval, missing_keys, extra_keys = contains_keys(
                self._init_pose_bounds,
                required_keys=required_keys,
                exclusive=True,
            )
            if not retval:  # Validate keys
                arg_keys_error(
                    "init_pose_bounds",
                    missing_keys=missing_keys,
                    extra_keys=extra_keys,
                )

        # Init obj pose
        valid_types = (dict,)
        valid_items_types = (float, int)
        required_keys = ["x", "y", "z", "rx", "ry", "rz", "rw"]
        retval, depth, invalid_types = has_invalid_type(
            self._init_obj_pose,
            variable_types=valid_types,
            items_types=valid_items_types,
        )
        if retval:  # Validate type
            arg_type_error(
                "init_obj_pose",
                depth,
                invalid_types,
                valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self._init_obj_pose,
            required_keys=required_keys,
            exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "init_obj_pose",
                missing_keys=missing_keys,
                extra_keys=extra_keys,
            )

        # Object bounds
        valid_types = (dict,)
        valid_items_types = (float, int)
        required_keys = ["x_min", "y_min", "x_max", "y_max"]
        retval, depth, invalid_types = has_invalid_type(
            self._obj_bounds, variable_types=valid_types, items_types=valid_items_types
        )
        if retval:  # Validate type
            arg_type_error(
                "obj_bounds",
                depth,
                invalid_types,
                valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self._obj_bounds,
            required_keys=required_keys,
            exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "obj_bounds",
                missing_keys=missing_keys,
                extra_keys=extra_keys,
            )

        # Reward type
        valid_types = (str,)
        valid_values = REWARD_TYPES
        retval, depth, invalid_types = has_invalid_type(
            self.reward_type, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "reward_type",
                depth,
                invalid_types,
                valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self.reward_type, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "reward_type",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Robot arm control type
        valid_types = (str,)
        valid_values = self._robot_control_types["arm"]
        retval, depth, invalid_types = has_invalid_type(
            self._robot_arm_control_type, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "robot_arm_control_type",
                depth,
                invalid_types,
                valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self._robot_arm_control_type, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "robot_arm_control_type",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Robot hand control type
        valid_types = (str,)
        valid_values = self._robot_control_types["hand"]
        retval, depth, invalid_types = has_invalid_type(
            self._robot_hand_control_type, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "robot_hand_control_type",
                depth,
                invalid_types,
                valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self._robot_hand_control_type, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "robot_hand_control_type",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Robot n actions
        if hasattr(self, "n_actions"):
            valid_types = (int,)
            retval, depth, invalid_types = has_invalid_type(
                self._n_actions, variable_types=valid_types
            )
            if retval:  # Validate type
                arg_type_error(
                    "n_actions",
                    depth,
                    invalid_types,
                    valid_types,
                )

        # Action space joints
        # NOTE: The joint names in the action_Space_joints variable are validated at
        # the end of the constructor
        valid_types = (list,)
        retval, depth, invalid_types = has_invalid_type(
            self._action_space_joints, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "action_space_joints",
                depth,
                invalid_types,
                valid_types,
            )

        # Use gripper width
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self._use_gripper_width, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "use_gripper_width",
                depth,
                invalid_types,
                valid_types,
            )
