"""This module contains the Robot Gazebo Goal environment class that is responsible
for creating the link between the Gazebo simulator and the openAI package.
"""

# Main python imports
import os
import gym
from gym.utils import seeding

from panda_openai_sim.functions import (
    lower_first_char,
    model_state_msg_2_link_state_dict,
    find_gazebo_model_path,
)
from panda_openai_sim.extras import Quaternion
from panda_openai_sim.exceptions import SpawnModelError, SetModelStateError

# ROS python imports
import rospy
from openai_ros.gazebo_connection import GazeboConnection
from openai_ros.controllers_connection import ControllersConnection
from rospy.exceptions import ROSException, ROSInterruptException
from genpy.message import SerializationError

# ROS msgs and srvs
from panda_openai_sim.msg import RLExperimentInfo
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import (
    SpawnModel,
    SpawnModelRequest,
    SetModelState,
    SetModelStateRequest,
)

# Script Parameters
GAZEBO_SPAWN_SDF_MODEL_TOPIC = "/gazebo/spawn_sdf_model"
GAZEBO_SPAWN_URDF_MODEL_TOPIC = "/gazebo/spawn_urdf_model"
GAZEBO_SET_MODEL_STATE_TOPIC = "/gazebo/set_model_state"
GAZEBO_MODEL_STATES_TOPIC = "/gazebo/model_states"
GAZEBO_LINK_STATES_TOPIC = "/gazebo/link_states"
DIRNAME = os.path.dirname(__file__)
GAZEBO_MODELS_FOLDER_PATH = os.path.abspath(
    os.path.join(DIRNAME, "../../../resources/models")
)


#################################################
# Panda Robot Gazebo Environment Class ##########
#################################################
class RobotGazeboGoalEnv(gym.GoalEnv):
    """Class responsible for the communication between Gazebo and the openai gym
    environment.

    Attributes
    ----------
    gazebo : openai_ros.gazebo_connection.GazeboConnection
        The openai_ros Gazebo connection object.
    reset_robot_pose : bool
        Whether to reset the robot pose when the environment is reset.
    reset_controls : bool
        Boolean specifying whether to reset the controllers when the simulation
        is reset.
    controllers_object : openai_ros.controllers_connection.ControllersConnection
        The openai_ros controller manager object.
    episode_num : str
        The episode number.
    link_states : dict
        The current Gazebo link_states.
    model_states : dict
        The current Gazebo model_states.

    Methods
    ----------
    close():
        Function executed when closing the environment.
    reset():
        Function used for resetting the simulation.
    seed(seed=None):
        Function used for generating a random gym seed.
    step(action):
        Function executed each time step.
    np_random(low, high, size):
        Draw samples from a uniform distribution.
    """

    def __init__(
        self, robot_name_space, reset_robot_pose, reset_controls, reset_control_list
    ):
        # TODO: Check what happends in original gym environment with already setting environment
        """Initializes a new Panda Robot Gazebo Goal environment.

        Parameters
        ----------
        robot_name_space : str
            Namespace of the robot.
        reset_robot_pose : bool
            Boolean specifying whether to reset the robot pose when the simulation is
            reset.
        reset_controls : bool
            Boolean specifying whether to reset the controllers when the simulation
            is reset.
        reset_control_list : numpy.ndarray
            Names of the controllers of the robot.
        """

        # To reset Simulations
        rospy.loginfo("Initializing Panda RobotGazeboGoal environment.")
        self.gazebo = GazeboConnection(
            start_init_physics_parameters=False, reset_world_or_sim="WORLD"
        )
        self.reset_robot_pose = reset_robot_pose
        self.reset_controls = reset_controls
        self.controllers_object = ControllersConnection(
            namespace=robot_name_space, controllers_list=reset_control_list
        )
        rospy.logdebug("reset_robot_controls: %s" % self.reset_controls)
        self.seed()

        # Set up ROS related variables
        self.episode_num = 0
        self._reward_pub = rospy.Publisher(
            "/openai/reward", RLExperimentInfo, queue_size=10
        )

        #########################################
        # Connect Gazebo services ###############
        #########################################

        # Connect to Gazebo sdf and urdf spawn model services
        GAZEBO_SPAWN_SDF_MODEL_TOPIC = "/gazebo/spawn_sdf_model"
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_SPAWN_SDF_MODEL_TOPIC)
            rospy.wait_for_service(
                GAZEBO_SPAWN_SDF_MODEL_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._gazebo_spawn_sdf_model_client = rospy.ServiceProxy(
                GAZEBO_SPAWN_SDF_MODEL_TOPIC, SpawnModel
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_SPAWN_SDF_MODEL_TOPIC)
            self._services_connection_status[GAZEBO_SPAWN_SDF_MODEL_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_SPAWN_SDF_MODEL_TOPIC
            )
            self._services_connection_status[GAZEBO_SPAWN_SDF_MODEL_TOPIC] = False
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % GAZEBO_SPAWN_URDF_MODEL_TOPIC
            )
            rospy.wait_for_service(
                GAZEBO_SPAWN_URDF_MODEL_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._gazebo_spawn_urdf_model_client = rospy.ServiceProxy(
                GAZEBO_SPAWN_URDF_MODEL_TOPIC, SpawnModel
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_SPAWN_URDF_MODEL_TOPIC)
            self._services_connection_status[GAZEBO_SPAWN_URDF_MODEL_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_SPAWN_URDF_MODEL_TOPIC
            )
            self._services_connection_status[GAZEBO_SPAWN_URDF_MODEL_TOPIC] = False

        # Connect to Gazebo 'set_model_state' service
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_SET_MODEL_STATE_TOPIC)
            rospy.wait_for_service(
                GAZEBO_SET_MODEL_STATE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._gazebo_set_model_state_client = rospy.ServiceProxy(
                GAZEBO_SET_MODEL_STATE_TOPIC, SetModelState
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_SET_MODEL_STATE_TOPIC)
            self._services_connection_status[GAZEBO_SET_MODEL_STATE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_SET_MODEL_STATE_TOPIC
            )
            self._services_connection_status[GAZEBO_SET_MODEL_STATE_TOPIC] = False

        #########################################
        # Unpause sim and reset controllers #####
        #########################################

        # Unpause the simulation and reset the controllers if needed
        # NOTE: To check any topic we need to have the simulations running, we need to
        # do two things:
        # 1) Unpause the simulation: without that th stream of data doesn't flow. This
        # is for simulations that are pause for whatever the reason.
        # 2) If the simulation was running already for some reason, we need to reset
        # the controllers.
        # This has to do with the fact that some plugins with tf, don't understand the
        # reset of the simulation and need to be reset to work properly.
        self.gazebo.unpauseSim()
        if self.reset_controls:
            self.controllers_object.reset_controllers()

        # Environment initiation complete message
        rospy.loginfo("Panda RobotGazeboGoal environment initialized.")

    #############################################
    # Panda Robot Goal env main methods #########
    #############################################
    def seed(self, seed=None):
        """Create gym random seed.

        Parameters
        ----------
        seed : int, optional
            Random seed, by default None (seeds from an operating system
            specific randomness source), by default None.

        Returns
        -------
        list
            List containing an opengym random seed (randomstate, seed).
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # TODO: Check what happends in the original step function with is_success
        """Function executed each time step.
        Here we get the action execute it in a time step and retrieve the
        observations generated by that action. Here a action num is converted
        to a movement action, executed in the simulation and after which
        the observations result of performing that action is returned.

        Parameters
        ----------
        action :
            The the robot has to perform.

        Returns
        -------
            obs, reward, done, info :
                The step observation, setp reward, whether the task is done and some
                additional debug info.
        """

        # Execute action in the simulation
        rospy.loginfo("Taking step.")
        rospy.logdebug("Unpause sim.")
        self.gazebo.unpauseSim()
        rospy.logdebug("Set action.")
        rospy.logdebug("Action: %s" % action)
        self._set_action(action)
        self._step_callback()

        # Retrieve observation
        rospy.loginfo("Get observation.")
        obs = self._get_obs()
        rospy.logdebug("Check if episode is done.")
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)

        # Publish reward and return action result
        rospy.logdebug("Publishing reward.")
        self._publish_reward_topic(reward, self.episode_num)
        return obs, reward, done, info

    def reset(self):
        # TODO: Check what happens in the original gym environment
        """Function used for resetting the simulation.

        Returns
        -------
            list :
                An observation of the initial state.
        """
        rospy.loginfo("Resetting Panda RobotGazeboGoal environment.")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        return obs

    def close(self):
        """Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        """
        rospy.logdebug("Closing Panda RobotGazeboGoal environment.")
        rospy.signal_shutdown("Closing Panda RobotGazeboGoal environment.")

    #############################################
    # Panda Gazebo env helper methods ###########
    #############################################
    def _spawn_object(self, object_name, model_name, pose=None):
        """Spawns a object from the model directory into gazebo.

        Parameters
        ----------
        object_name : str
            The name you want the model to have.
        model_name : str
            The model type (The name of the xml file you want to use).
        pose : geometry_msgs.msg.Pose, optional
            The pose of the model, by default :py:class:`geometry_msgs.msg.Pose`.

        Returns
        -------
        bool
            A boolean specifying whether the model was successfully spawned.

        Raises
        ------
        panda_openai_sim.exceptions.SpawnModelError
            When model was not spawned successfully.
        """

        # Initiate default model pose
        if not pose:
            pose = Pose()
            pose.orientation = Quaternion.normalize_quaternion(pose.orientation)

        # Check if model is already present
        if object_name in self.model_states.keys():
            rospy.logwarn(
                "A model with model name '%s' already exists. Please check if this is "
                "the right model." % (model_name)
            )
            return False

        # Spawn model using the gazebo spawn sdf/urdf service
        rospy.logdebug("Spawning model '%s' as '%s'." % (model_name, object_name))
        if (
            self._services_connection_status[GAZEBO_SPAWN_URDF_MODEL_TOPIC]
            and self._services_connection_status[GAZEBO_SPAWN_SDF_MODEL_TOPIC]
        ):

            # Find model xml
            rospy.logdebug("Looking for '%s' model file." % model_name)
            model_xml, extension = find_gazebo_model_path(
                model_name, GAZEBO_MODELS_FOLDER_PATH
            )
            if not model_xml:  # If model file was not found
                logwarn_msg = (
                    "Spawning model '%s' as '%s' failed since the sdf/urd model file "
                    "was not found. Please make sure you added the model sdf/urdf file "
                    "to the '%s' folder."
                    % (model_name, object_name, GAZEBO_MODELS_FOLDER_PATH)
                )
                rospy.logwarn(logwarn_msg)
                raise SpawnModelError(message=logwarn_msg)

            # Load content from the model xml file
            xml_file = open(model_xml, "r")
            model_xml_content = xml_file.read()

            # Create spawn model request message
            rospy.logdebug("Spawning '%s' model as '%s'." % (model_name, object_name))
            spawn_model_req = SpawnModelRequest(
                model_name=object_name,
                model_xml=model_xml_content,
                initial_pose=pose,
                reference_frame="world",
            )

            # Request model spawn from sdf or urdf spawn service
            if extension == "sdf":  # Use sdf service
                try:
                    retval = self._gazebo_spawn_sdf_model_client.call(spawn_model_req)
                except (
                    SerializationError,
                    AttributeError,
                    rospy.ServiceException,
                ) as e:  # Raise SpawnModelError if not successfull
                    logwarn_msg = "Spawning model '%s' as '%s' failed since %s." % (
                        model_name,
                        object_name,
                        lower_first_char(e.args[0]),
                    )
                    rospy.logwarn(logwarn_msg)
                    raise SpawnModelError(message=logwarn_msg, details={"exception": e})

                # Return success bool
                rospy.logdebug(retval.status_message)
                return retval
            else:  # Use urdf service
                try:
                    retval = self._gazebo_spawn_urdf_model_client.call(spawn_model_req)
                except (
                    SerializationError,
                    AttributeError,
                    rospy.ServiceException,
                ) as e:  # Raise SpawnModelError if not successfull
                    logwarn_msg = "Spawning model '%s' as '%s' failed since %s." % (
                        model_name,
                        object_name,
                        lower_first_char(e.args[0]),
                    )
                    rospy.logwarn(logwarn_msg)
                    raise SpawnModelError(message=logwarn_msg, details={"exception": e})

                # Return success bool
                rospy.logdebug(retval.status_message)
                return retval
        else:  # Raise SpawnModelError since service was not found
            logwarn_msg = (
                "Spawning model '%s' as '%s' failed since the '/%s' and '%s' services "
                "are not available."
                % (
                    model_name,
                    object_name,
                    GAZEBO_SPAWN_SDF_MODEL_TOPIC,
                    GAZEBO_SPAWN_URDF_MODEL_TOPIC,
                )
            )
            rospy.logwarn(logwarn_msg)
            raise SpawnModelError(message=logwarn_msg)

    def _set_model_state(self, object_name, pose, twist=Twist()):
        """Sets the model state of a gazebo object.

        Parameters
        ----------
        object_name : str
            The name of the object.
        pose : geometry_msgs.Pose
            The pose you want the object to have.
        twist : geometry_msgs.Twist, optional
            The twist you want the object to have, by default
            :geometry_msgs:`Twist()<html/msg/Twist.html>`.

        Returns
        -------
        bool
            A boolean specifying whether the state was successfully set.

        Raises
        ------
        panda_openai_sim.exceptions.SpawnModelError
            When model state was not set successfully.
        """

        # Set model state
        rospy.logdebug("setting '%s' model state." % object_name)
        if self._services_connection_status[GAZEBO_SET_MODEL_STATE_TOPIC]:

            # Create SetModelState msg
            model_state = ModelState(model_name=object_name, pose=pose, twist=twist)
            set_model_state_req = SetModelStateRequest(model_state)

            # Send set model state request to gazebo service
            retval = self._gazebo_set_model_state_client.call(set_model_state_req)
            if not retval:
                rospy.logwarn(retval.status_message)
            else:
                rospy.logdebug(retval.status_message)
            return retval
        else:

            # Raise SpawnModelError since service was not found
            logwarn_msg = (
                "Model state for object '%s' could not be set as the '%s' could not be "
                "found." % (object_name, GAZEBO_SET_MODEL_STATE_TOPIC)
            )
            rospy.logwarn(logwarn_msg)
            raise SetModelStateError(message=logwarn_msg)

    def _update_episode(self):
        """Increases the episode number by one."""
        self.episode_num += 1

    def _publish_reward_topic(self, reward, episode_number=1):
        """This function publishes the given reward in the reward topic for
        easy access from ROS infrastructure.


        Parameters
        ----------
        reward : :obj:`numpy.float32`
            The episode reward.
        episode_number : int, optional
            The episode number, by default 1.
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self._reward_pub.publish(reward_msg)

    #############################################
    # Callback functions ########################
    #############################################
    # NOTE: Implemented as a class property since using subscribers without the
    # ros.spin() command did not work.
    @property
    def link_states(self):
        """Callback function for retrieving the link_state data from Gazebo."""

        # Convert Gazebo link_states msgs to a link_states dictionary and store as
        # attribute
        return model_state_msg_2_link_state_dict(
            rospy.wait_for_message(GAZEBO_LINK_STATES_TOPIC, ModelStates)
        )

    @property
    def model_states(self):
        """Callback function for retrieving the model_state data from gazebo."""

        # Convert Gazebo model_states msgs to a model_states dictionary and store as
        # attribute
        return model_state_msg_2_link_state_dict(
            rospy.wait_for_message(GAZEBO_MODEL_STATES_TOPIC, ModelStates)
        )

    #############################################
    # Extension methods #########################
    #############################################
    # NOTE: These methods CAN be overloaded by robot or task env)
    def _reset_sim(self):
        """Resets a simulation.

        Returns
        -------
        bool
            Boolean specifying whether reset was successful.
        """

        # Reset simulation (and controls)
        if self.reset_controls:
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            if self.reset_robot_pose:  # Reset robot pose
                self._set_init_pose()
            self._set_init_obj_pose()  # Reset object pose
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        else:
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            if self.reset_robot_pose:  # Reset robot pose
                self._set_init_pose()
            self._set_init_obj_pose()  # Reset object pose
            self.gazebo.resetWorld()
            self._check_all_systems_ready()

        # Return result bool
        return True

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Can be used
        to enforce additional constraints on the simulation state.
        """
        pass

    #############################################
    # Wrapper functions #########################
    #############################################
    # NOTE: Functions used to make the package fully compatible with both the openai
    # gym and the openai_ros package.

    def compute_reward(self, achieved_goal, desired_goal, info):
        """Calculates the reward to give based on the observations given. This function
        serves as a wrapper to point calls to the :py:meth:`gym.compute_reward`
        function to our overloaded :py:meth:`_compute_reward` function.

        Parameters
        ----------
        achieved_goal : object
            The goal that was achieved during execution.
        desired_goal : object
            The desired goal that we asked the agent to attempt to achieve.
        info : dict
            An info dictionary with additional information
        """

        # Convert the gym input arguments into panda_openai_sim input arguments
        observations = {}
        observations["achieved_goal"] = achieved_goal
        observations["desired_goal"] = desired_goal
        if info:
            done = info["is_success"]
        else:
            done = None

        # Call the panda_openai_sim _compute_rewards function
        return self._compute_reward(observations, done)

    def _is_success(self, achieved_goal, desired_goal):
        """Check if task is done. This function serves as a wrapper to point calls to
        the :py:meth:`gym._is_success` function to our overloaded :py:meth:`_is_done`
        function.

        Parameters
        ----------
        achieved_goal : object
            The goal that was achieved during execution.
        desired_goal : object
            The desired goal that we asked the agent to attempt to achieve.

        Returns
        -------
        bool
            Boolean specifying whether the episode is done (e.i. distance to the goal is
            within the distance threshold, robot has fallen etc.).

        Note
        -----------
            The reason the name was changed to `_is_done` is twofold. First, with the
            use of an additional wrapper function, this makes the environment compatible
            with both packages that expect the openai :py:mod:`gym` package structure as
            well as packages that expect the :py:mod:`openai_ros` package structure.
            Additionally, I think the new naming better represents cases where the
            episode is terminated because, for example, the robot has fallen.
        """

        # Convert the gym input arguments into panda_openai_sim input arguments
        observations = {}
        observations["achieved_goal"] = achieved_goal
        observations["desired_goal"] = desired_goal

        # Call the panda_openai_sim _is_done function
        return self._is_done(self, observations)

    #############################################
    # Setup virtual methods #####################
    #############################################
    # NOTE: These virtual methods NEED to be overloaded by the Robot and Task env

    #########################################
    # Robot env virtual methods #############
    #########################################

    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    #########################################
    # Task env virtual methods ##############
    #########################################

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
