"""An openai gym ROS Panda reach environment."""

# Main python imports
from gym import utils

from .panda_task_env import PandaTaskEnv


#################################################
# Panda reach task environment Class ############
#################################################
class PandaReachEnv(PandaTaskEnv, utils.EzPickle):
    """Classed used to create a Panda reach environment. It inherits from the
    main Panda Task environment
    :py:class:`panda_openai_sim.envs.task_envs.PandaTaskEnv`.
    """

    def __init__(
        self,
        reward_type="sparse",
        distance_threshold=0.05,
        target_bounds=None,
        target_sampling_strategy=None,
        init_pose=None,
        init_pose_bounds=None,
        init_obj_pose=None,
        obj_bounds=None,
        use_gripper_width=True,
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
    ):
        """Initializes a Panda Pick and Place task environment.

        Parameters
        ----------
        reward_type : str, optional
            The reward type, i.e. ``sparse`` or ``dense``, by default sparse.
        distance_threshold : float, optional
            The threshold after which a goal is considered achieved, by default 0.05.
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
            {x, y, z, rx, ry, rz, rw}, by default None. The object will be spawned
            relative to this pose in a region defined by the obj_bounds.
        obj_bounds : dict, optional
            A dictionary in which the bounds for sampling the object
            positions is defined ``{x_min, y_min, x_max, y_max}``, by default None. This
            bounds are relative to the init_obj_pose.
        use_gripper_width : bool, optional
            Whether you want to use the gripper_width instead of the individual panda
            finger joint commands during the training, by default True. Consequently,
            using the gripper width reduces the action space by 1.
        robot_arm_control_type : str, optional
            The type of control you want to use for the robot arm. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``, by default
            joint_trajectory_control.
        robot_hand_control_type : str, optional
            The type of control you want to use for the robot hand. Options are
            ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control``,
            ``joint_group_effort_control`` or ``ee_control``, by default
            joint_trajectory_control.

        Note
        -----------
            If the default value for a argument is set to None this means the default
            values of the parent class
            :class:`panda_openai_sim.envs.task_envs.panda_task_env.PandaTaskEnv` are
            used.
        """

        # Initiate main Panda Task environment
        super(PandaReachEnv, self).__init__(
            reward_type=reward_type,
            distance_threshold=distance_threshold,
            has_object=False,
            block_gripper=True,
            target_in_the_air=True,
            gripper_extra_height=0.2,
            target_offset={"x": 0.0, "y": 0.0, "z": 0.0},
            target_bounds=target_bounds,
            target_sampling_strategy=target_sampling_strategy,
            init_pose=init_pose,
            init_pose_bounds=init_pose_bounds,
            use_gripper_width=use_gripper_width,
            robot_arm_control_type=robot_arm_control_type,
            robot_hand_control_type=robot_hand_control_type,
        )
        utils.EzPickle.__init__(self)
