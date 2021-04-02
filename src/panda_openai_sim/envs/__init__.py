"""Module that contains the Panda openai :gym:`gym <>` task, robot and Goal
environments.
"""

# Python 2 and 3:
# To make Py2 code safer (more like Py3) by preventing
# implicit relative imports, you can also add this to the top:
from __future__ import absolute_import

# Import module classes
from panda_openai_sim.envs.robot_gazebo_goal_env import RobotGazeboGoalEnv

# Import task environments
from panda_openai_sim.envs.task_envs import (
    PandaSlideEnv,
    PandaPickAndPlaceEnv,
    PandaReachEnv,
    PandaPushEnv,
)

# Import classes used for the gym environment registration
from gym.envs.registration import register

#################################################
# Register gym environments #####################
#################################################
for reward_type in ["sparse", "dense"]:  # Create both dense and sparse environments
    suffix = "Dense" if reward_type == "dense" else ""
    kwargs = {
        "reward_type": reward_type,
    }

    # Main task env (Contains all input arguments)
    register(
        id="PandaTask{}-v0".format(suffix),
        entry_point="panda_openai_sim.envs.task_envs:PandaTaskEnv",
        kwargs=kwargs,
        max_episode_steps=50,
    )

    # Slide
    register(
        id="PandaSlide{}-v0".format(suffix),
        entry_point="panda_openai_sim.envs.task_envs:PandaSlideEnv",
        kwargs=kwargs,
        max_episode_steps=50,
    )

    # Pick and Place
    register(
        id="PandaPickAndPlace{}-v0".format(suffix),
        entry_point="panda_openai_sim.envs.task_envs:PandaPickAndPlaceEnv",
        kwargs=kwargs,
        max_episode_steps=50,
    )

    # Reach
    register(
        id="PandaReach{}-v0".format(suffix),
        entry_point="panda_openai_sim.envs.task_envs:PandaReachEnv",
        kwargs=kwargs,
        max_episode_steps=50,
    )

    # Push
    register(
        id="PandaPush{}-v0".format(suffix),
        entry_point="panda_openai_sim.envs.task_envs:PandaPushEnv",
        kwargs=kwargs,
        max_episode_steps=50,
    )
