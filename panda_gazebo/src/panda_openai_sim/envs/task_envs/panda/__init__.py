"""This module contains several different Task environments on which a RL algorithms can
be trained.
"""

# Python 2 and 3:
# To make Py2 code safer (more like Py3) by preventing
# implicit relative imports, you can also add this to the top:
from __future__ import absolute_import

# Import module classes
from panda_openai_sim.envs.task_envs.panda_pick_and_place_env import (
    PandaPickAndPlaceEnv,
)
from panda_openai_sim.envs.task_envs.panda_push_env import PandaPushEnv
from panda_openai_sim.envs.task_envs.panda_reach_env import PandaReachEnv
from panda_openai_sim.envs.task_envs.panda_slide_env import PandaSlideEnv
from panda_openai_sim.envs.task_envs.panda_task_env import PandaTaskEnv
