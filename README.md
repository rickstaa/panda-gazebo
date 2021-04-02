# Panda_openai_sim

A ROS package that creates an Openai gym environment for the Panda Emika Franka robot.
It contains a simulated version of the Panda robot together with a Panda gym environment
that can be used to train RL algorithms as is done with the original
[openai_gym robotics environments](https://gym.openai.com/envs/#robotics).

## Environments

The panda_openai_sim package currently contains the following task environments:

- **PandaPickAndPlace-v0:** Lift a block into the air.
- **PandaPush-v0:** Push a block to a goal position.
- **PandaReach-v0:** Move fetch to a goal position.
- **PandaSlide-v0:** Slide a puck to a goal position.

Here is a bare minimum example of getting something running. This will run an instance
of the PandaReach-v0 environment for 1000 timesteps.

```python
import gym
import panda_openai_sim.envs
env = gym.make('PandaReach-v0')
env = gym.wrappers.FlattenObservation(env)
env.reset()
for _ in range(1000):
    env.step(env.action_space.sample()) # take a random action
env.close()
```

For this to work, you must run the simulated Panda robot. After you have
successfully build the `panda_openai_sim` ROS package and installed all the dependencies
you can run the simulation using the following ROS command:

```bash
roslaunch panda_openai_sim start.launch
```

## Installation and Usage

Please see the [docs](https://rickstaa.github.io/panda_openai_sim/) for installation
and usage instructions.
