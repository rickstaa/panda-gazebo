.. _use:

.. _`issue #17`: https://github.com/rickstaa/panda_openai_sim/issues/17/

Tutorial
========================

Using the panda_openai_sim package
--------------------------------------

Here is a bare minimum example of getting something running. This will run an instance
of the PandaReach-v0 environment for 1000 timesteps.

.. code-block:: python

    import gym
    import panda_openai_sim.envs
    env = gym.make('PandaReach-v0')
    env = gym.wrappers.FlattenObservation(env)
    env.reset()
    for _ in range(1000):
        env.step(env.action_space.sample()) # take a random action
    env.close()

For this to work, you must run the simulated Panda robot. After you have
successfully build the :panda_openai_sim:`panda_openai_sim <>` ROS package and
installed all the dependencies you can run the simulation using the following
ROS command:

.. code-block:: bash

    roslaunch panda_openai_sim start.launch

.. warning::

    Since ROS does not yet fully support python 3
    (see `issue #17`_), we need
    to run the Panda gym environments and the :panda_training:`panda_training <>`
    scripts inside a separate python 3 environment. You therefore, have to make sure
    the training scripts are started inside the virtual environment that was created
    during the :ref:`the installation <py3_virtual_env>`.


Changing parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Most of the Panda Robot environment parameters can be changed using the environment
constructor (see the :ref:`code api <api>` for more information). Some additional global parameters
can be found in the :env_config:`env_config.yaml <>` file.

Using the panda_training package
------------------------------------

To run the example training scripts of the :panda_training:`panda_training package <>` you
first have to make sure the :panda_openai_sim:`panda_openai_sim <>` Panda_simulation is
running. Following you have to make sure that you activated the python 3 virtual
environment using the ``source  ~/.catkin_ws_python3/openai_venv/bin/activate`` command.
After you are inside the python environment, you can run each of the example scripts.

Changing parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All of the parameters for the example scripts are found at the top of the scripts.