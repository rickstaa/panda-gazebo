.. _api:

Python Code API
===============

The :gazebo-panda-gym:`panda_openai_sim <>` package consists of two sub-packages:
the :panda_openai_sim:`panda_openai_sim <>` ROS package and the
:panda_training:`panda_training <>` python package. Each of the modules, classes and
scripts in these two packages will be documented here.

Panda_openai_sim package
------------------------

Modules
^^^^^^^^

.. autosummary::
   :toctree: _autosummary
   :recursive:

    panda_openai_sim.core
    panda_openai_sim.envs
    panda_openai_sim.extras
    panda_openai_sim.exceptions
    panda_openai_sim.errors
    panda_openai_sim.functions


ROS nodes
^^^^^^^^^

.. autosummary::
   :toctree: _autosummary
   :recursive:

    panda_control_server
    panda_moveit_server

Panda_training package
----------------------

Modules
^^^^^^^

.. autosummary::
   :toctree: _autosummary
   :recursive:

    panda_training.functions


Scripts
^^^^^^^

.. autosummary::
   :toctree: _autosummary
   :recursive:

    morvan_ddpg_panda_train_and_inference
    stable_baselines_ddpg_panda_train_and_inference
    stable_baselines_her_ddpg_panda_train_and_inference
    stable_baselines_sac_panda_train_and_inference
