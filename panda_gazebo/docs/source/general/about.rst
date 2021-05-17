.. _about:

Introduction
============

The :gazebo-panda-gym:`gazebo-panda-gym <>` package contains several
:gym:`openai gym <>` environments for the :franka:`Panda Emika franka <>` robot that can
be used to train RL algorithms. It was based on the :gym:`openai gym <>` and
:openai_ros:`openai_ros <>` packages and can be used similarly as the original
:gym:`openai gym Robotics <envs/#robotics>` environments. The
:gazebo-panda-gym:`gazebo-panda-gym <>` package consists of two main sub-packages:

* **The panda_openai_sim package:** This package contains all of of the available :gym:`openai gym <>` environments and the :franka:`Panda Emika Franka <>` simulation.
* **The panda_training package:** This package includes several examples of how the :panda_openai_sim:`panda_openai_sim <>` environments can be used to train RL algorithms.

Panda_openai_sim package
------------------------

Environments
^^^^^^^^^^^^

The :panda_openai_sim:`panda_openai_sim package <>` currently contains the following task
environments:

* **PandaPickAndPlace-v0:** Lift a block into the air.
* **PandaPush-v0:** Push a block to a goal position.
* **PandaReach-v0:** Move fetch to a goal position.
* **PandaSlide-v0:** Slide a puck to a goal position.

Panda_training package
----------------------------------

The :panda_training:`panda_training <>` package contains the following example training
scripts:

* **morvan_ddpg_panda_train_and_inference:** Trains a DDPG_ RL algorithm using the classes of MorvanZhou_.
* **stable_baselines_ddpg_panda_train_and_inference:** Trains a :stable-baselines:`DDPG <modules/ddpg.html>` RL algorithm using the :stable-baselines:`stable-baselines package <>`.
* **stable_baselines_her_ddpg_panda_train_and_inference:** Trains a :stable-baselines:`DDPG <modules/ddpg.html>` RL algorithm using the :stable-baselines:`stable-baselines <>` :stable-baselines:`HER wrapper<modules/her.html>`.
* **stable_baselines_sac_panda_train_and_inference:** Trains a :stable-baselines:`SAC <modules/sac.html>` RL algorithm using the :stable-baselines:`stable-baselines package <>`.

.. _DDPG: https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow/
.. _MorvanZhou: https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow/tree/master/contents/9_Deep_Deterministic_Policy_Gradient_DDPG/
