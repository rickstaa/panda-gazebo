:py:mod:`panda_gazebo.common`
=============================

.. py:module:: panda_gazebo.common

.. autoapi-nested-parse::

   Contains some extra components (classes and functions) that are used for
   creating the :panda_gazebo:`panda_gazebo <>` simulation.



Submodules
----------
.. toctree::
   :titlesonly:
   :maxdepth: 1

   controlled_joints_dict/index.rst
   controller_info_dict/index.rst
   functions/index.rst


Package Contents
----------------

Classes
~~~~~~~

.. autoapisummary::

   panda_gazebo.common.ControlledJointsDict
   panda_gazebo.common.ControllerInfoDict




.. py:class:: ControlledJointsDict(*args, **kwargs)


   Bases: :py:obj:`dict`

   Used for storing information about the currently controlled joints.
   This class overloads the normal ``dict`` class in order to pre-initialize the
   dictionary with the needed keys.


.. py:class:: ControllerInfoDict(*args, **kwargs)


   Bases: :py:obj:`dict`

   Used for storing information about the Gazebo robot controllers.
   This class overloads the normal :obj:`dict` class in order to pre-initialize the
   dictionary with the needed keys.


