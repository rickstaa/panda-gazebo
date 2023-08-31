============
Installation
============

Install dependencies
--------------------

The following dependencies are required to run the :panda-gazebo:`panda_gazebo <>` package:

* `ROS Noetic - Desktop full <https://wiki.ros.org/noetic/Installation>`_.
* `libfranka library <https://github.com/frankaemika/libfranka>`_ **(automatically installed by rosdep)**.
* `Python 3 <https://www.python.org/downloads/>`_.

Clone instructions
------------------

To use this workspace, clone the repository inside your Catkin workspace folder. Since the repository contains several git
submodules to use all the features, it needs to be cloned using the ``--recurse-submodules`` argument:

.. code-block:: bash

    git clone --recurse-submodules https://github.com/rickstaa/panda-gazebo.git

If you already cloned the repository and forgot the `--recurse-submodule` argument you
can pull the submodules using the following git command:

.. code-block:: bash

    git submodule update --init --recursive

Build instructions
------------------

After you cloned the repository, you have to install the system dependencies using the `rosdep tool`_:

.. code-block:: bash

    rosdep install --from-path src --ignore-src -r -y

When this command finishes, you can build the ROS packages inside the Catkin workspace using the following build command:

.. code-block:: bash

    catkin build -j4 -DCMAKE_BUILD_TYPE=Release

.. _`rosdep tool`: https://wiki.ros.org/rosdep
.. _`libfranka`: https://github.com/frankaemika/libfranka
.. _`the franka documentation`: https://frankaemika.github.io/docs/installation_linux.html#building-from-source)
