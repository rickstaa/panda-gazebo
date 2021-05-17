.. _install:

.. _`issue #17`: https://github.com/rickstaa/panda_openai_sim/issues/17/

Installation
============

Install dependencies
--------------------

The following dependencies are required to run the
:gazebo-panda-gym:`panda_openai_sim <>` package:

* `ROS Melodic - Desktop full <https://wiki.ros.org/melodic/Installation/Ubuntu/>`_
* `Franka_ros (Build from source) <https://frankaemika.github.io/docs/>`_
* Several system dependencies.

System dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^

To use the scripts inside the :panda_training:`panda_training <>` package, the following
system dependencies are needed:

.. code-block:: bash

    sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev

ROS melodic
^^^^^^^^^^^

A guide on how to install ROS melodic can be found
`here <https://wiki.ros.org/melodic/Installation/Ubuntu/>`_.

Franka_ros
^^^^^^^^^^

The steps for building the `franka_ros <https://github.com/frankaemika/franka_ros/>`_
package from source can be found in the
`Franka documentation <https://frankaemika.github.io/docs/installation_linux.html#building-from-source/>`_.


Build the package
-----------------

Compile several ROS packages for python 3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _catkin_ws_python:

Since the Panda gym environments and RL training scripts are written in python 3 and ROS
does not fully support python 3
(see `issue #17`_) we need to
build the following ROS packages for python 3 from source:

* `geometry2 <https://github.com/ros/geometry2/>`_
* `ros_comm <https://github.com/ros/ros_comm/>`_
* `geometry_msgs <https://github.com/ros/common_msgs/>`_

The following dependencies are required to build these packages:

.. code-block:: bash

    sudo apt update
    sudo apt install python-catkin-tools python3-catkin-pkg-modules python3-rospkg-modules python3-empy python3-sip python3-sip-dev

Then prepare catkin workspace:

.. code-block:: bash

    mkdir -p ~/.catkin_ws_python3/src
    cd ~/.catkin_ws_python3
    catkin_make
    source devel/setup.bash
    wstool init src
    rosinstall_generator ros_comm common_msgs  geometry2 --rosdistro melodic --deps | wstool merge -t src -
    wstool update -t src -j8
    rosdep install --from-paths src --ignore-src -y -r

Finally compile the ROS packages for Python 3 using the following catkin build command:

.. code-block:: bash

    catkin build --cmake-args \
                -DCMAKE_BUILD_TYPE=Release \
                -DPYTHON_EXECUTABLE=/usr/bin/python3 \
                -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
                -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

Build the main catkin package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the required ROS packages have been build for python 3 you can build
the :gazebo-panda-gym:`panda_openai_sim <>` package. To do this first clone the
:gazebo-panda-gym:`panda_openai_sim <>` repository:

.. code-block:: bash

    mkdir ~/gazebo-panda-gym
    cd ~/gazebo-panda-gym
    git clone --recursive https://github.com/rickstaa/panda_openai_sim.git src


Then install the required dependencies using the following command:

.. code-block:: bash

    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka


Finally, the catkin package can be build by executing the following command from within
the catkin workspace:

.. code-block:: bash

    catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build

Create a python 3 virtual environments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. _py3_virtual_env:

Since ROS does not yet fully support python 3
(see `issue #17`_), we need
to separate the training script (python 3) and the ROS gazebo simulation (python 2). To
do this, please create a virtual environment:

.. code-block:: bash

    sudo apt install virtualenv
    virtualenv ~/.catkin_ws_python3/openai_venv --python=python3

After this environment is created, you can activate it using the
``source ~/.catkin_ws_python3/openai_venv/bin/activate`` command. Following you need
to make sure all the python dependencies for the :panda_openai_sim:`panda_openai_sim <>`
and :panda_training:`panda_training <>` packages have been installed inside this
python 3 environment. This can be done by running the following pip command from within
the ``panda_training`` and ``panda_openai_sim`` folders:

.. code-block:: bash

    pip install .
