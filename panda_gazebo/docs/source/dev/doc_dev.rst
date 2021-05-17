.. _doc_dev:

Documentation development
=========================

Install requirements
--------------------

To build the :gazebo-panda-gym:`panda_openai_sim <>` documentation the
:sphinx:`sphinx <>` tool together with some other python packages is needed. These
python requirements can be installed by running the following pip command inside your
python 3 environment:

.. code-block:: bash

    cd ~/gazebo-panda-gym/panda_openai_sim
    pip install .[docs]

Build the documentation
-----------------------

To be able to build the code API, you need first to make sure the catkin_ws has
been built. Additionally you need to make sure all the python dependencies for
the :panda_openai_sim:`panda_openai_sim <>` and
:panda_training:`panda_training <>` modules have been installed
inside your python 3 environment. This can be done by running the following pip command
from within the ``panda_training`` and ``panda_openai_sim`` folders:

.. code-block:: bash

    pip install .

After all the dependencies are installed you can build the documentation using the
following steps:

    #. Source the :gazebo-panda-gym:`panda_openai_sim <>` catkin workspace ``source ../devel/setup.bash``.
    #. Source the devel workspace of the python 3 :ros:`ROS <>` packages ``source ~./.catkin_ws_python3/devel/setup.bash``.
    #. Run the ``make clean`` command inside the ``docs`` folder.
    #. Run the ``make html`` command inside the ``docs`` folder.


.. warning::

    Please make sure you use a (virtual) python 3 environment when building the
    documentation as most of the scripts are written in python 3.

Deploy the documentation
------------------------

To deploy documentation to the Github Pages site for the repository, push the
documentation to the ``melodic-devel`` branch and run the ``make gh-pages`` command
inside the ``panda_autograsp/docs`` directory. This will create the documentation and
move the HTML files to the ``gh-pages`` branch.

.. warning::

    Please make sure you are on the ``melodic-devel`` branch while deploying the
    documentation. Otherwise, you will be greeted by errors.

Known issues
------------

ROS python 3 incompatibility issues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :gazebo-panda-gym:`panda_openai_sim <>` package is not yet fully ported to
python 3 as Moveit is not yet released for :ros:`ROS Noetic <noetic>`. As a result one
might encounter one of the following errors trying to build the documentation:

- ``dynamic module does not define module export function (PyInit__tf2)``
- ``dynamic module does not define module export function (PyInit__moveit_roscpp_initializer)``

These errors are solved by building several :ros:`ROS <>` packages for python 3 from
source and sourcing the ``devel/setup.bash``. How this is done is explained in the
:ref:`installation instructions <catkin_ws_python>`.
