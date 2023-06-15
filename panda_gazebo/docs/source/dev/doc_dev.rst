=====================
Release documentation
=====================

.. contents:: Table of Contents

Install requirements
--------------------

Building the :panda_gazebo:`panda_gazebo <>` documentation requires `sphinx`_,
the ``panda_gazebo`` package and several system and python packages. To install the system dependencies, run the following command
inside your catkin workspace:

.. code-block:: bash

    rosdep install --from-path src --ignore-src -r -y -t doc

Additionally, the Python dependencies can be installed using the following `pip`_ command inside the ``./panda_gazebo`` folder:

.. code-block:: bash

    pip install -r requirements/doc_requirements.txt

.. _`sphinx`: http://www.sphinx-doc.org/en/master
.. _`pip`: https://pypi.org/project/pip/

.. note::
    You can also install these requirements in a virtual environment. If you want to do so, you are advised to use the
    `venv`_ package instead of `Conda`_ since the latter is known to cause issues when used with ROS. If you use the 
    `venv`_ package, please use the `--system-site-packages` flag when creating the virtual environment. This will
    ensure all the ROS system packages are available in the virtual environment. This is required because the `sphinx`_ 
    package needs to be able to import the `panda_gazebo` ROS package.

.. _venv: https://docs.python.org/3/library/venv.html
.. _Conda: https://docs.conda.io/en/latest/

Build the documentation
-----------------------

To build the Python and ROS documentation, go into the :panda_gazebo:`docs/ <tree/noetic/panda_gazebo/docs>` directory and run the
``make docs`` command. This command will use `rosdoc_lite`_ and `sphinx`_ to generate the 
html documentation inside the ``docs/build/html`` directory. If the documentation is successfully built, you can also use the 
``make linkcheck`` command to check for broken links.

.. note::
    If you used a virtual environment, ensure you are in the environment where you installed the panda_gazebo package with its
    dependencies.

.. note::
    You can also use the `make html` command to build the documentation. Although warnings and errors are now coloured, it does not produce the
    accompanying ROS package documentation. 

.. _rosdoc_lite: http://wiki.ros.org/rosdoc_lite
.. _HTML: https://www.w3schools.com/html/


Deploying
---------

The documentation is automatically built and deployed to the Github Pages site by the `Docs workflow`_ when a new version
is released. You must `create a new release`_ to deploy documentation to the Github Pages. Additionally, you can manually
deploy the documentation through the `GitHub action interface`_ by running the `Docs workflow`_.

.. _`create a new release`: https://rickstaa.dev/panda-gazebo/dev/contributing.html#release-guidelines
.. _`Docs workflow`: https://github.com/rickstaa/panda-gazebo/actions/workflows/documentation.yml
.. _`GitHub action interface`: https://docs.github.com/en/actions/using-workflows/triggering-a-workflow#defining-inputs-for-manually-triggered-workflows
If you used a virtual environment, ensure you are in the environment where you installed the panda_gazebo package with its dependencies.
