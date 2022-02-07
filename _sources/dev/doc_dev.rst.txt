=====================
Release documentation
=====================

.. contents:: Table of Contents

Install requirements
--------------------

Building the :panda_gazebo:`panda_gazebo <>` documentation requires `sphinx`_,
the panda_gazebo package and several plugins. All of the above can be
installed using the following `pip`_ command inside the ``./panda_gazebo`` folder:

.. code-block:: bash

    pip install -e .[docs]

.. _`sphinx`: http://www.sphinx-doc.org/en/master
.. _`pip`: https://pypi.org/project/pip/

Build the documentation
-----------------------

To build the `HTML`_ documentation, go into the :panda_gazebo:`docs/ <tree/noetic/panda_gazebo/docs>` directory and run the
``make html`` command. This command will generate the html documentation
inside the ``docs/build/html`` directory.

.. note::
    Make sure you are in the Conda environment in which you installed the panda_gazebo package
    with it's dependencies.

.. _`HTML`: https://www.w3schools.com/html/

Deploying
---------

To deploy documentation to the Github Pages site for the repository, push the
documentation to the :panda_gazebo:`noetic <tree/noetic>` branch and run the ``make gh-pages`` command
inside the :panda_gazebo:`docs/ <tree/noetic/panda_gazebo/docs>` directory.

.. warning::

    Please make sure you are on the :panda_gazebo:`noetic <tree/noetic>` branch while building the documentation. Otherwise,
    errors will greet you.
