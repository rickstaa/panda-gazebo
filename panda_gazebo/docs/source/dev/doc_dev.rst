=====================
Release documentation
=====================

.. contents:: Table of Contents

Install requirements
--------------------

Building the :panda-gazebo:`panda-gazebo <>` documentation requires `sphinx`_,
the panda-gazebo package and several plugins. All of the above can be
installed using the following `pip`_ command:

.. code-block:: bash

    pip install -e .[docs]

.. _`sphinx`: http://www.sphinx-doc.org/en/master
.. _`pip`: https://pypi.org/project/pip/

Build the documentation
-----------------------

To build the `HTML`_ documentation, go into the `docs/`_ directory and run the
``make html`` command. This command will generate the html documentation
inside the ``docs/build/html`` directory.

.. note::
    Make sure you are in the Conda environment in which you installed the panda-gazebo package
    with it's dependencies.

.. _`HTML`: https://www.w3schools.com/html/

Deploying
---------

To deploy documentation to the Github Pages site for the repository,
push the documentation to the `noetic-devel`_ branch and run the
``make gh-pages`` command inside the `docs/`_ directory.

.. warning::

    Please make sure you are on the `noetic-devel`_ branch while building the documentation. Otherwise,
    errors will greet you.

.. _`docs/`: https://github.com/rickstaa/panda-gazebo/tree/main/panda/docs
.. _`noetic-devel`: https://github.com/rickstaa/panda/tree/noetic-devel