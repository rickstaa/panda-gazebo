"""Module that contains the core components (classes and functions) that are needed for
creating the Panda openai :gym:`gym <>` environment.
"""

# Python 2 and 3:
# To make Py2 code safer (more like Py3) by preventing
# implicit relative imports, you can also add this to the top:
from __future__ import absolute_import

from panda_gazebo.core.control_server import PandaControlServer
from panda_gazebo.core.control_switcher import PandaControlSwitcher

# Import module classes
from panda_gazebo.core.group_publisher import GroupPublisher

# NOTE: We can not yet import Moveit when using both python 2 and python 3 as Moveit is
# not yet python3 compatible.
# TODO: Fix if moveit becomes python 3 compatible (ROS NOETIC)
# from .moveit_server import PandaMoveitPlannerServer
