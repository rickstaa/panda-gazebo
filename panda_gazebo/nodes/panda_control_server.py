#!/usr/bin/env python3
"""This node sets up a number of services that can be used to control the Panda Emika
Franka robot.

Source code
-----------

.. literalinclude:: ../../../../../panda_gazebo/nodes/panda_control_server.py
   :language: python
   :linenos:
   :lines: 14-
"""

import rospy
from panda_gazebo.core.control_server import PandaControlServer

if __name__ == "__main__":
    rospy.init_node("panda_control_server")

    # Get ROS parameters
    try:  # Auto fill joint traj position field if left empty
        autofill_traj_positions = rospy.get_param("~autofill_traj_positions")
    except KeyError:
        autofill_traj_positions = False
    try:  # Check if extra services should be loaded
        load_extra_services = rospy.get_param("~load_extra_services")
    except KeyError:
        load_extra_services = False
    try:  # Disables the gripper width reached check when grasping.
        brute_force_grasping = rospy.get_param("~brute_force_grasping")
    except KeyError:
        brute_force_grasping = False

    # Start control server
    control_server = PandaControlServer(
        autofill_traj_positions=autofill_traj_positions,
        load_extra_services=load_extra_services,
        brute_force_grasping=brute_force_grasping,
    )
    rospy.spin()  # Maintain the service open
