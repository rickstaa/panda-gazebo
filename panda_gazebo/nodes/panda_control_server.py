#!/usr/bin/env python3
"""This node sets up a number of services that can be used to control the Panda Emika
Franka robot.

Source code
----------------------------
.. literalinclude:: ../../../../panda_openai_sim/nodes/panda_control_server.py
   :language: python
   :linenos:
   :lines: 13-
"""

# Import ROS packages
import rospy

# Panda_autograsp modules, msgs and srvs
from panda_gazebo.core import PandaControlServer

if __name__ == "__main__":
    rospy.init_node("panda_control_server")

    # Get ROS parameters
    try:
        use_group_controller = rospy.get_param("~use_group_controller")
    except KeyError:
        use_group_controller = False
    try:  # Auto fill joint traj position field if left empty
        autofill_traj_positions = rospy.get_param("~autofill_traj_positions")
    except KeyError:
        autofill_traj_positions = False
    try:  # Check required services
        create_all_services = rospy.get_param("~services_load_type")
    except KeyError:
        create_all_services = False

    # Start control server
    control_server = PandaControlServer(
        use_group_controller=use_group_controller,
        autofill_traj_positions=autofill_traj_positions,
        create_all_services=create_all_services,
    )
    rospy.spin()  # Maintain the service open
