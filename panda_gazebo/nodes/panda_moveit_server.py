#!/usr/bin/env python
"""This node sets up a number of services that can be used to control
the Panda Emika Franka robot using the Moveit framework.

Source code
----------------------------
.. literalinclude:: ../../../../panda_openai_sim/nodes/panda_moveit_server.py
   :language: python
   :linenos:
   :lines: 13-
"""

# Import ROS packages
import rospy

# Panda_autograsp modules, msgs and srvs
from panda_openai_sim.core.moveit_server import PandaMoveitPlannerServer


#################################################
# Main script####################################
#################################################
if __name__ == "__main__":

    # Initiate Moveit Planner Server
    rospy.init_node("panda_moveit_planner_server")

    # Get private parameters specified in the launch file
    try:  # Check end effector
        arm_ee_link = rospy.get_param("~end_effector")
    except KeyError:
        arm_ee_link = "panda_hand"
    try:  # Check required services
        create_all_services = rospy.get_param("~services_load_type")
    except KeyError:
        create_all_services = False

    # Start moveit planner server
    moveit_planner_server = PandaMoveitPlannerServer(
        arm_ee_link=arm_ee_link, create_all_services=create_all_services
    )
    rospy.spin()  # Maintain the service open
