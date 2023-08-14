#!/usr/bin/env python3
"""This node sets up several extra services that can be used to control the Panda Emika
Franka robot using the Moveit framework.

Source code
-----------

.. literalinclude:: ../../../../../panda_gazebo/nodes/panda_moveit_server.py
   :language: python
   :linenos:
   :lines: 14-
"""
import rospy

from panda_gazebo.core.moveit_server import PandaMoveItPlannerServer

if __name__ == "__main__":
    rospy.init_node("panda_moveit_planner_server")

    # Get private parameters specified in the launch file.
    try:
        arm_ee_link = rospy.get_param("~end_effector")
    except KeyError:
        arm_ee_link = "panda_link8"
    try:
        load_gripper = rospy.get_param("~load_gripper")
    except KeyError:
        load_gripper = True
    try:  # Check if set ee pose service should be loaded.
        load_set_ee_pose_service = rospy.get_param("~load_set_ee_pose_service")
    except KeyError:
        load_set_ee_pose_service = True
    try:  # Check if extra services should be loaded.
        load_extra_services = rospy.get_param("~load_extra_services")
    except KeyError:
        load_extra_services = False

    # Start MoveIt planner server.
    moveit_planner_server = PandaMoveItPlannerServer(
        arm_ee_link=arm_ee_link,
        load_gripper=load_gripper,
        load_set_ee_pose_service=load_set_ee_pose_service,
        load_extra_services=load_extra_services,
    )
    rospy.spin()  # Maintain the service open.
