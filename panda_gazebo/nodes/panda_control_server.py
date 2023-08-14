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

if __name__ == "__main__":  # noqa: C901
    rospy.init_node("panda_control_server")

    # Get ROS parameters.
    try:  # Auto fill joint traj position field if left empty.
        autofill_traj_positions = rospy.get_param("~autofill_traj_positions")
    except KeyError:
        autofill_traj_positions = False
    try:
        load_gripper = rospy.get_param("~load_gripper")
    except KeyError:
        load_gripper = True
    try:  # Check if set joint commands service should be loaded.
        load_set_joint_commands_service = rospy.get_param(
            "~load_set_joint_commands_service"
        )
    except KeyError:
        load_set_joint_commands_service = True
    try:  # Check if arm follow joint trajectory action should be loaded.
        load_arm_follow_joint_trajectory_action = rospy.get_param(
            "~load_arm_follow_joint_trajectory_action"
        )
    except KeyError:
        load_arm_follow_joint_trajectory_action = False
    try:  # Check if extra services should be loaded.
        load_extra_services = rospy.get_param("~load_extra_services")
    except KeyError:
        load_extra_services = False
    try:  # Disables the gripper width reached check when grasping.
        brute_force_grasping = rospy.get_param("~brute_force_grasping")
    except KeyError:
        brute_force_grasping = False
    try:  # The rate with which we check for the used controllers to be active.
        controllers_check_rate = rospy.get_param("~controllers_check_rate")
    except KeyError:
        controllers_check_rate = 0.1

    # Start control server.
    control_server = PandaControlServer(
        autofill_traj_positions=autofill_traj_positions,
        load_gripper=load_gripper,
        load_set_joint_commands_service=load_set_joint_commands_service,
        load_arm_follow_joint_trajectory_action=load_arm_follow_joint_trajectory_action,
        load_extra_services=load_extra_services,
        brute_force_grasping=brute_force_grasping,
        controllers_check_rate=controllers_check_rate,
    )
    rospy.spin()  # Maintain the service open.
