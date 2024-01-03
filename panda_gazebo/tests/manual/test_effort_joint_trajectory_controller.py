#!/usr/bin/env python3
"""Small script that can be used to test the effort_joint_trajectory_controller."""

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node("position_joint_trajectory_controller_test")

    # Create action client.
    follow_joint_traj_client = actionlib.SimpleActionClient(
        "/panda_arm_trajectory_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )

    # Waits until the action server has started up and started
    # listening for goals.
    retval = follow_joint_traj_client.wait_for_server(timeout=rospy.Duration(5))
    if not retval:
        rospy.signal_shutdown("Action server not available!")

    # Create action client goal.
    header = Header()
    # header.stamp = rospy.get_rostime()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ]
    point = JointTrajectoryPoint()
    point.positions = [
        -0.1898988458903048,
        -1.0548482299719186,
        -0.6408902483998249,
        -2.207504181306631,
        -0.5692399052822985,
        1.3015368095958824,
        0.07330453737040443,
    ]
    point.time_from_start.secs = 1
    goal.trajectory.points.append(point)
    point2 = JointTrajectoryPoint()
    point2.positions = [
        0.5,
        -0.5,
        0.1,
        -0.579418370643596,
        -0.208283306265037815,
        0.3501401410371793,
        0.44266583523250507,
    ]
    point2.time_from_start.secs = 2
    goal.trajectory.points.append(point2)
    # goal.trajectory.header = header
    # goal.goal_time_tolerance.secs = 5

    # Send goal.
    follow_joint_traj_client.send_goal(goal)
    follow_joint_traj_client.wait_for_result()
    result = follow_joint_traj_client.get_result()
    print(result)
