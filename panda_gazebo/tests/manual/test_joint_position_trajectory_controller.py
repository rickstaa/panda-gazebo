#!/usr/bin/env python3
"""Small script that sends a sinusoid command to one of the panda joints."""
import time

import actionlib
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

from panda_gazebo.common.helpers import ros_exit_gracefully

TIMESTEP = 0.01
TIME = 10


def periodic_test(trajectory_client):
    """Send a periodic command to the robot using the trajectory controller."""
    # Get starting state.
    joint_states = rospy.wait_for_message("/joint_states", JointState)
    joint_states_dict = dict(zip(joint_states.name, joint_states.position))

    # Create action client goal.
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
    for t in np.arange(0, TIME, TIMESTEP):
        point = JointTrajectoryPoint()
        point.positions = [
            # 2.9671 * np.sin(t),
            joint_states_dict["panda_joint1"],
            joint_states_dict["panda_joint2"],
            # 1.8326 * np.sin(t),
            joint_states_dict["panda_joint3"],
            2.9671 * np.sin(t),
            joint_states_dict["panda_joint4"],
            joint_states_dict["panda_joint5"],
            joint_states_dict["panda_joint6"],
            joint_states_dict["panda_joint7"],
        ]
        point.time_from_start = rospy.Time.from_sec(t)
        goal.trajectory.points.append(point)

    # Send goal.
    trajectory_client.send_goal(goal)

    # Publish goal to /control_signal topic.
    pub = rospy.Publisher("/control_signal", JointState, queue_size=10)
    rate = rospy.Rate(1 / TIMESTEP)
    t_start = time.time()
    while time.time() - t_start < TIME and not rospy.is_shutdown():
        msg = JointState(position=[2.9671 * np.sin(time.time() - t_start)])
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    # Retrieve result.
    trajectory_client.wait_for_result()
    result = trajectory_client.get_result()
    print(result)


def position_test(client):
    """Send a position command to the robot using the trajectory controller."""
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
    goal.trajectory.header = header
    # goal.goal_time_tolerance.secs = 5

    # Send goal.
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print(result)


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
        ros_exit_gracefully(
            shutdown_msg=f"Shutting down '{rospy.get_name()}'", exit_code=1
        )

    # Periodic test.
    # periodic_test(follow_joint_traj_client)

    # Position test.
    position_test(follow_joint_traj_client)
