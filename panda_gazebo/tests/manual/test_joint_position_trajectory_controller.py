#!/usr/bin/env python3
"""Small script that sends a sinusoid command to one of the panda joints."""
import sys
import time

import actionlib
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

TIMESTEP = 0.01
TIME = 10


if __name__ == "__main__":
    rospy.init_node("position_joint_trajectory_controller_test")

    # Create action client.
    follow_joint_traj_client = actionlib.SimpleActionClient(
        "/panda/position_joint_trajectory_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )

    # Waits until the action server has started up and started
    # listening for goals.
    retval = follow_joint_traj_client.wait_for_server(timeout=rospy.Duration(5))
    if not retval:
        ros_exit_gracefully(
            shutdown_msg=f"Shutting down '{rospy.get_name()}'", exit_code=1
        )

    # Get starting state.
    joint_states = rospy.wait_for_message("/panda/joint_states", JointState)
    joint_states_dict = dict(zip(joint_states.name, joint_states.position))

    # Create action client goal.
    header = Header()
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
    follow_joint_traj_client.send_goal(goal)
    pub = rospy.Publisher("/panda/control_signal", JointState, queue_size=10)
    rate = rospy.Rate(1 / TIMESTEP)
    t_start = time.time()
    while time.time() - t_start < TIME and not rospy.is_shutdown():
        msg = JointState(position=[2.9671 * np.sin(time.time() - t_start)])
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    follow_joint_traj_client.wait_for_result()
    result = follow_joint_traj_client.get_result()
    print(result)
