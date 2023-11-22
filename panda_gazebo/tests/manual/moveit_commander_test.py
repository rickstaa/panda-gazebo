#!/usr/bin/env python3
"""Small script for testing capabilities of the MoveIt commander."""
import sys
import math

from geometry_msgs.msg import Pose
import moveit_commander
import moveit_msgs.msg
import rospy

if __name__ == "__main__":
    # Initialise MoveIt commander and node.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    # Create commanders.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Load arm and hand groups.
    move_group = moveit_commander.MoveGroupCommander("panda_arm")
    hand_move_group = moveit_commander.MoveGroupCommander("hand")

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print(f"============ Planning frame: {planning_frame}")

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print(f"============ End effector link: {eef_link}")

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the.
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    # -- Plan arm joint goal --

    # We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -math.tau / 8
    joint_goal[2] = 0
    joint_goal[3] = -math.tau / 4
    joint_goal[4] = 0
    joint_goal[5] = math.tau / 6  # 1/6 of a turn
    joint_goal[6] = 0

    # METHOD 1
    # -- UNCOMMENT BELOW TO SEE THE SCALING FACTOR IN ACTION --
    # max_vel_scale_factor = 0.2
    # max_vel_scale_factor = 1.0
    # max_acc_scale_factor = 0.2
    # max_acc_scale_factor = 1.0
    # move_group.set_max_velocity_scaling_factor(max_vel_scale_factor)
    # move_group.set_max_acceleration_scaling_factor(max_acc_scale_factor)
    # -- UNCOMMENT ABOVE TO SEE THE SCALING FACTOR IN ACTION --
    move_group.set_joint_value_target(joint_goal)
    move_group.plan()
    move_group.go(wait=True)

    # METHOD 2
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group.
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # -- Plan pose goal --
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    # -- Plan hand goal --
    hand_move_group.set_joint_value_target([0.04, 0.04])
    (hand_plan_retval, plan, _, error_code) = hand_move_group.plan()
    retval = hand_move_group.execute(plan, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
