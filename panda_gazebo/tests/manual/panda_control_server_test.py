#!/usr/bin/env python3
"""Script used to manually test the '/panda_control_server' control services."""

import actionlib
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

from panda_gazebo.common.helpers import ros_exit_gracefully
from panda_gazebo.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from panda_gazebo.srv import (
    GetControlledJoints,
    GetControlledJointsRequest,
    SetGripperWidth,
    SetGripperWidthRequest,
    SetJointCommands,
    SetJointCommandsRequest,
    SetJointEfforts,
    SetJointEffortsRequest,
    SetJointPositions,
    SetJointPositionsRequest,
)

# --TESTS--
# For the `joint effort`, `joint_position` and `joint_trajectory` services test the
# following test trajectory services:
#   1. Empty message -> Success
#   2. Only control commands -> Success
#   3. To little or to much control commands -> Fail
#   4. Wrong joint_names --> Fail
#   5. Commands not equal to joint_names field --> fail
#   6. Equal to each other --> success

if __name__ == "__main__":
    rospy.init_node("panda_control_server_test")

    # -- TEST SET JOINT COMMANDS --

    # %% /panda_control_server/set_joint_commands test

    # Connect to /panda_control_server/set_joint_commands
    rospy.logdebug("Connecting to '/panda_control_server/set_joint_commands' service.")
    rospy.wait_for_service("/panda_control_server/set_joint_commands", timeout=10)
    set_arm_joint_effort_srv = rospy.ServiceProxy(
        "/panda_control_server/set_joint_commands", SetJointCommands
    )
    rospy.logdebug("Connected to '/panda_control_server/set_joint_commands' service!")

    # Generate joint_efforts msg.
    set_joint_commands_msg = SetJointCommandsRequest()
    set_joint_commands_msg.joint_names = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "gripper_width",
        "gripper_max_effort",
    ]
    # set_joint_commands_msg.joint_names = [
    #     "panda_joint1",
    #     "panda_joint3",
    #     "gripper_width",
    #     "gripper_max_effort",
    # ]  # NOTE: WRONG input!
    # set_joint_commands_msg.joint_names = [
    #     "panda_joint33",
    #     "panda_joint2",
    #     "panda_joint3",
    #     "gripper_width",
    #     "gripper_max_effort",
    # ]  # NOTE: WRONG input!
    # set_joint_commands_msg.grasping = False
    set_joint_commands_msg.arm_wait = True
    set_joint_commands_msg.hand_wait = True
    set_joint_commands_msg.control_type = "position"
    # set_joint_commands_msg.control_type = "effort"
    # set_joint_commands_msg.joint_commands = [1, 2, 3, 0.03]  # NOTE: Wrong input!
    set_joint_commands_msg.joint_commands = [1, 2, 3, 0.03, 130]
    set_joint_commands_msg.grasping = True
    resp = set_arm_joint_effort_srv.call(set_joint_commands_msg)
    print(resp.message)

    # -- TEST SET ARM JOINT EFFORTS --

    # %% /panda_control_server/panda_arm/set_joint_efforts test

    # Connect to /panda_control_server/set_joint_efforts
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_arm/set_joint_efforts' service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_arm/set_joint_efforts", timeout=10
    )
    set_arm_joint_effort_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_arm/set_joint_efforts", SetJointEfforts
    )
    rospy.logdebug(
        "Connected to '/panda_control_server/panda_arm/set_joint_efforts' service!"
    )

    # Generate joint_efforts msg.
    set_arm_joint_efforts_msg = SetJointEffortsRequest()
    set_arm_joint_efforts_msg.joint_names = ["panda_joint2", "panda_joint3"]
    set_arm_joint_efforts_msg.joint_efforts = [0, 0]
    # set_arm_joint_efforts_msg.joint_efforts = [0, 0, 0]
    resp = set_arm_joint_effort_srv.call(set_arm_joint_efforts_msg)
    print(resp.message)

    # -- TEST SET ARM JOINT POSITIONS --

    # %% /panda_control_server/panda_arm/set_joint_positions test

    # Connect to /panda_control_server/set_joint_positions
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_arm/set_joint_positions' "
        "service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_arm/set_joint_positions", timeout=10
    )
    set_arm_joint_positions_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_arm/set_joint_positions", SetJointPositions
    )
    rospy.logdebug(
        "Connected to '/panda_control_server/panda_arm/set_joint_positions' " "service!"
    )

    # Generate set_arm_joint_positions_msg.
    set_arm_joint_positions_msg = SetJointPositionsRequest()
    # set_arm_joint_positions_msg.joint_names = ["panda_joint5", "panda_joint6"]
    set_arm_joint_positions_msg.joint_positions = [1.5, 2]
    # set_arm_joint_positions_msg.joint_positions = [
    # 0.0,
    # 0.0,
    # 0.0,
    # 1.5,
    # 1.5,
    # 0.0,
    # 0.0,
    # ]
    # set_arm_joint_positions_msg.joint_positions = [
    #     1.5,
    #     1.0,
    #     1.0,
    #     1.5,
    #     1.5,
    #     1.0,
    #     1.0,
    # ]
    # set_arm_joint_positions_msg.joint_positions = [0.0, 1.5]
    set_arm_joint_positions_msg.wait = True
    resp = set_arm_joint_positions_srv.call(set_arm_joint_positions_msg)
    print(resp.message)

    # -- TEST SET ARM JOINT TRAJ SERVICE --

    # %% /panda_control_server/panda_arm/follow_joint_trajectory test

    # Create action client.
    follow_joint_traj_client = actionlib.SimpleActionClient(
        "/panda_control_server/panda_arm/follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )

    # Waits until the action server has started up and started.
    # listening for goals.
    retval = follow_joint_traj_client.wait_for_server(timeout=rospy.Duration(5))
    if not retval:
        ros_exit_gracefully(
            shutdown_msg=f"Shutting down '{rospy.get_name()}'", exit_code=1
        )

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
        0.5,
        -0.5,
        0.1,
        -0.579418370643596,
        -0.208283306265037815,
        0.3501401410371793,
        0.44266583523250507,
    ]
    point.time_from_start.secs = 1
    goal.trajectory.points.append(point)
    # goal.trajectory.header = header
    # goal.goal_time_tolerance.secs = 5

    # Send goal.
    follow_joint_traj_client.send_goal(goal)
    follow_joint_traj_client.wait_for_result()
    result = follow_joint_traj_client.get_result()
    print(result)

    # -- Test set gripper width service --

    # Without grasp.
    req = SetGripperWidthRequest()
    req.width = 0.04
    req.wait = True
    set_gripper_width_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_gripper_width",
        SetGripperWidth,
    )
    resp = set_gripper_width_srv.call(req)
    print(resp.message)
    req = SetGripperWidthRequest()
    req.width = 0.0
    req.wait = True
    set_gripper_width_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_gripper_width",
        SetGripperWidth,
    )
    resp = set_gripper_width_srv.call(req)
    print(resp.message)

    # With grasp.
    req = SetGripperWidthRequest()
    req.width = 0.04
    req.wait = True
    req.grasping = True
    set_gripper_width_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_gripper_width",
        SetGripperWidth,
    )
    resp = set_gripper_width_srv.call(req)
    print(resp.message)
    req = SetGripperWidthRequest()
    req.grasping = True
    req.width = 0.03
    req.wait = True
    set_gripper_width_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_gripper_width",
        SetGripperWidth,
    )
    resp = set_gripper_width_srv.call(req)
    print(resp.message)

    # Reset grasp.
    req = SetGripperWidthRequest()
    req.width = 0.04
    req.wait = True
    set_gripper_width_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_gripper_width",
        SetGripperWidth,
    )
    resp = set_gripper_width_srv.call(req)
    print(resp.message)

    # -- Test get controlled joints service --
    req = GetControlledJointsRequest()
    req.control_type = "position"
    get_controlled_joints_srv = rospy.ServiceProxy(
        "/panda_control_server/get_controlled_joints",
        GetControlledJoints,
    )
    resp = get_controlled_joints_srv.call(req)
    print(resp.message)
