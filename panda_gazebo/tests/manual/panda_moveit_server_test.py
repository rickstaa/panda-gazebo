#!/usr/bin/env python3
"""This script can be used to manually test the 'panda_moveit_server' services"""
import rospy
from geometry_msgs.msg import Pose, Quaternion

from panda_gazebo.msg import BoundingRegion
from panda_gazebo.srv import (
    AddBox,
    AddBoxRequest,
    AddPlane,
    AddPlaneRequest,
    GetEe,
    GetEePose,
    GetEePoseJointConfig,
    GetEePoseJointConfigRequest,
    GetEePoseRequest,
    GetEeRequest,
    GetEeRpy,
    GetEeRpyRequest,
    GetMoveItControlledJoints,
    GetMoveItControlledJointsRequest,
    GetRandomEePose,
    GetRandomEePoseRequest,
    GetRandomJointPositions,
    GetRandomJointPositionsRequest,
    SetEe,
    SetEePose,
    SetEePoseRequest,
    SetEeRequest,
    SetJointPositions,
    SetJointPositionsRequest,
)

if __name__ == "__main__":
    rospy.init_node("test_panda_moveit_server")

    # # -- Test set robot joint positions service --
    # req = SetJointPositionsRequest()
    # req.joint_positions = [
    #     2.15,
    #     1.12,
    #     -1.59,
    #     -1.94,
    #     -2.44,
    #     1.88,
    #     1.54,
    #     0.02,
    #     0.02,
    # ]
    # req.joint_names = [
    #     "panda_joint1",
    #     "panda_joint2",
    #     "panda_joint3",
    #     "panda_joint4",
    #     "panda_joint5",
    #     "panda_joint6",
    #     "panda_joint7",
    #     "panda_finger_joint1",
    #     "panda_finger_joint2",
    # ]
    # set_joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/set_joint_positions", SetJointPositions
    # )
    # resp = set_joint_positions_srv.call(req)
    # print(resp.message)

    # # -- Test panda_arm set robot joint positions service --
    # req = SetJointPositionsRequest()
    # req.joint_positions = [-0.60, 0.33, -0.83, -1.46, 0.26, 1.69, -0.64, 0.016, 0.016]
    # # req.joint_names = ["panda_joint1", "panda_joint2"]
    # setarm__joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_joint_positions", SetJointPositions
    # )
    # resp = setarm__joint_positions_srv.call(req)
    # print(resp.message)

    # # -- Test set panda_hand robot joint positions service --
    # req = SetJointPositionsRequest()
    # req.joint_positions = [0.0, 0.0]
    # # req.joint_positions = [0.03]
    # # req.joint_names = ["panda_finger_joint1"]
    # set_hand_joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_hand/set_joint_positions",
    #     SetJointPositions,
    # )
    # resp = set_hand_joint_positions_srv.call(req)
    # print(resp.message)

    # # -- Test set ee pose service --
    # req = SetEePoseRequest()
    # req.pose.position.x = 0
    # req.pose.position.y = 0.5
    # req.pose.position.z = 0.5
    # set_ee_pose_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_ee_pose", SetEePose
    # )
    # resp = set_ee_pose_srv.call(req)
    # print(resp.message)

    # # -- Test get ee pose service --
    # req = GetEePoseRequest()
    # get_ee_pose_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee_pose", GetEePose
    # )
    # resp = get_ee_pose_srv.call(req)
    # print(resp.message)

    # # -- Test get ee rpy service --
    # req = GetEeRpyRequest()
    # get_ee_rpy_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee_rpy", GetEeRpy
    # )
    # resp = get_ee_rpy_srv.call(req)
    # print(resp.message)

    # # -- Test get ee service --
    # req = GetEeRequest()
    # get_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee", GetEe
    # )
    # resp = get_ee_srv.call(req)
    # print(resp.message)

    # # -- Test set ee service --
    # req = SetEeRequest()
    # req.ee_name = "panda_link0"
    # set_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_ee", SetEe
    # )
    # resp = set_ee_srv.call(req)
    # print(resp.message)

    # # Set back.
    # req = SetEeRequest()
    # req.ee_name = "panda_link8"
    # set_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_ee", SetEe
    # )
    # resp = set_ee_srv.call(req)

    # # -- Test get random joint positions service --
    # req = GetRandomJointPositionsRequest()
    # req.joint_limits.names = [
    #     "panda_joint1_min",
    #     "panda_joint1_max",
    #     "panda_finger_joint1_min",
    #     "panda_finger_joint1_max",
    # ]
    # req.joint_limits.values = [0.0, 0.1, 0.0, 0.04]
    # get_random_joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/get_random_joint_positions",
    #     GetRandomJointPositions,
    # )
    # resp = get_random_joint_positions_srv.call(req)
    # print(resp.message)

    # -- Test get random pose service --
    req = GetRandomEePoseRequest()
    req.bounding_region = BoundingRegion(x_min=0.0, x_max=0.5)
    get_random_ee_pose_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/get_random_ee_pose",
        GetRandomEePose,
    )
    resp = get_random_ee_pose_srv.call(req)
    print(resp.message)

    # # -- Test get ee pose joint config service --
    # req = GetEePoseJointConfigRequest()
    # req.pose.position.x = 0
    # req.pose.position.y = 0.5
    # req.pose.position.z = 0.5
    # get_ee_pose_joint_config_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee_pose_joint_config",
    #     GetEePoseJointConfig,
    # )
    # resp = get_ee_pose_joint_config_srv.call(req)
    # print(resp.joint_names)
    # print(resp.joint_positions)
    # print(resp.message)

    # # -- Test get controlled joints service --
    # req = GetMoveItControlledJointsRequest()
    # get_controlled_joints_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/get_controlled_joints",
    #     GetMoveItControlledJoints,
    # )
    # resp = get_controlled_joints_srv.call(req)
    # print(resp.message)

    # # -- Test add Box service --
    # # req = AddBoxRequest()
    # req = AddBoxRequest(
    #     name="box", pose=Pose(orientation=Quaternion(0, 0, 0, 1)), size=[1, 1, 1]
    # )
    # add_box_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/planning_scene/add_box",
    #     AddBox,
    # )
    # resp = add_box_srv.call(req)
    # print(resp.message)

    # # -- Test add plane service --
    # # req = AddPlaneRequest()
    # req = AddPlaneRequest(name="plane", pose=Pose(orientation=Quaternion(0, 0, 0, 1)))
    # add_plane_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/planning_scene/add_plane",
    #     AddPlane,
    # )
    # resp = add_plane_srv.call(req)
    # print(resp.message)
