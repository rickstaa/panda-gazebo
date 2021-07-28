"""This script can be used to manually test the 'panda_moveit_server' services"""

import rospy
from panda_gazebo.msg import BoundingRegion
from panda_gazebo.srv import (
    GetEe,
    GetEePose,
    GetEePoseRequest,
    GetEeRequest,
    GetEeRpy,
    GetEeRpyRequest,
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
    GetMoveItControlledJoints,
    GetMoveItControlledJointsRequest,
)

if __name__ == "__main__":
    rospy.init_node("test_panda_moveit_server")

    # -- Test set robot joint positions service --
    req = SetJointPositionsRequest()
    req.joint_positions = [
        1.499895698181251,
        1.503605675322719,
        1.498779085379117,
        -0.07011811940859314,
        1.5185806604171868,
        1.5004606311218627,
        1.4998977401721056,
        0.02,
        0.02,
    ]
    # req.joint_positions = [
    #     -1.5588153829789288e-06,
    #     0.007296781094924877,
    #     6.0527248901820485e-06,
    #     0.00016249652149724625,
    #     -4.777426150681663e-06,
    #     -0.0698786875740316,
    #     3.394301404391342e-06,
    #     0.5098243409314671,
    #     8.292685809152545e-07,
    # ]  # NOTE: Second pose
    # req.joint_positions = [0.00] # NOTE: Should fail
    req.joint_names = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        "panda_finger_joint1",
        "panda_finger_joint2",
    ]
    set_joint_positions_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/set_joint_positions", SetJointPositions
    )
    resp = set_joint_positions_srv.call(req)
    print(resp.message)

    # # -- Test panda_arm set robot joint positions service --
    # req = SetJointPositionsRequest()
    # req.joint_positions = [
    #     1.499895698181251,
    #     1.503605675322719,
    #     1.498779085379117,
    #     -0.07011811940859314,
    #     1.5185806604171868,
    #     1.5004606311218627,
    #     1.4998977401721056,
    # ]
    # # req.joint_positions = [0.5, 0.5, 0, 5, 4, 3, 2] # NOTE:: Should fail
    # # req.joint_names = ["panda_joint1", "panda_joint2"]
    # setarm__joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_joint_positions", SetJointPositions
    # )
    # resp = setarm__joint_positions_srv.call(req)
    # print(resp)

    # # -- Test set panda_hand robot joint positions service --
    # req = SetJointPositionsRequest()
    # # req.joint_positions = [0.23, 0.03] # NOTE:: Should fail
    # req.joint_positions = [0.03, 0.03]
    # # req.joint_positions = [0.03]
    # # req.joint_names = ["panda_finger_joint1"]
    # set_hand_joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_hand/set_joint_positions", SetJointPositions
    # )
    # resp = set_hand_joint_positions_srv.call(req)
    # print(resp)

    # -- Test set ee pose service --
    req = SetEePoseRequest()
    req.pose.position.x = 0
    req.pose.position.y = 0.5
    req.pose.position.z = 0.5
    set_ee_pose_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/set_ee_pose", SetEePose
    )
    resp = set_ee_pose_srv.call(req)
    print(resp)

    # -- Test get ee pose service --
    req = GetEePoseRequest()
    get_ee_pose_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/get_ee_pose", GetEePose
    )
    resp = get_ee_pose_srv.call(req)
    print(resp)

    # -- Test get ee rpy service --
    req = GetEeRpyRequest()
    get_ee_rpy_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/get_ee_rpy", GetEeRpy
    )
    resp = get_ee_rpy_srv.call(req)
    print(resp)

    # -- Test get ee service --
    req = GetEeRequest()
    get_ee_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/get_ee", GetEe
    )
    resp = get_ee_srv.call(req)
    print(resp)

    # -- Test set ee service --
    req = SetEeRequest()
    req.ee_name = "panda_link0"
    set_ee_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/set_ee", SetEe
    )
    resp = set_ee_srv.call(req)
    print(resp)

    # Set back
    req = SetEeRequest()
    req.ee_name = "panda_link8"
    set_ee_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/set_ee", SetEe
    )
    resp = set_ee_srv.call(req)

    # -- Test get random joint positions service --
    req = GetRandomJointPositionsRequest()
    req.joint_limits.names = ["panda_joint1_min", "panda_joint1_max"]
    req.joint_limits.values = [0.0, 0.5]
    get_random_joint_positions_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/get_random_joint_positions",
        GetRandomJointPositions,
    )
    resp = get_random_joint_positions_srv.call(req)
    print(resp)

    # -- Test get random pose service --
    req = GetRandomEePoseRequest()
    req.bounding_region = BoundingRegion(x_min=0.0, x_max=1.0)
    get_random_ee_pose_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/get_random_ee_pose",
        GetRandomEePose,
    )
    resp = get_random_ee_pose_srv.call(req)
    print(resp)

    # -- Test get controlled joints service --
    req = GetMoveItControlledJointsRequest()
    get_controlled_joints_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/get_controlled_joints",
        GetMoveItControlledJoints,
    )
    resp = get_controlled_joints_srv.call(req)
    print(resp)
