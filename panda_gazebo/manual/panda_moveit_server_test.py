"""This script can be used to manually test the 'panda_moveit_server' services"""

# Import ROS related python packages
import rospy

# Import ROS msgs and srvs
from panda_openai_sim.msg import BoundingRegion
from panda_openai_sim.srv import (
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
)

# Main function
if __name__ == "__main__":

    # Initiate ROS node
    rospy.init_node("test_panda_moveit_server")

    # # # -- Test set robot joint positions service --
    # req = SetJointPositionsRequest()
    # req.joint_positions = [0.02, 0.02]
    # # req.joint_positions = [0.00]
    # req.joint_names = [
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
    # # req.joint_positions = [
    # #     1.499895698181251,
    # #     1.503605675322719,
    # #     1.498779085379117,
    # #     -0.07011811940859314,
    # #     1.5185806604171868,
    # #     1.5004606311218627,
    # #     1.4998977401721056,
    # # ]
    # req.joint_positions = [0.5, 0.5, 0, 5, 4, 3, 2]
    # # req.joint_names = ["panda_joint1", "panda_joint2"]
    # setarm__joint_positions_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_joint_positions", SetJointPositions
    # )
    # resp = setarm__joint_positions_srv.call(req)
    # print(resp)

    # # -- Test set panda_hand robot joint positions service --
    # req = SetJointPositionsRequest()
    # req.joint_positions = [0.23, 0.03]
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
    req.pose.position.y = 0
    req.pose.position.z = 1.5
    set_ee_pose_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/set_ee_pose", SetEePose
    )
    resp = set_ee_pose_srv.call(req)
    print(resp)

    # # -- Test get ee pose service --
    # req = GetEePoseRequest()
    # get_ee_pose_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee_pose", GetEePose
    # )
    # resp = get_ee_pose_srv.call(req)
    # print(resp)

    # # -- Test get ee rpy service --
    # req = GetEeRpyRequest()
    # get_ee_rpy_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee_rpy", GetEeRpy
    # )
    # resp = get_ee_rpy_srv.call(req)
    # print(resp)

    # # -- Test get ee service --
    # req = GetEeRequest()
    # get_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/get_ee", GetEe
    # )
    # resp = get_ee_srv.call(req)
    # print(resp)

    # # -- Test set ee service --
    # req = SetEeRequest()
    # req.ee_name = "panda_link0"
    # set_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_ee", SetEe
    # )
    # resp = set_ee_srv.call(req)
    # print(resp)

    # # -- Test get random joint positions service --
    # req = GetRandomJointPositionsRequest()
    # set_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/get_random_joint_positions",
    #     GetRandomJointPositions,
    # )
    # resp = set_ee_srv.call(GetRandomJointPositionsRequest())
    # print(resp)

    # # -- Test get random pose service --
    # req = GetRandomEePoseRequest()
    # req.bounding_region = BoundingRegion(x_min=0.0, x_max=1.0)
    # set_ee_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/get_random_ee_pose", GetRandomEePose,
    # )
    # resp = set_ee_srv.call(req)
    # print(resp)
