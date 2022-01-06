#!/usr/bin/env python
"""Small python ROS node that can be used to set the logging level of the
'franka_gripper' node."""

import argparse
import rospy
from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest

if __name__ == "__main__":
    rospy.init_node("set_franka_gripper_logger_level")

    # Get input argument
    parser = argparse.ArgumentParser(description="Retrieve log level")
    parser.add_argument(
        "-l", "--level", nargs="?", type=str, default="warn", help="logging level"
    )
    args = parser.parse_args()

    # Call '/gazebo/set_logger_level' service
    try:
        rospy.wait_for_service("/gazebo/set_logger_level", timeout=10)
        set_logger_level_srv = rospy.ServiceProxy(
            "/gazebo/set_logger_level", SetLoggerLevel
        )
        set_logger_level_srv(
            SetLoggerLevelRequest(
                logger="ros.franka_gazebo.FrankaGripperSim", level=args.level
            )
        )
    except rospy.ServiceException as e:
        print("Service call '/gazebo/set_logger_level' failed: %s" % e)