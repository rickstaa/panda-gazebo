#!/usr/bin/env python
"""Small python ROS node that can be used to set the logging level of the
'franka_gazebo' node."""

import argparse
import sys
import rospy
from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest

if __name__ == "__main__":
    rospy.init_node("set_franka_gazebo_logger_level")

    # Get input argument
    parser = argparse.ArgumentParser(description="Retrieve log level")
    parser.add_argument(
        "-l", "--level", nargs="?", type=str, default="error", help="logging level"
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # Call '/gazebo/set_logger_level' service
    try:
        rospy.wait_for_service("/gazebo/set_logger_level", timeout=10)
        set_logger_level_srv = rospy.ServiceProxy(
            "/gazebo/set_logger_level", SetLoggerLevel
        )
        set_logger_level_srv(
            SetLoggerLevelRequest(logger="ros.franka_gazebo", level=args.level)
        )
    except rospy.ServiceException as e:
        print("Service call '/gazebo/set_logger_level' failed: %s" % e)
