#!/usr/bin/env python
"""Small python ROS node that can be used to set the logging level of a ROS logger."""
import argparse
import sys

import rospy
from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest

if __name__ == "__main__":
    rospy.init_node("set_logger_level")

    # Get input arguments.
    parser = argparse.ArgumentParser(description="Set logging level of a ROS logger.")
    parser.add_argument(
        "-l", "--level", nargs="?", type=str, default="info", help="logging level"
    )
    parser.add_argument(
        "-n", "--name", nargs="?", type=str, required=True, help="logger name"
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # Call '/gazebo/set_logger_level' service
    try:
        rospy.wait_for_service("/gazebo/set_logger_level", timeout=10)
        set_logger_level_srv = rospy.ServiceProxy(
            "/gazebo/set_logger_level", SetLoggerLevel
        )
        set_logger_level_srv(SetLoggerLevelRequest(logger=args.name, level=args.level))
    except rospy.ServiceException as e:
        print("Service call '/gazebo/set_logger_level' failed: %s" % e)
