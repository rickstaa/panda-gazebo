#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
joint positions.
"""
import rospy
from dynamic_reconfigure.server import Server
from panda_gazebo.cfg import JointPositionConfig


def callback(config, level):
    """Dynamic reconfigure callback function.

    Args:
        config (): The current dynamic reconfigure configuration object.
        level ():

    Returns:
        : Modified dynamic reconfigure configuration object.
    """
    rospy.loginfo(
        """Reconfigure Request: {int_param}, {double_param},\
          {str_param}, {bool_param}, {size}""".format(
            **config
        )
    )
    return config


if __name__ == "__main__":
    rospy.init_node(
        "joint_position_control_test_dynamic_reconfigure_srv", anonymous=False
    )

    srv = Server(JointPositionConfig, callback)
    rospy.spin()
