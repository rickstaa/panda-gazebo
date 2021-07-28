#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
joint efforts.
"""
import rospy
from dynamic_reconfigure.server import Server
from panda_gazebo.cfg import JointEffortConfig


def callback(config, level):
    """Dynamic reconfigure callback function.

    Args:
        config (:obj:`dynamic_reconfigure.encoding.Config`): The current dynamic
            reconfigure configuration object.
        level (int): Bitmask that gives information about which parameter has been
            changed.

    Returns:
        :obj:`~dynamic_reconfigure.encoding.Config`: Modified dynamic reconfigure
            configuration object.
    """
    rospy.loginfo(
        (
            "Reconfigure Request: {joint1_effort}, {joint2_effort}, {joint3_effort} "
            "{joint4_effort}, {joint5_effort}, {joint6_effort}, {joint7_effort}"
        ).format(**config)
    )
    return config


if __name__ == "__main__":
    rospy.init_node(
        "joint_effort_control_test_dynamic_reconfigure_server_test", anonymous=False
    )

    srv = Server(JointEffortConfig, callback)
    rospy.spin()
