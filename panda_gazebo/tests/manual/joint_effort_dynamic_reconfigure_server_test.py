#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
joint efforts.

.. note::
    Please make sure to uncomment the
"""
import rospy
from dynamic_reconfigure.server import Server
from panda_gazebo.cfg import JointEffortConfig
from std_msgs.msg import Float64


class JointEffortDynamicReconfigureServer:
    def __init__(self):
        self.srv = Server(JointEffortConfig, self.callback)

        # Create joint effort publishers
        self.arm_joint1_pub = rospy.Publisher(
            "/panda_arm_joint1_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint2_pub = rospy.Publisher(
            "/panda_arm_joint2_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint3_pub = rospy.Publisher(
            "/panda_arm_joint3_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint4_pub = rospy.Publisher(
            "/panda_arm_joint4_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint5_pub = rospy.Publisher(
            "/panda_arm_joint5_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint6_pub = rospy.Publisher(
            "/panda_arm_joint6_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint7_pub = rospy.Publisher(
            "/panda_arm_joint7_effort_controller/command", Float64, queue_size=10
        )
        self.hand_finger_joint1_pub = rospy.Publisher(
            "/panda_hand_finger1_effort_controller/command", Float64, queue_size=10
        )
        self.hand_finger_joint2_pub = rospy.Publisher(
            "/panda_hand_finger2_effort_controller/command", Float64, queue_size=10
        )
        self.pubs = [
            self.arm_joint1_pub,
            self.arm_joint2_pub,
            self.arm_joint3_pub,
            self.arm_joint4_pub,
            self.arm_joint5_pub,
            self.arm_joint6_pub,
            self.arm_joint7_pub,
            self.hand_finger_joint1_pub,
            self.hand_finger_joint2_pub,
        ]

    def callback(self, config, level):
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
                "Reconfigure Request: {joint1_effort}, {joint2_effort}, "
                "{joint3_effort} {joint4_effort}, {joint5_effort}, {joint6_effort}, "
                "{joint7_effort}"
            ).format(**config)
        )

        # Write joint efforts to controller topics
        if level != -1:
            self.pubs[level].publish(list(config.values())[level])

        return config


if __name__ == "__main__":
    rospy.init_node(
        "joint_effort_control_test_dynamic_reconfigure_server_test", anonymous=False
    )

    effort_dyn_server = JointEffortDynamicReconfigureServer()

    rospy.spin()
