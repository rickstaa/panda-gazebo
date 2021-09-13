#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
joint positions.
"""
import rospy
from dynamic_reconfigure.server import Server
from panda_gazebo.cfg import JointPositionConfig
from std_msgs.msg import Float64


class JointPositionDynamicReconfigureServer:
    def __init__(self):
        self.srv = Server(JointPositionConfig, self.callback)

        # Create joint position publishers
        self.arm_joint1_pub = rospy.Publisher(
            "/panda_arm_joint1_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint2_pub = rospy.Publisher(
            "/panda_arm_joint2_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint3_pub = rospy.Publisher(
            "/panda_arm_joint3_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint4_pub = rospy.Publisher(
            "/panda_arm_joint4_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint5_pub = rospy.Publisher(
            "/panda_arm_joint5_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint6_pub = rospy.Publisher(
            "/panda_arm_joint6_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint7_pub = rospy.Publisher(
            "/panda_arm_joint7_position_controller/command", Float64, queue_size=10
        )
        self.hand_finger_joint1_pub = rospy.Publisher(
            "/panda_hand_finger1_position_controller/command", Float64, queue_size=10
        )
        self.hand_finger_joint2_pub = rospy.Publisher(
            "/panda_hand_finger2_position_controller/command", Float64, queue_size=10
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
                "Reconfigure Request: {joint1_position}, {joint2_position}, "
                "{joint3_position} {joint4_position}, {joint5_position}, "
                "{joint6_position}, {joint7_position}"
            ).format(**config)
        )

        # Write joint positions to controller topics
        if level != -1:
            self.pubs[level].publish(list(config.values())[level])

        return config


if __name__ == "__main__":
    rospy.init_node(
        "joint_position_control_test_dynamic_reconfigure_server_test", anonymous=False
    )

    position_dyn_server = JointPositionDynamicReconfigureServer()

    rospy.spin()
