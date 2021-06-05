#!/usr/bin/env python3
"""Script that creates a dynamic parameter server which can be used to test out the
Panda position_controllers
"""

import rospy
from dynamic_reconfigure.server import Server
from panda_gazebo.cfg import JointPositionConfig
from panda_gazebo.srv import SetJointPositions, SetJointPositionsRequest


class JointPositionTester(object):
    """Class used for testing whether the joint positions work.

    Attributes:
        :obj:`rospy.impl.tcpros_service.ServiceProxy`: Joint positions server client.
        :obj:`dynamic_reconfigure.server.Server`: Dynamic reconfigure server client.
    """

    def __init__(self):
        """Initiate joint tester instance."""
        rospy.logdebug(
            "Connecting to '/panda_control_server/set_joint_positions' service."
        )
        rospy.wait_for_service("/panda_control_server/set_joint_positions", timeout=10)
        self.set_joint_positions_client = rospy.ServiceProxy(
            "/panda_control_server/set_joint_positions", SetJointPositions
        )
        rospy.logdebug(
            "Connected to 'panda_control_server/set_joint_positions' service!"
        )

        # Generate joint_positions msg
        self.dynamic_reconfig_srv = Server(
            JointPositionConfig, self._dynamic_reconfig_callback
        )

    def _dynamic_reconfig_callback(self, config, level):
        """The callback function for the dynamic reconfigure server

        Args:
            config (:obs:`dynamic_reconfigure.encoding.Config`): Dynamic reconfigure
                config dictionary.
            level (int): Dynamic reconfigure level parameter indicating which
                parameter has been changed.

        Returns:
            :obs:`dynamic_reconfigure.encoding.Config`: Dynamic reconfigure config
                dictionary.
        """
        rospy.loginfo(
            "Joint positions reconfigure Request: [{finger_joint1_position}, "
            "{finger_joint2_position}, {joint1_position}, {joint2_position}, "
            "{joint3_position}, {joint4_position}, {joint5_position}, "
            "{joint6_position}, {joint7_position}]".format(**config)
        )

        # Put changed values on object
        self.config = config
        self.joint_positions_setpoint = [
            self.config["finger_joint1_position"],
            self.config["finger_joint2_position"],
            self.config["joint1_position"],
            self.config["joint2_position"],
            self.config["joint3_position"],
            self.config["joint4_position"],
            self.config["joint5_position"],
            self.config["joint6_position"],
            self.config["joint7_position"],
        ]

        # Call set_joint_positions service
        set_joint_positions_msg = SetJointPositionsRequest()
        set_joint_positions_msg.joint_positions = self.joint_positions_setpoint
        self.set_joint_positions_client.call(set_joint_positions_msg)

        return config


if __name__ == "__main__":
    rospy.init_node("panda_position_control_tester", anonymous=False)

    # Create joint position tester class
    joint_tester = JointPositionTester()

    # Spin till killed
    rospy.spin()
