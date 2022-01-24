#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
joint positions.
"""
import rospy
from dynamic_reconfigure.server import Server
from panda_gazebo.cfg import JointPositionConfig
from std_msgs.msg import Float64
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal
from sensor_msgs.msg import JointState


class JointPositionDynamicReconfigureServer:
    def __init__(self):
        self.srv = Server(JointPositionConfig, self.callback)

        # Create joint position publishers
        self.arm_joint1_pub = rospy.Publisher(
            "panda_arm_joint1_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint2_pub = rospy.Publisher(
            "panda_arm_joint2_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint3_pub = rospy.Publisher(
            "panda_arm_joint3_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint4_pub = rospy.Publisher(
            "panda_arm_joint4_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint5_pub = rospy.Publisher(
            "panda_arm_joint5_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint6_pub = rospy.Publisher(
            "panda_arm_joint6_position_controller/command", Float64, queue_size=10
        )
        self.arm_joint7_pub = rospy.Publisher(
            "panda_arm_joint7_position_controller/command", Float64, queue_size=10
        )
        self.arm_pubs = [
            self.arm_joint1_pub,
            self.arm_joint2_pub,
            self.arm_joint3_pub,
            self.arm_joint4_pub,
            self.arm_joint5_pub,
            self.arm_joint6_pub,
            self.arm_joint7_pub,
        ]
        self.gripper_move_client = actionlib.SimpleActionClient(
            "franka_gripper/move", MoveAction
        )
        self.gripper_connected = self.gripper_move_client.wait_for_server(
            timeout=rospy.Duration(secs=5)
        )

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
                "{joint6_position}, {joint7_position}, {width}, {speed}"
            ).format(**config)
        )

        # Set initial joint states
        if level == -1:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            position_dict = dict(zip(joint_states.name, joint_states.position))
            set_values = list(position_dict.values())[:-2]
            set_values.append(list(position_dict.values())[-1] * 2)
            config.update(
                **dict(
                    zip([key for key in config.keys() if key != "groups"], set_values)
                )
            )

        # Write joint positions to controller topics
        if level > -1 and level < 7:
            self.arm_pubs[level].publish(list(config.values())[level])
        elif level in [7, 8] and self.gripper_connected:
            move_goal = MoveGoal(width=config["width"], speed=config["speed"])
            self.gripper_move_client.send_goal(move_goal)
            result = self.gripper_move_client.wait_for_result()
            if not result:
                rospy.logerr("Something went wrong while setting the gripper commands.")
        elif level in [7, 8] and not self.gripper_connected:
            rospy.logwarn_once(
                "Gripper commands not applied since the gripper command action was not "
                "found."
            )
        return config


if __name__ == "__main__":
    rospy.init_node("joint_position_reconfig_server", anonymous=False)

    position_dyn_server = JointPositionDynamicReconfigureServer()

    rospy.spin()
