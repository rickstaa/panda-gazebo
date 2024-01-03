"""Small script that can be used to checkout how the robot moves when sending
a sinusoid command to one of the panda joints using the position joint group controller.
"""
import time

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":
    rospy.init_node("position_group_joint_controller_test")

    # Create joint controller publisher.
    arm_joint_group_controller_pub = rospy.Publisher(
        "/panda_arm_joint_position_controller/command",
        Float64MultiArray,
        queue_size=10,
    )

    # Send sinonoidal command to two joints.
    t_start = time.time()
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = [
            0,
            0,
            2.9671 * np.sin(time.time() - t_start),
            2.9671 * np.sin(time.time() - t_start),
            0,
            0,
            0,
        ]
        arm_joint_group_controller_pub.publish(msg)
        time.sleep(0.01)
    print("Done!")
