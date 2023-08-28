#!/usr/bin/env python3
"""Small example python script that publishes a OverlayText."""
import math

import rospy

# from random import random, choice
from geometry_msgs.msg import Point

# import math
from jsk_rviz_plugins.msg import OverlayText

if __name__ == "__main__":
    rospy.init_node("pictogram_object_demo_node")

    cur_location = destination = Point()
    p = rospy.Publisher("/text_overlay", OverlayText, queue_size=1)
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        overlay = OverlayText()
        text = "Reward: 12345"
        text_size = 0.0
        overlay.bg_color.r = 0.0
        overlay.bg_color.g = 0.0
        overlay.bg_color.b = 0.0
        overlay.bg_color.a = 1.0
        overlay.fg_color.r = 1.0
        overlay.fg_color.g = 1.0
        overlay.fg_color.b = 1.0
        overlay.fg_color.a = 1.0
        overlay.text_size = text_size
        overlay.text = text
        overlay.action = OverlayText.ADD
        # overlay.width = math.ceil(len(text) * text_size * 0.731)
        # overlay.height = math.ceil(text_size * 3 / 2)
        overlay.width = 50
        overlay.height = 100
        overlay.top = 10
        overlay.left = 10

        p.publish(overlay)
        r.sleep()
