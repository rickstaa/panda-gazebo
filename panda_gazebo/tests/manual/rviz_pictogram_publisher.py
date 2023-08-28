#!/usr/bin/env python3
"""Small example python script that publishes a pictogram."""
import rospy

# from random import random, choice
from geometry_msgs.msg import Point

# import math
from jsk_rviz_plugins.msg import Pictogram, PictogramArray

if __name__ == "__main__":
    rospy.init_node("pictogram_object_demo_node")

    cur_location = destination = Point()
    p = rospy.Publisher("/pictogram_array", PictogramArray, queue_size=1)

    r = rospy.Rate(1)
    pictograms = ["fa-building", "location"]

    while not rospy.is_shutdown():
        arr = PictogramArray()
        arr.header.frame_id = "world"
        arr.header.stamp = rospy.Time.now()
        for index, character in enumerate(pictograms):
            msg = Pictogram()
            msg.header.frame_id = "world"
            msg.action = Pictogram.JUMP
            if index == 0:
                msg.header.stamp = rospy.Time.now()
                msg.pose.position = Point(x=0.0, y=1.0, z=1.0)
            else:
                msg.header.stamp = rospy.Time.now()
                msg.pose.position = Point(x=0.0, y=-1.0, z=1.0)

            # It has to be like this to have them vertically orient the icons.
            msg.pose.orientation.w = 0.7
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = -0.7
            msg.pose.orientation.z = 0
            msg.size = 1
            msg.color.r = 25 / 255.0
            msg.color.g = 255 / 255.0
            msg.color.b = 240 / 255.0
            msg.color.a = 1.0
            msg.character = character
            arr.pictograms.append(msg)

        p.publish(arr)
        r.sleep()
