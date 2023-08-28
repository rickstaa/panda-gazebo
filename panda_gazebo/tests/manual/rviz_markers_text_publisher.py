#!/usr/bin/env python3
"""Small python script that publishes a marker."""
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class MarkerBasics(object):
    def __init__(self):
        self.marker_object_objectlister = rospy.Publisher(
            "/marker_basic", Marker, queue_size=1
        )
        self.rate = rospy.Rate(1)
        self.init_marker()

    def init_marker(self):
        self.marker_object = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=1,
            scale=Vector3(0.0, 0.0, 0.25),
            lifetime=rospy.Duration(1),
            text="Reward: 123453454",
            color=ColorRGBA(0, 100, 100, 1.0),
            header=Header(frame_id="world"),
            pose=Pose(Point(0.0, 3, 3), Quaternion(0, 0, 0, 1)),
        )

    def start(self):
        while not rospy.is_shutdown():
            self.marker_object_objectlister.publish(self.marker_object)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("marker_basic_node", anonymous=True)
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass
