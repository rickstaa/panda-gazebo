"""Class used for displaying a gasp target goal marker in rviz. This class overloads
the :visualization_msgs:`visualization_msgs.msgs.Marker <html/msg/Marker.html>`
class in order to pre-initialize some of its attributes. To visualize the marker you
can publish it on the ``panda_openai_sim/current_goal`` topic.
"""

# Main python 2/3 compatibility imports
from builtins import super

# Main python imports
import sys

# ROS python imports
import rospy
from rospy.exceptions import ROSInitException

from panda_openai_sim.extras import Quaternion

# ROS msgs and srvs
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3


#################################################
# TargetMarker ####################################
#################################################
class TargetMarker(Marker):
    """Class used to create an rviz target goal marker.

    Attributes
    ----------
    id : int
        The marker object id.
    type : str
        The marker type.
    action : float
        The marker message action (add or remove).
    pose : geometry_msgs.Pose
        The marker pose.
    scale : geometry_msgs.Vector3
        The marker scale
    color : std_msgs.ColorRGBA
        The marker color.
    lifetime : duration
        The lifetime duration.
    frame_locked : bool
        Boolean specifying whether the marker frame is locked to the world.
    point : geometry_msgs.Point
        The marker points.
    string : str
        A description.
    mesh_resource : str
        Marker mess location.
    mesh_use_embedded_materials : bool
        Boolean specifying whether we want to use a mesh.
    """

    def __init__(self, *args, **kwds):
        """Initialize TargetMarker object"""

        # Apply superclass initiation
        super().__init__(*args, **kwds)

        # Overwrite attributes with defaults if not supplied in the constructor
        if "header" not in kwds.keys():

            # Pre-initialize header
            self.header = Header()
            try:  # Check if rostime was initialized
                self.header.stamp = rospy.Time.now()
            except ROSInitException:
                raise Exception(
                    "Goal marker could not be created as the ROS time is not "
                    "initialized. Have you called init_node()?"
                )
                sys.exit(0)
            self.header.frame_id = "world"
        if "color" not in kwds.keys():
            self.color = ColorRGBA()
            self.color.a = 1.0
            self.color.r = 1.0
            self.color.g = 0.0
            self.color.b = 0.0
        if "scale" not in kwds.keys():
            self.scale = Vector3()
            self.scale.x = 0.025
            self.scale.y = 0.025
            self.scale.z = 0.025
        self.id = 0 if "id" not in kwds.keys() else self.id
        self.type = Marker.CUBE if "type" not in kwds.keys() else self.type
        self.action = Marker.ADD if "action" not in kwds.keys() else self.action
        self.lifetime = (
            rospy.Duration(-1) if "lifetime" not in kwds.keys() else self.lifetime
        )

        # Make sure the position quaternion is normalized
        self.pose.orientation = Quaternion.normalize_quaternion(self.pose.orientation)
