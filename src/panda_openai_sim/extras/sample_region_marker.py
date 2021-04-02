"""Class used for displaying the a sampling region in rviz. This class overloads
the :visualization_msgs:`visualization_msgs.msgs.Marker <html/msg/Marker.html>`
class in order to pre-initialize some of its attributes. It further also adds the
ability to specify the marker scale using ``x``, ``y``, ``z`` max and min values. To
visualize the marker you can publish it on the ``panda_openai_sim/goal_region`` topic.
"""

# Main python 2/3 compatibility imports
from builtins import super

# Main python imports
import sys

from panda_openai_sim.extras import Quaternion

# ROS python imports
import rospy
from rospy.exceptions import ROSInitException

# ROS msgs and srvs
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA


#################################################
# SampleRegionMarker ####################################
#################################################
class SampleRegionMarker(Marker):
    """Class used to create an rviz goal sample region marker.

    Attributes
    ----------
    x_min : float
        The min x position of the marker.
    y_min : float
        The min y position of the marker.
    z_min : float
        The min z position of the marker.
    x_max : float
        The max x position of the marker.
    y_max : float
        The max y position of the marker.
    z_max :
        The max z position of the marker.
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

    def __init__(
        self,
        x_min=0.0,
        y_min=0.0,
        z_min=0.0,
        x_max=0.0,
        y_max=0.0,
        z_max=0.0,
        *args,
        **kwds
    ):
        """Initialize SampleRegionMarker object"""

        # Initiate class attributes
        self.__x_min = 0.0
        self.__x_max = 0.0
        self.__y_min = 0.0
        self.__z_min = 0.0
        self.__y_max = 0.0
        self.__z_max = 0.0

        # Initiate property overloaded superclass attributes
        self.__scale = Vector3()
        self.__pose = Pose()

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
                    "Goal sample region marker could not be created as the ROS time is "
                    "not initialized. Have you called init_node()?"
                )
                sys.exit(0)
            self.header.frame_id = "world"
        if "pose" not in kwds.keys():
            self.pose = Pose()
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0
        if "color" not in kwds.keys():
            self.color = ColorRGBA()
            self.color.a = 0.1
            self.color.r = 1.0
            self.color.g = 0.0
            self.color.b = 0.0
        self.id = 1 if "id" not in kwds.keys() else self.id
        self.type = Marker.CUBE if "type" not in kwds.keys() else self.type
        self.action = Marker.ADD if "action" not in kwds.keys() else self.action
        self.lifetime = (
            rospy.Duration(-1) if "lifetime" not in kwds.keys() else self.lifetime
        )

        # Add class attributes
        self.__x_min = x_min
        self.__x_max = x_max
        self.__y_min = y_min
        self.__z_min = z_min
        self.__y_max = y_max
        self.__z_max = z_max

        # Adjust scale based on x, y, z max values
        self.__scale.x = abs(self.x_max - self.x_min)
        self.__scale.y = abs(self.y_max - self.y_min)
        self.__scale.z = abs(self.z_max - self.z_min)

        # Adjust pose based on x, y, z max values
        self.__pose.position.x = self.scale.x / 2 + self.x_min
        self.__pose.position.y = self.scale.y / 2 + self.y_min
        self.__pose.position.z = self.scale.z / 2 + self.z_min

        # Make sure the position quaternion is normalized
        self.__pose.orientation = Quaternion.normalize_quaternion(self.pose.orientation)

    @property
    def x_min(self):
        """Retrieve the value of x_min."""
        return self.__x_min

    @property
    def y_min(self):
        """Retrieve the value of y_min."""
        return self.__y_min

    @property
    def z_min(self):
        """Retrieve the value of z_min."""
        return self.__z_min

    @property
    def x_max(self):
        """Retrieve the value of x_max."""
        return self.__x_max

    @property
    def y_max(self):
        """Retrieve the value of y_max."""
        return self.__y_max

    @property
    def z_max(self):
        """Retrieve the value of z_max."""
        return self.__z_max

    @x_min.setter
    def x_min(self, val):
        """Set the value of x_min and update the x scale and position."""
        self.__x_min = val
        self.scale.x = abs(self.x_max - self.x_min)
        self.pose.position.x = self.scale.x / 2 + self.x_min

    @y_min.setter
    def y_min(self, val):
        """Set the value of y_min and update the y scale and position."""
        self.__y_min = val
        self.scale.y = abs(self.y_max - self.y_min)
        self.pose.position.y = self.scale.y / 2 + self.y_min

    @z_min.setter
    def z_min(self, val):
        """Set the value of z_min and update the z scale and position."""
        self.__z_min = val
        self.scale.z = abs(self.z_max - self.z_min)
        self.pose.position.z = self.scale.z / 2 + self.z_min

    @x_max.setter
    def x_max(self, val):
        """Set the value of x_max and update the x scale and position."""
        self.__x_max = val
        self.scale.x = abs(self.x_max - self.x_min)
        self.pose.position.x = self.scale.x / 2 + self.x_min

    @y_max.setter
    def y_max(self, val):
        """Set the value of y_max and update the y scale and position."""
        self.__y_max = val
        self.scale.x = abs(self.y_max - self.y_min)
        self.pose.position.y = self.scale.y / 2 + self.y_min

    @z_max.setter
    def z_max(self, val):
        """Set the value of z_max and update the z scale and position."""
        self.__z_max = val
        self.scale.z = abs(self.z_max - self.z_min)
        self.pose.position.z = self.scale.z / 2 + self.z_min

    @property
    def scale(self):
        """Retrieve scale attribute."""
        return self.__scale

    @scale.setter
    def scale(self, val):
        """Set scale attribute and update the pose and x,y z min and max."""
        self.__scale = val
        self.x_min = self.pose.position.x - self.scale.x / 2
        self.x_max = self.pose.position.x + self.scale.x / 2
        self.y_min = self.pose.position.y - self.scale.y / 2
        self.y_max = self.pose.position.y + self.scale.y / 2
        self.z_min = self.pose.position.z - self.scale.z / 2
        self.z_max = self.pose.position.z + self.scale.z / 2

    @property
    def pose(self):
        """Retrieve pose attribute."""
        return self.__pose

    @pose.setter
    def pose(self, val):
        """Set scale attribute and update the pose and x,y z min and max."""
        self.__pose = val
        self.x_min = self.pose.position.x - self.scale.x / 2
        self.y_min = self.pose.position.y + self.scale.x / 2
        self.z_min = self.pose.position.z - self.scale.x / 2
        self.x_max = self.pose.position.x + self.scale.x / 2
        self.y_max = self.pose.position.y - self.scale.x / 2
        self.z_max = self.pose.position.z + self.scale.x / 2
