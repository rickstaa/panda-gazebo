"""Class used for storing and modifying quaternions. It is based on the geometry_msgs
Quaternion class but adds some extra features.
"""

# Main python imports
from numpy import linalg
from numpy import nan
from math import isnan
import copy

# ROS python imports
import rospy

# ROS msgs and srvs
from geometry_msgs.msg import Quaternion as QuaternionParent


#################################################
# Quaternion class ##############################
#################################################
class Quaternion(QuaternionParent):
    """Used for storing and modifying quaternions.

    Attributes
    ----------
    x : float
        The quaternion x coordinate.
    y : float
        The quaternion y coordinate.
    z : float
        The quaternion z coordinate.
    w : float
        The quaternion w coordinate.
    norm : float
        The quaternion norm.

    Methods
    -------
    quaternion_norm(cls, Quaternion):
        Calculates the norm of a quaternion.
    normalize_quaternion(cls, quaternion):
        Normalizes a given quaternion.
    normalize():
        Normalize the quaternion.
    """

    def __init__(self, *args, **kwds):
        """Initiates quaternion object."""

        # Initiate superclass
        super().__init__(*args, **kwds)

    def normalize(self):
        """Normalizes the quaternion."""

        # Retrieve norm
        norm = self.norm

        # Normalize quaternion and return
        if norm == 0.0 or isnan(norm):
            rospy.logwarn(
                "Quaternion could not be normalized since the norm could not be "
                "calculated."
            )
        else:
            self.x = self.x / norm
            self.y = self.y / norm
            self.z = self.z / norm
            self.w = self.w / norm

    @property
    def norm(self):
        """Retrieves the quaternion norm.

        Returns
        -------
        float
            The quaternion norm.
        """
        return self.quaternion_norm(self)

    @classmethod
    def normalize_quaternion(cls, quaternion):
        """Normalizes a given quaternion.

        Parameters
        ----------
        Quaternion : geometry_msgs.msg.Quaternion
            A quaternion.

        Returns
        -------
        geometry_msgs.msg.Quaternion
            The normalized quaternion.
        """

        # Create a hardcopy such that the original object is not modified
        quaternion = copy.deepcopy(quaternion)

        # Retrieve quaternion
        norm = cls.quaternion_norm(quaternion)

        # Normalize quaternion and return
        if norm == nan:
            # test
            rospy.logwarn(
                "Quaternion could not be normalized since the norm could not be "
                "calculated."
            )
        elif norm == 0.0:  # Transform to identity
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = 0.0
            quaternion.w = 1.0
        else:
            quaternion.x = quaternion.x / norm
            quaternion.y = quaternion.y / norm
            quaternion.z = quaternion.z / norm
            quaternion.w = quaternion.w / norm

        # Return quaternion
        return quaternion

    @classmethod
    def quaternion_norm(cls, Quaternion):
        """Calculates the norm of a quaternion.

        Parameters
        ----------
        Quaternion : geometry_msgs.msg.Quaternion
            A quaternion.

        Returns
        -------
        float :
            The norm of the quaternion.
        """
        return linalg.norm([Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w])
