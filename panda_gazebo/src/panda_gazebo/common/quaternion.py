"""Class used for storing and modifying quaternions. It is based on the geometry_msgs
Quaternion class but adds some extra features.
"""

import copy
from math import isnan

import rospy
from geometry_msgs.msg import Quaternion as QuaternionParent
from numpy import linalg, nan


class Quaternion(QuaternionParent):
    """Used for storing and modifying quaternions.

    Attributes:
        x (float): The quaternion x coordinate.
        y (float): The quaternion y coordinate.
        z (float): The quaternion z coordinate.
        w (float): The quaternion w coordinate.
        norm (float): The quaternion norm.

    Methods:
        quaternion_norm(cls, Quaternion): Calculates the norm of a quaternion.
        normalize_quaternion(cls, quaternion): Normalizes a given quaternion.
        normalize(): Normalize the quaternion.
    """

    def __init__(self, *args, **kwds):
        """Initiates quaternion object."""
        super().__init__(*args, **kwds)

    def normalize(self):
        """Normalizes the quaternion."""
        norm = self.norm
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

        Returns:
            float: The quaternion norm.
        """
        return self.quaternion_norm(self)

    @classmethod
    def normalize_quaternion(cls, quaternion):
        """Normalizes a given quaternion.

        Args:
            quaternion (:obj:`geometry_msgs.msg.Quaternion`): A quaternion.

        Returns:
            :obj:`geometry_msgs.msg.Quaternion`: The normalized quaternion.
        """
        quaternion = copy.deepcopy(quaternion)
        norm = cls.quaternion_norm(quaternion)

        # Normalize quaternion
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
        return quaternion

    @classmethod
    def quaternion_norm(cls, Quaternion):
        """Calculates the norm of a quaternion.

        Args:
            Quaternion (:obj:`geometry_msgs.msg.Quaternion`): A quaternion.

        Returns:
            float: The norm of the quaternion.
        """
        return linalg.norm([Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w])
