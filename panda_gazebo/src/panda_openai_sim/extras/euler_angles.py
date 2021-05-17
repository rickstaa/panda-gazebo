"""Class for storing euler angles. It uses the ypr (x-y-z) euler angle convention.
"""


#################################################
# EulerAngles ###################################
#################################################
class EulerAngles(object):
    """Used for storing euler angles.

    Attributes
    ----------
    y : float
        Yaw angle (z)
    p : float
        Pitch angle (p), by default 0.0
    r : float
        Roll angle (r), by default 0.0
    """

    def __init__(self, y=0.0, p=0.0, r=0.0):
        """Initializes the EulerAngles object.

        Parameters
        ----------
        y : float, optional
            Yaw angle (z), by default 0.0
        p : float, optional
            Pitch angle (p), by default 0.0
        r : float, optional
            Roll angle (r), by default 0.0
        """

        # Create class attributes
        self.y = y
        self.p = p
        self.r = r
