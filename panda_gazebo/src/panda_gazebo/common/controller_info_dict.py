"""Class used to store information about the Gazebo controllers."""
import copy

CONTROLLER_INFO_DICT = {
    "arm": {
        "running": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
        "loaded": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
        "stopped": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
    },
    "hand": {
        "running": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
        "loaded": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
        "stopped": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
    },
    "other": {
        "running": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
        "loaded": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
        "stopped": {
            "end_effector": [],
            "trajectory": [],
            "position": [],
            "effort": [],
        },
    },
}


class ControllerInfoDict(dict):
    """Used for storing information about the Gazebo robot controllers. This class
    overloads the normal :obj:`dict` class in order to pre-initialise the dictionary
    with the required keys to store this information.
    """

    def __init__(self, *args, **kwargs):
        """Initialise the ControllerInfoDict.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        super().__init__(*args, **kwargs)
        super().update(copy.deepcopy(CONTROLLER_INFO_DICT))
