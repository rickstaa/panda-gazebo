"""Class used to store information about the Gazebo controllers.
"""

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
    """Used for storing information about the Gazebo robot controllers.
    This class overloads the normal `dict` class in order to pre-initialize the
    dictionary with the needed keys.
    """

    def __init__(self, *args, **kwargs):
        """Initiate the ControllerInfoDict"""
        super().__init__(*args, **kwargs)
        super().update(copy.deepcopy(CONTROLLER_INFO_DICT))
