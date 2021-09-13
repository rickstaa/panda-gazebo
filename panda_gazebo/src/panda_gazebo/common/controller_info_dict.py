﻿"""Class used to store information about the Gazebo controllers.
"""

import copy

# Script parameters
CONTROLLER_INFO_DICT = {
    "arm": {
        "running": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
        "loaded": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
        "stopped": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
    },
    "hand": {
        "running": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
        "loaded": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
        "stopped": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
    },
    "other": {
        "running": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
        "loaded": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
        },
        "stopped": {
            "ee_control": [],
            "trajectory_control": [],
            "position_control": [],
            "effort_control": [],
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