"""Class used to store information about the currently controlled joints."""
import copy

CONTROLLED_JOINTS_DICT = {
    "effort": {"arm": [], "hand": [], "both": []},
    "position": {"arm": [], "hand": [], "both": []},
    "trajectory": {"arm": [], "hand": [], "both": []},
}


class ControlledJointsDict(dict):
    """Used for storing information about the currently controlled joints.
    This class overloads the normal ``dict`` class in order to pre-initialise the
    dictionary with the needed keys.
    """

    def __init__(self, *args, **kwargs):
        """Initialise the ControllerInfoDict."""
        super().__init__(*args, **kwargs)
        super().update(copy.deepcopy(CONTROLLED_JOINTS_DICT))
