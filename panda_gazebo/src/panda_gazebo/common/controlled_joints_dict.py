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
    dictionary with the required keys to store this information.
    """

    def __init__(self, *args, **kwargs):
        """Initialise the ControllerInfoDict.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        super().__init__(*args, **kwargs)
        super().update(copy.deepcopy(CONTROLLED_JOINTS_DICT))
