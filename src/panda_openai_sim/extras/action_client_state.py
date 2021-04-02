"""Small class used to store the state of a
actionlib.simple_action_client.SimpleActionClient in a more human friendly format.
"""


#################################################
# ActionClientState class #######################
#################################################
class ActionClientState(object):
    """Class used to save the action client state in a more human friendly format.

    Attributes
    ----------
    action_client : actionlib.simple_action_client.SimpleActionClient
        The action client to which the ActionClientState object is linked.
    state : int
        The current state value of the action client.
    state_string : str
        The name of the current state of the action client.
    state_value_dict : dict
        The dictionary containing all the possible state values of the action client
        with the corresponding states.
    state_dict : dict
        The dictionary containing all the possible states of the action client with
        the corresponding state values.
    """

    def __init__(self, action_client=None, state=-1, state_string=""):
        """Initializes the InputMessageInvalidError exception object.

        Parameters
        ----------
        action_client : actionlib.simple_action_client.SimpleActionClient, optional
            The action client to which you want to link the ActionClientState object,
            by default None.
        state : int, optional
            The state value of the actionclient, by
            default -1.
        state_string : str, optional
            The name of the current action client state.
        """

        # Create member attributes
        if not action_client:
            self.action_client = action_client
            self.__state = state
            self.__state_string = state_string
            self.state_value_dict = {}
        else:

            # Set actionclient
            self.action_client = action_client

            # Set state and state string
            if hasattr(action_client.gh, "comm_state_machine"):

                # Set attributes
                self.__state = (
                    state
                    if (state != -1)
                    else action_client.gh.comm_state_machine.latest_goal_status.status
                )
                state_machine = action_client.gh.comm_state_machine
                self.state_value_dict = {
                    value: attr
                    for attr, value in (
                        state_machine.latest_goal_status.__class__.__dict__.items()
                    )
                    if attr[0] != "_" and all(map(str.isupper, attr.replace("_", "")))
                }
                self.state_dict = {
                    value: attr for attr, value in self.state_value_dict.items()
                }
                self.__state_string = (
                    state_string
                    if (state_string != "")
                    else (
                        self.state_value_dict[self.state]
                        .lower()
                        .capitalize()
                        .replace("_", " ")
                        + "."
                    )
                )
            else:
                self.__state = state
                self.__state_string = state_string
                self.state_value_dict = {}

    @property
    def state(self):
        """Getter used to update and retrieve the current action client state from the
        actionclient object."""
        if not self.action_client:
            return self.__state
        else:

            # Update state
            self.__state = (
                self.action_client.gh.comm_state_machine.latest_goal_status.status
            )
            self.__state_string = (
                self.state_value_dict[self.__state]
                .lower()
                .capitalize()
                .replace("_", " ")
                + "."
            )

            # Return State
            return self.__state

    @property
    def state_string(self):
        """Getter used to update and retrieve the current action client state from the
        actionclient object."""
        if not self.action_client:
            return self.__state_string
        else:

            # Update state and state string
            self.__state = (
                self.action_client.gh.comm_state_machine.latest_goal_status.status
            )
            self.__state_string = (
                self.state_value_dict[self.__state]
                .lower()
                .capitalize()
                .replace("_", " ")
                + "."
            )

            # Return State
            return self.__state_string
