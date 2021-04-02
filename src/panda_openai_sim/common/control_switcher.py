"""This class is responsible for switching the control type that is used for
controlling the Panda Robot robot ``arm`` and ``hand``. It serves as a wrapper around
the services created by the ROS
`controller_manager <https://wiki.ros.org/controller_manager>`_ class. It allows the
user to switch between the following control types:

* `joint_trajectory_control <https://wiki.ros.org/joint_trajectory_controller/>`_

* `joint_position_control <https://wiki.ros.org/position_controllers/>`_

* `joint_effort_control <https://wiki.ros.org/effort_controllers/>`_

* `joint_group_position_control <https://wiki.ros.org/ros_controllers/>`_

* `joint_group_effort_control <https://wiki.ros.org/ros_controllers/>`_
"""

# Main python imports
import sys
from itertools import compress

from panda_openai_sim.functions import dict_clean, flatten_list
from panda_openai_sim.extras import ControllerInfoDict

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from controller_manager_msgs.srv import (
    SwitchController,
    SwitchControllerRequest,
    ListControllers,
    ListControllersRequest,
    LoadController,
    LoadControllerRequest,
)
from panda_openai_sim.srv import (
    SwitchControlType,
    SwitchControlTypeRequest,
    ListControlType,
    ListControlTypeRequest,
)

# Global variables
ARM_CONTROLLERS = {
    "ee_control": "panda_arm_controller",
    "joint_trajectory_control": "panda_arm_controller",
    "joint_position_control": [
        "panda_arm_joint1_position_controller",
        "panda_arm_joint2_position_controller",
        "panda_arm_joint3_position_controller",
        "panda_arm_joint4_position_controller",
        "panda_arm_joint5_position_controller",
        "panda_arm_joint6_position_controller",
        "panda_arm_joint7_position_controller",
    ],
    "joint_effort_control": [
        "panda_arm_joint1_effort_controller",
        "panda_arm_joint2_effort_controller",
        "panda_arm_joint3_effort_controller",
        "panda_arm_joint4_effort_controller",
        "panda_arm_joint5_effort_controller",
        "panda_arm_joint6_effort_controller",
        "panda_arm_joint7_effort_controller",
    ],
    "joint_group_position_control": "panda_arm_joint_group_position_controller",
    "joint_group_effort_control": "panda_arm_joint_group_effort_controller",
}
HAND_CONTROLLERS = {
    "joint_trajectory_control": "panda_hand_controller",
    "joint_position_control": [
        "panda_hand_finger1_position_controller",
        "panda_hand_finger2_position_controller",
    ],
    "joint_effort_control": [
        "panda_hand_finger1_effort_controller",
        "panda_hand_finger2_effort_controller",
    ],
    "joint_group_position_control": "panda_hand_joint_group_position_controller",
    "joint_group_effort_control": "panda_hand_joint_group_effort_controller",
}
CONTROLLER_DICT = {"arm": ARM_CONTROLLERS, "hand": HAND_CONTROLLERS}


#################################################
# Panda Controller Switcher #####################
#################################################
class PandaControlSwitcher(object):
    """Used for switching the Panda robot controllers.

    Attributes
    ----------
    verbose : bool
        Boolean specifying whether we want to display log messages during switching.

    Methods
    -------
    switch(self, control_group, control_type, load_controllers=True):
        Switch Panda robot control type.
    """

    def __init__(self, connection_timeout=10, verbose=True):
        """Initializes the PandaControlSwitcher object.

        Parameters
        ----------
        connection_timeout : str, optional
            The timeout for connecting to the controller_manager services,
            by default 3 sec.
        verbose : bool, optional
            Whether to display debug log messages, defaults to True.
        """

        # Create class attributes
        self.verbose = verbose
        self._controller_switch_timeout = 3

        # Connect to controller_manager services
        try:

            # Connect to switch service
            rospy.logdebug(
                "Connecting to '/controller_manager/switch_controller' service."
            )
            rospy.wait_for_service(
                "/controller_manager/switch_controller", timeout=connection_timeout
            )
            self._switch_controller_client = rospy.ServiceProxy(
                "/controller_manager/switch_controller", SwitchController
            )
            rospy.logdebug(
                "Connected to '/controller_manager/switch_controller' service!"
            )

            # Connect to list service
            rospy.logdebug(
                "Connecting to '/controller_manager/list_controllers' service."
            )
            rospy.wait_for_service(
                "/controller_manager/list_controllers", timeout=connection_timeout
            )
            self._list_controller_client = rospy.ServiceProxy(
                "/controller_manager/list_controllers", ListControllers
            )
            rospy.logdebug(
                "Connected to '/controller_manager/list_controllers' service!"
            )

            # Connect to load controller service
            rospy.logdebug(
                "Connecting to '/controller_manager/load_controller' service."
            )
            rospy.wait_for_service(
                "/controller_manager/load_controller", timeout=connection_timeout
            )
            self._load_controller_client = rospy.ServiceProxy(
                "/controller_manager/load_controller", LoadController
            )
            rospy.logdebug(
                "Connected to '/controller_manager/load_controller' service!"
            )

            # Connect to panda_control_server/switch_control_type service
            rospy.logdebug(
                "Connecting to '/panda_control_server/switch_control_type' service."
            )
            rospy.wait_for_service(
                "/panda_control_server/switch_control_type", timeout=connection_timeout
            )
            self._panda_control_server_switch_control_type_client = rospy.ServiceProxy(
                "/panda_control_server/switch_control_type", SwitchControlType
            )
            rospy.logdebug(
                "Connected to '/panda_control_server/switch_control_type' service!"
            )

            # Connect to panda_control_server/list_control_type service
            rospy.logdebug(
                "Connecting to '/panda_control_server/list_control_type' service."
            )
            rospy.wait_for_service(
                "/panda_control_server/list_control_type", timeout=connection_timeout
            )
            self._panda_control_server_list_control_type_client = rospy.ServiceProxy(
                "/panda_control_server/list_control_type", ListControlType
            )
            rospy.logdebug(
                "Connected to '/panda_control_server/switch_control_type' service!"
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException) as e:
            rospy.logerr(
                "Shutting down '%s' because no connection could be established "
                "with the '%s' service and this service is needed "
                "when using 'joint_position_control'."
                % (
                    rospy.get_name(),
                    "/" + e.args[0].split(" /")[1],
                )
            )
            sys.exit(0)

    def _list_controllers_state(self):
        """Get information about the currently running and loaded controllers.

        Returns
        -------
        dict
            Dictionary containing information about which controllers are currently
            running or initialized divided by control group (arm/hand and other).
        """

        # Get the controller that is currently running
        list_controllers_resp = self._list_controller_client.call(
            ListControllersRequest()
        )

        # Check which Panda controllers are running
        controllers_state = ControllerInfoDict()
        for controller in list_controllers_resp.controller:
            categorized = False

            # Add Panda controllers to controllers_state
            for control_group, control_group_items in CONTROLLER_DICT.items():
                for control_type, controller_names in control_group_items.items():
                    if controller.name in controller_names:
                        categorized = True
                        if controller.state == "running":
                            controllers_state[control_group]["running"][
                                control_type
                            ].append(controller.name)
                        elif controller.state == "initialized":
                            controllers_state[control_group]["loaded"][
                                control_type
                            ].append(controller.name)
                        elif controller.state == "stopped":
                            controllers_state[control_group]["stopped"][
                                control_type
                            ].append(controller.name)

            # Add non Panda controllers to controllers_state
            if not categorized:
                if controller.state == "running":
                    controllers_state["other"]["running"][control_type].append(
                        controller.name
                    )
                elif controller.state == "initialized":
                    controllers_state["other"]["loaded"][control_type].append(
                        controller.name
                    )
                elif controller.state == "stopped":
                    controllers_state["other"]["stopped"][control_type].append(
                        controller.name
                    )

        # Remove empty fields from dictionary
        controllers_state = dict_clean(controllers_state)

        # Return list with running and initialized controllers
        return controllers_state

    def _load(self, controllers):
        """Load controllers. For this to work the parameters of the controllers
        have to be loaded onto the ROS parameter server.

        Parameters
        ----------
        controllers : str or list
            The name of the controller you want to load.

        Returns
        -------
        bool
            Success boolean or success boolean list.
        """

        # Create load_controller request
        if type(controllers) is str:

            # Send load controller request
            resp = self._load_controller_client(LoadControllerRequest(name=controllers))
            return [resp.ok]
        elif type(controllers) is list:
            # Loop through controllers and request to load them
            resp = []
            for controller in controllers:

                # Send load controller request
                resp.append(
                    self._load_controller_client(
                        LoadControllerRequest(name=controller)
                    ).ok
                )

            # Return result
            return resp
        else:
            rospy.logwarn(
                "Controllers could not be loaded as the data type of the 'controllers' "
                " variable was %s while the 'PandaControlSwitcher' only takes an "
                "argument of type 'str' or 'list'." % type(controllers)
            )
            return [False]

    def switch(
        self,
        control_group,
        control_type,
        load_controllers=True,
        timeout=None,
        verbose=None,
    ):
        """Switch Panda robot control type. This function stops all currently running
        controllers and starts the required controllers for a given control type.

        Parameters
        ----------
        control_group : str
            The control group of which you want the switch the control type. Options
            are 'hand' or 'arm'.
        control_type : str
            The robot control type you want to switch to for the given 'control_group'.
            Options are: ``joint_trajectory_control``, ``joint_position_control``,
            ``joint_effort_control``, ``joint_group_position_control`` and
            ``joint_group_effort_control``.
        load_controllers : bool
            Try to load the required controllers for a given control_type if they are
            not yet loaded.
        timeout : int, optional
            The timout for switching to a given controller, by default
            :py:attr:`self._controller_switch_timeout`.
        verbose : bool, optional
            Whether to display debug log messages, defaults to verbose value set during
            the class initiation.

        Returns
        -------
        ControllerSwitcherResponse
            Contains information about whether the switch operation was successfull
            'success' and the previously used controller 'prev_control_type'.
        """

        # Check verbosity
        if not verbose:
            verbose = self.verbose

        # Create response message
        resp = ControllerSwitcherResponse()

        # Validate input arguments
        control_type = control_type.lower()
        control_group = control_group.lower()
        if type(control_group) == list:
            if len(control_group) > 1:

                # Log result and return
                rospy.logwarn(
                    "Please specify a single control group you want to switch the "
                    "control type for."
                )
                resp.success = False
                return resp
            else:
                control_group = control_group[0]
        if control_group not in CONTROLLER_DICT.keys():

            # Log result and return
            rospy.logwarn(
                "The '%s' control group you specified is not valid. Valid control "
                "groups for the Panda robot are %s"
                % (control_group, CONTROLLER_DICT.keys())
            )
            resp.success = False
            return resp
        if control_type not in CONTROLLER_DICT[control_group].keys():

            # Log result and return
            rospy.logwarn(
                "The '%s' control type you specified is not valid. Valid control types "
                "for the Panda robot are %s"
                % (control_type, CONTROLLER_DICT[control_group].keys())
            )
            resp.success = False
            return resp
        if not timeout:  # If None use default
            timeout = self._controller_switch_timeout

        # Get active controllers
        controllers_state = self._list_controllers_state()

        # Save current controller type
        prev_control_type = str(
            list(controllers_state[control_group]["running"].keys())[0]
        )

        # Generate switch controller msg
        controller_already_running = False
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.strictness = SwitchControllerRequest.STRICT
        switch_controller_msg.timeout = timeout
        if (
            "running" in controllers_state[control_group].keys()
            and control_type in controllers_state[control_group]["running"].keys()
        ):  # If control type controllers are already running
            controller_already_running = True
        elif (
            "stopped" in controllers_state[control_group].keys()
            and control_type in controllers_state[control_group]["stopped"].keys()
        ):  # If controller was stopped

            # Fill the start_controllers field of the switch control message
            switch_controller_msg.start_controllers = (
                CONTROLLER_DICT[control_group][control_type]
                if type(CONTROLLER_DICT[control_group][control_type]) is list
                else [CONTROLLER_DICT[control_group][control_type]]
            )

            # Fill the stop_controllers field of the switch control message
            switch_controller_msg.stop_controllers = flatten_list(
                controllers_state[control_group]["running"].values()
            )
        elif (
            "loaded" not in controllers_state[control_group].keys()
            or control_type not in controllers_state[control_group]["loaded"].keys()
        ):  # Try to load the controllers if not yet loaded

            # Load the required controllers
            if load_controllers:
                retval = self._load(CONTROLLER_DICT[control_group][control_type])
                failed_controllers = list(
                    compress(
                        CONTROLLER_DICT[control_group][control_type],
                        [not val for val in retval],
                    )
                )
            else:

                # Log result and return
                rospy.logwarn(
                    "The Panda %s control type was not switched to '%s' because the %s "
                    "controllers could not be loaded as 'load_controllers' was set to "
                    "argument was set to False."
                    % (
                        control_group,
                        control_type,
                        CONTROLLER_DICT[control_group][control_type],
                    )
                )
                resp.success = False
                resp.prev_control_type = prev_control_type
                return resp

            # Check if all controllers were loaded successfully
            if len(failed_controllers) == 0:

                # Fill the start_controllers field of the switch control message
                switch_controller_msg.start_controllers = (
                    CONTROLLER_DICT[control_group][control_type]
                    if type(CONTROLLER_DICT[control_group][control_type]) is list
                    else [CONTROLLER_DICT[control_group][control_type]]
                )

                # Fill the stop_controllers field of the switch control message
                switch_controller_msg.stop_controllers = flatten_list(
                    controllers_state[control_group]["running"].values()
                )
            else:

                # Log result and return
                rospy.logwarn(
                    "The Panda %s control type was not switched to '%s' as the %s "
                    "controllers could not be loaded."
                    % (control_group, control_type, failed_controllers)
                )
                resp.success = False
                resp.prev_control_type = prev_control_type
                return resp
        else:

            # Fill the start_controllers field of the switch control message
            switch_controller_msg.start_controllers = (
                CONTROLLER_DICT[control_group][control_type]
                if type(CONTROLLER_DICT[control_group][control_type]) is list
                else [CONTROLLER_DICT[control_group][control_type]]
            )

            # Fill the stop_controllers field of the switch control message
            switch_controller_msg.stop_controllers = flatten_list(
                controllers_state[control_group]["running"].values()
            )

        # Send switch_controller msgs
        if not controller_already_running:
            rospy.logdebug(
                "Switching Panda %s control type to '%s'."
                % (control_group, control_type)
            )
            retval = self._switch_controller_client(switch_controller_msg)

            # Change panda_controller_server control type if needed and switch was
            # successfull
            if retval.ok:

                # Request 'panda_controller_server' to use group control if needed
                if control_type in [
                    "joint_group_position_control",
                    "joint_group_effort_control",
                ]:
                    if (
                        self._panda_control_server_list_control_type_client(
                            ListControlTypeRequest()
                        ).control_type
                        != "joint_group_control"
                    ):
                        self._panda_control_server_switch_control_type_client(
                            SwitchControlTypeRequest()
                        )
                elif control_type in ["joint_position_control", "joint_effort_control"]:
                    if (
                        self._panda_control_server_list_control_type_client(
                            ListControlTypeRequest()
                        ).control_type
                        != "joint_control"
                    ):
                        self._panda_control_server_switch_control_type_client(
                            SwitchControlTypeRequest()
                        )

                # Log result and return
                rospy.logdebug(
                    "Switching Panda %s control type to '%s' successfull."
                    % (control_group, control_type)
                )
                resp.success = retval.ok
                resp.prev_control_type = prev_control_type
                return resp
            else:

                # Log result and return
                rospy.logwarn(
                    "Switching Panda %s control type to '%s' failed."
                    % (control_group, control_type)
                )
                resp.success = retval.ok
                resp.prev_control_type = prev_control_type
                return resp

        else:

            # Log result and return
            if verbose:
                rospy.logdebug(
                    "Panda %s control type not switched to '%s' as the Panda robot was "
                    "already using '%s'."
                    % (control_group, control_type, prev_control_type)
                )
            resp.success = True
            resp.prev_control_type = prev_control_type
            return resp


#################################################
# Panda Controller Switcher Response ############
#################################################
class ControllerSwitcherResponse:
    """Class used for returning the result of the ControllerSwitcher.switch method.

    Attributes
    ----------
    success : bool
        Specifies whether the switch operation was successfull.
    prev_control_type : str
        The previous used control type.
    """

    def __init__(self, success=True, prev_control_type=""):
        """Initiate ControllerSwitcher response object.

        Parameters
        ----------
        success : bool, optional
            Whether the switch operation was successfull, by default True.
        prev_control_type : str, optional
            The previous used control type, by default ``""``.
        """

        # Set class attributes
        self.success = success
        self.prev_control_type = prev_control_type
