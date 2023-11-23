"""This class is responsible for switching the control type that is used for
controlling the Panda Robot robot arm and hand. It serves as a wrapper around the
services created by the ROS
`controller_manager <https://wiki.ros.orgcontroller_manager>`_ class.

Control types:
    * `trajectory <https://wiki.ros.org/joint_trajectory_controller/>`_
    * `position <https://wiki.ros.org/position_controllers/>`_
    * `effort <https://wiki.ros.org/effort_controllers/>`_
"""
import time
from itertools import compress

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
)
from rospy.exceptions import ROSException, ROSInterruptException

from panda_gazebo.common import ControllerInfoDict
from panda_gazebo.common.helpers import (
    dict_clean,
    flatten_list,
    get_unique_list,
    ros_exit_gracefully,
)

# Global script vars.
ARM_CONTROLLERS = {
    "end_effector": "panda_arm_trajectory_controller",
    "trajectory": "panda_arm_trajectory_controller",
    "position": "panda_arm_joint_position_controller",
    "effort": "panda_arm_joint_effort_controller",
}
HAND_CONTROLLERS = {
    "position": "franka_gripper",
}
CONTROLLER_DICT = {"arm": ARM_CONTROLLERS, "hand": HAND_CONTROLLERS}


class ControllerSwitcherResponse:
    """Class used for returning the result of the :meth:`PandaControlSwitcher.switch`
    method.

    Attributes:
        success (bool): Specifies whether the switch operation was successful.
        prev_control_type (str): The previous used control type.
    """

    def __init__(self, success=True, prev_control_type=""):
        """Initialise ControllerSwitcherResponse object.

        Args:
            success (bool, optional): Whether the switch operation was successful.
                Defaults to ``True``.
            prev_control_type (str, optional): The previous used control type. Defaults
                to ``""``.
        """
        self.success = success
        self.prev_control_type = prev_control_type


class PandaControlSwitcher(object):
    """Used for switching the Panda robot controllers.

    Attributes:
        verbose (bool): Boolean specifying whether we want to display log messages
            during switching.
        arm_control_types (list): List of currently active arm control types.
        hand_control_types (list): List of currently active hand control types.
    """

    def __init__(self, connection_timeout=10, verbose=True, robot_name_space=""):
        """Initialise PandaControlSwitcher object.

        Args:
            connection_timeout (str, optional): The timeout for connecting to the
                controller_manager services. Defaults to `10` sec.
            verbose (bool, optional): Whether to display debug log messages. Defaults to
                ``True``.
            robot_name_space (string, optional): The namespace the robot, and thus the
                'controller_manager' is on. Defaults to ``""``.
        """
        self.verbose = verbose
        self._controller_spawner_wait_timeout = 5

        # Connect to controller_manager services.
        try:
            switch_controller_srv_topic = "%s/controller_manager/switch_controller" % (
                robot_name_space
            )
            rospy.logdebug(
                "Connecting to '%s' service." % (switch_controller_srv_topic)
            )
            rospy.wait_for_service(
                switch_controller_srv_topic, timeout=connection_timeout
            )
            self._switch_controller_client = rospy.ServiceProxy(
                switch_controller_srv_topic, SwitchController
            )
            rospy.logdebug("Connected to '%s' service!" % switch_controller_srv_topic)
            list_controllers_srv_topic = "%s/controller_manager/list_controllers" % (
                robot_name_space
            )
            rospy.logdebug("Connecting to '%s' service." % list_controllers_srv_topic)
            rospy.wait_for_service(
                list_controllers_srv_topic, timeout=connection_timeout
            )
            self._list_controller_client = rospy.ServiceProxy(
                list_controllers_srv_topic, ListControllers
            )
            rospy.logdebug("Connected to '%s' service!" % list_controllers_srv_topic)
            load_controller_srv_topic = "%s/controller_manager/load_controller" % (
                robot_name_space
            )
            rospy.logdebug("Connecting to '%s' service." % load_controller_srv_topic)
            rospy.wait_for_service(
                load_controller_srv_topic, timeout=connection_timeout
            )
            self._load_controller_client = rospy.ServiceProxy(
                load_controller_srv_topic, LoadController
            )
            rospy.logdebug("Connected to '%s' service!" % load_controller_srv_topic)
        except (rospy.ServiceException, ROSException, ROSInterruptException) as e:
            err_msg = (
                "Shutting down '%s' because no connection could be established "
                "with the '%s' service and this service is needed "
                "when using 'position'."
                % (
                    rospy.get_name(),
                    "/" + e.args[0].split(" /")[1],
                )
            )
            ros_exit_gracefully(shutdown_message=err_msg, exit_code=1)

    def _list_controllers_state(self):  # noqa: C901
        """Get information about the currently running and loaded controllers.

        Returns:
            dict: Dictionary containing information about which controllers are
                currently running or loaded divided by control group arm/hand and
                other).
        """
        list_controllers_resp = self._list_controller_client.call(
            ListControllersRequest()
        )

        # Check which Panda controllers are running.
        controllers_state = ControllerInfoDict()
        for controller in list_controllers_resp.controller:
            categorized = False

            # Add Panda controllers to controllers_state.
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

            # Add non Panda controllers to controllers_state.
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

        return dict_clean(controllers_state)

    def _load(self, controllers):
        """Load controllers. For this to work the parameters of the controllers
        have to be loaded onto the ROS parameter server.

        Args:
            controllers (union[str, list]): The name of the controller you want to load.

        Returns:
            bool: Success boolean or success boolean list.
        """
        # Create load_controller request.
        if isinstance(controllers, str):
            # Send load controller request.
            resp = self._load_controller_client(LoadControllerRequest(name=controllers))
            return [resp.ok]
        elif isinstance(controllers, list):
            # Loop through controllers and request to load them.
            resp = []
            for controller in controllers:
                # Send load controller request.
                resp.append(
                    self._load_controller_client(
                        LoadControllerRequest(name=controller)
                    ).ok
                )
            return resp
        else:
            rospy.logwarn(
                "Controllers could not be loaded as the data type of the 'controllers' "
                f"variable was {type(controllers)} while the 'PandaControlSwitcher' "
                "only takes an argument of type 'str' or 'list'."
            )
            return [False]

    @property
    def arm_control_types(self):
        """Returns the currently active arm control types. Returns empty list when no
        control type is enabled.
        """
        controllers_state = self._list_controllers_state()
        arm_state = controllers_state.get("arm", {})
        return list(arm_state.get("running", {}).keys())

    @property
    def hand_control_types(self):
        """Returns the currently active hand control types. Returns empty when no
        control type is enabled.
        """
        controllers_state = self._list_controllers_state()
        hand_state = controllers_state.get("hand", {})
        return list(hand_state.get("running", {}).keys())

    def wait_for_control_type(self, control_group, control_type, timeout=None, rate=10):
        """Function that can be used to wait till all the controllers used for a given
        'control_group' and 'control_type' are running. Useful four when you expect a
        launch file to load certain controllers.

        Args:
            control_group (str): The control group of which you want the switch the
                control type. Options are ``hand`` or ``arm``.
            control_type (str): The robot control type you want to switch to for the
                given 'control_group'. Options are: ``trajectory``, ``position`` and
                ``effort``.
            timeout (float, optional): The function timeout. Defaults to ``None`` meaning
                the function will wait for ever.
            rate (int, optional): The 'control_type' check rate. Defaults to `10`
                hz.

        Raises:
            TimeoutError: Thrown when the set timeout has passed.
        """
        start_time = time.time()
        timeout = timeout if timeout else float("inf")
        while (
            (control_group == "arm" and control_type not in self.arm_control_types)
            or (control_group == "hand" and control_type not in self.hand_control_types)
        ) and time.time() - start_time <= timeout:
            time.sleep(1.0 / rate)
        if time.time() - start_time > timeout:
            raise TimeoutError(
                f"Control type '{control_type}' for control_group '{control_group}' "
                f"was not spawned within the set timeout (i.e. {timeout})."
            )

    def switch(  # noqa: C901
        self,
        control_group,
        control_type,
        load_controllers=True,
        timeout=3,
        verbose=None,
    ):
        """Switch Panda robot control type. This function stops all currently running
        controllers and starts the required controllers for a given control type.

        Args:
            control_group (str): The control group of which you want the switch the
                control type. Options are ``hand`` or ``arm``.
            control_type (str): The robot control type you want to switch to for the
                given 'control_group'. Options are: ``trajectory``, ``position``
                and ``effort``.
            load_controllers (bool): Try to load the required controllers for a given
                control_type if they are not yet loaded.
            timeout (int, optional): The timeout for switching to a given controller.
                Defaults to ``3`` sec.
            verbose (bool, optional): Whether to display debug log messages. Defaults
                to verbose value set during the class initiation.

        Returns:
            :obj:`~panda_gazebo.core.control_switcher.ControllerSwitcherResponse`:
                Contains information about whether the switch operation was successful
                'success' and the previously used controller 'prev_control_type'.
        """
        resp = ControllerSwitcherResponse()
        verbose = self.verbose if verbose is None else verbose

        # Validate input arguments.
        control_type = control_type.lower()
        control_group = control_group.lower()
        if isinstance(control_group, list):
            if len(control_group) > 1:
                rospy.logwarn(
                    "Please specify a single control group you want to switch the "
                    "control type for."
                )
                resp.success = False
                return resp
            else:
                control_group = control_group[0]
        if control_group not in CONTROLLER_DICT:
            rospy.logwarn(
                f"The '{control_group}' control group you specified is not valid. "
                "Valid control groups for the Panda robot are "
                f"{list(CONTROLLER_DICT.keys())}."
            )
            resp.success = False
            return resp
        if control_type not in CONTROLLER_DICT[control_group]:
            rospy.logwarn(
                f"The '{control_type} control type you specified is not valid. Valid "
                "control types for the Panda robot are "
                f"{list(CONTROLLER_DICT[control_group].keys())}."
            )
            resp.success = False
            return resp

        # Retrieve information about the currently running and loaded controllers.
        # NOTE: Here we wait a bit till we are sure the controller_spawner is ready.
        start_time = time.time()
        controllers_state = {}
        while (
            control_group not in controllers_state
            or "running" not in controllers_state[control_group]
        ) and time.time() - start_time <= self._controller_spawner_wait_timeout:
            controllers_state = self._list_controllers_state()
        running_controllers = controllers_state.get(control_group, {}).get("running")
        prev_control_type = ", ".join(running_controllers.keys())
        running_state = controllers_state.get(control_group, {}).get("running", {})
        running_control_types = list(running_state.keys())
        running_controllers = get_unique_list(flatten_list(running_state.values()))
        stopped_state = controllers_state.get(control_group, {}).get("stopped", {})
        stopped_control_types = list(stopped_state.keys())
        loaded_state = controllers_state.get(control_group, {}).get("loaded", {})
        loaded_control_types = list(loaded_state.keys())
        controllers = CONTROLLER_DICT[control_group][control_type]

        # Generate switch controller msg and load controllers if needed.
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.strictness = SwitchControllerRequest.STRICT
        switch_controller_msg.timeout = timeout
        switch_controller_msg.start_controllers = (
            controllers if isinstance(controllers, list) else [controllers]
        )
        if (
            control_type in running_control_types
        ):  # If control type controllers are already running.
            if verbose:
                rospy.logdebug(
                    f"Panda {control_group} control type not switched to "
                    f"'{control_type}' as the Panda robot was already using "
                    f"'{prev_control_type}'."
                )
            resp.success = True
            resp.prev_control_type = prev_control_type
            return resp
        elif control_type in stopped_control_types:  # If controller was stopped.
            switch_controller_msg.stop_controllers = flatten_list(
                controllers_state[control_group]["running"].values()
            )
        elif (
            control_type not in loaded_control_types
        ):  # Try to load the controllers if not yet loaded.
            # Load the required controllers.
            if load_controllers:
                retval = self._load(controllers)
                failed_controllers = list(
                    compress(
                        controllers,
                        [not val for val in retval],
                    )
                )
            else:
                rospy.logwarn(
                    f"The Panda {control_group} control group was not switched to "
                    f"'{control_type}' because the "
                    f"{controllers} controllers could "
                    "not be loaded as 'load_controllers' was set to argument was set "
                    "to 'False'."
                )
                resp.success = False
                resp.prev_control_type = prev_control_type
                return resp

            # Check if all controllers were loaded successfully.
            if len(failed_controllers) == 0:
                switch_controller_msg.stop_controllers = running_controllers
            else:
                if (
                    control_group == "arm"
                    and control_type not in self.arm_control_types
                ) or (
                    control_group == "hand"
                    and control_type not in self.hand_control_types
                ):
                    rospy.logwarn(
                        f"The Panda {control_group} control griyo was not switched to "
                        f"'{control_type}' as the {failed_controllers} controllers "
                        "could not be loaded."
                    )
                    resp.success = False
                else:
                    resp.success = False
                resp.prev_control_type = prev_control_type
                return resp
        else:
            switch_controller_msg.stop_controllers = running_controllers

        # Send switch_controller msgs.
        rospy.logdebug(
            f"Switching Panda {control_group} control group to '{control_type}'."
        )
        retval = self._switch_controller_client(switch_controller_msg)
        if retval.ok:
            rospy.logdebug(
                f"Switching Panda {control_group} control group to "
                f"'{control_type}' successful."
            )
        else:
            if (
                control_group == "arm" and control_type not in self.arm_control_types
            ) or (
                control_group == "hand" and control_type not in self.arm_control_types
            ):
                rospy.logwarn(
                    f"Switching Panda {control_group} control type to "
                    f"'{control_type}' failed."
                )
            else:
                retval.ok = True
        resp.success = retval.ok
        resp.prev_control_type = prev_control_type
        return resp
