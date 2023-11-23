#!/usr/bin/env python3
"""Smalls script to test if the ControlSwitcher class is working."""
from panda_gazebo.core.control_switcher import PandaControlSwitcher

if __name__ == "__main__":
    # Create control switcher object.
    control_switcher = PandaControlSwitcher(connection_timeout=3)

    # Switch controllers.
    arm_switch_resp = control_switcher.switch(
        control_group="arm", control_type="effort"
    )
    control_switcher.wait_for_control_type(control_group="arm", control_type="effort")
    hand_switch_resp = control_switcher.switch(
        control_group="hand", control_type="position"
    )  # NOTE: Currently the hand only supports position control.

    # Switch controllers back.
    arm_switch_resp = control_switcher.switch(
        control_group="arm", control_type="trajectory"
    )
    hand_switch_resp = control_switcher.switch(
        control_group="hand", control_type="position"
    )  # NOTE: Currently the hand only supports position control.

    print("You can put your debug point here")
