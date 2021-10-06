"""Smalls script to test if the ControlSwitcher class is working.
"""
from panda_gazebo.core.control_switcher import PandaControlSwitcher

if __name__ == "__main__":

    # Create control switcher object
    control_switcher = PandaControlSwitcher(connection_timeout=3)

    # Switch controllers
    arm_switch_resp = control_switcher.switch(
        control_group="arm", control_type="effort"
    )
    hand_switch_resp = control_switcher.switch(
        control_group="hand", control_type="effort"
    )

    print("You can put your debug point here")
