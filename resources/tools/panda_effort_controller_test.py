"""Script that creates a dynamic parameter server which can be used to test out the
Panda effort_controllers"""

# Import ROS python packages
import rospy
from dynamic_reconfigure.server import Server
from panda_openai_sim.cfg import JointEffortConfig
from panda_openai_sim.srv import SetJointEfforts, SetJointEffortsRequest


#################################################
# JointEffortTester #############################
#################################################
class JointEffortTester(object):
    def __init__(self):
        """Class constructor"""

        # Connect to /panda_control_server/set_joint_efforts
        rospy.logdebug(
            "Connecting to '/panda_control_server/panda_arm/set_joint_efforts' service."
        )
        rospy.wait_for_service(
            "/panda_control_server/panda_arm/set_joint_efforts", timeout=10
        )
        self.set_joint_efforts_client = rospy.ServiceProxy(
            "/panda_control_server/panda_arm/set_joint_efforts", SetJointEfforts
        )
        rospy.logdebug(
            "Connected to 'panda_control_server/panda_arm/set_joint_efforts' service!"
        )

        # Generate joint_efforts msg
        self.dynamic_reconfig_srv = Server(
            JointEffortConfig, self._dynamic_reconfig_callback
        )

        # Create member variables
        self.joint_efforts_setpoint = [
            self.config["joint1_effort"],
            self.config["joint2_effort"],
            self.config["joint3_effort"],
            self.config["joint4_effort"],
            self.config["joint5_effort"],
            self.config["joint6_effort"],
            self.config["joint7_effort"],
        ]

        # Call set_joint_efforts service
        set_joint_efforts_msg = SetJointEffortsRequest()
        set_joint_efforts_msg.joint_efforts = self.joint_efforts_setpoint
        self.set_joint_efforts_client.call(set_joint_efforts_msg)

        # # Create service caller
        # rate = rospy.Rate(25)
        # rospy.Timer(rate.sleep_dur, self._joint_effort_service_tester_callback)

    def _dynamic_reconfig_callback(self, config, level):
        """The callback function for the dynamic reconfigure server

        Parameters
        ----------
        config :
            Dynamic reconfigure config dictionary.
        level : int
            Dynamic reconfigure level parameter indicating which parameter has been
            changed.
        """

        # Print request
        rospy.loginfo(
            "Joint efforts reconfigure Request: [{joint1_effort}, {joint2_effort}, "
            "{joint3_effort}, {joint4_effort}, {joint5_effort}, {joint6_effort}, "
            "{joint7_effort}]".format(**config)
        )

        # Put changed values on object
        self.config = config
        self.joint_efforts_setpoint = [
            self.config["joint1_effort"],
            self.config["joint2_effort"],
            self.config["joint3_effort"],
            self.config["joint4_effort"],
            self.config["joint5_effort"],
            self.config["joint6_effort"],
            self.config["joint7_effort"],
        ]

        # Call set_joint_efforts service
        set_joint_efforts_msg = SetJointEffortsRequest()
        set_joint_efforts_msg.joint_efforts = self.joint_efforts_setpoint
        self.set_joint_efforts_client.call(set_joint_efforts_msg)

        # Return config
        return config

    def _joint_effort_service_tester_callback(self, event):
        """This callback periodically calls the 'panda_control_server/set_joint_efforts"
        "'. The rate at which this is done is specified in the class initiation.

        Parameters
        ----------
        event : rospy.timer.TimerEvent
            Event object which contains information about the timer and timer callback.
        """

        # Call set_joint_efforts service
        set_joint_efforts_msg = SetJointEffortsRequest()
        set_joint_efforts_msg.joint_efforts = self.joint_efforts_setpoint
        self.set_joint_efforts_client.call(set_joint_efforts_msg)


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate ROS node
    rospy.init_node("panda_effort_controller_tester", anonymous=False)

    # Create joint effort tester class
    joint_tester = JointEffortTester()

    # Spin till killed
    rospy.spin()
