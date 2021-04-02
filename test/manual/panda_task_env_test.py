"""Script used for testing features of the panda_openai_sim task environment during
develeopment.
"""

# Main python impots
import gym

# ROS python imports
import rospy

# Import panda gym environment
from panda_openai_sim.envs.task_envs import PandaTaskEnv

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("panda_openai_sim_her", log_level=rospy.DEBUG)

    # Create environment6
    env = gym.make(
        "PandaTask-v0",
        robot_arm_control_type="ee_control",
        robot_hand_control_type="joint_trajectory_control",
        block_gripper=True,
        target_bounds={
            "x_min": -1,
            "x_max": 1,
            "y_min": -1,
            "y_max": 1,
            "z_min": 0,
            "z_max": 1.5,
        },
        init_pose_bounds={
            "x_min": 0,
            "x_max": 0.4,
            "y_min": 0,
            "y_max": 0.4,
            "z_min": 0,
            "z_max": 1,
            "gripper_width_min": 0,
            "gripper_width_max": 0.4,
        },
        has_object=True,
        action_space_joints=["x", "y", "gripper_width"],
    )

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)

    # -- TEST action function --
    # -- Inference --
    # Visualize results
    obs = env.reset()
    obs, reward, done, _ = env.step(env.action_space.sample())
    obs, reward, done, _ = env.step(env.action_space.sample())
    obs, reward, done, _ = env.step(env.action_space.sample())
