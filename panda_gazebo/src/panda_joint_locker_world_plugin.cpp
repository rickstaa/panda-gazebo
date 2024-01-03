/*
 * @file panda_joint_locker_world_plugin.cpp
 * @brief A Gazebo WorldPlugin for controlling joint locking in a Panda robot.
 */
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "panda_gazebo/LockJoints.h"

namespace gazebo
{
const std::string MODEL_NAME = "panda";
const std::string SERVICE_NAME = "/lock_unlock_panda_joints";
const unsigned int JOINT_AXIS_INDEX = 0;

/**
 * @brief A Gazebo WorldPlugin class for controlling joint locking in a Panda robot.
 */
class PandaJointLockerPlugin : public WorldPlugin
{
public:
  /**
   * @brief Construct a new Panda Joint Locker Plugin object.
   */
  PandaJointLockerPlugin() : WorldPlugin()
  {
  }

  /**
   * @brief Load the plugin, initialize the ROS node and service, and store the initial max_force values for all joints.
   *
   * @param _world Pointer to the Gazebo world.
   * @param _sdf Pointer to the SDF element.
   */
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    ROS_INFO("Loading PandaJointLockerPlugin");

    // Make sure the ROS node for Gazebo has already been initialized.
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Create ROS node.
    this->rosNode.reset(new ros::NodeHandle("panda_joint_control_plugin"));

    // Create ROS service to lock and unlock specified joints.
    this->rosService = this->rosNode->advertiseService(SERVICE_NAME, &PandaJointLockerPlugin::LockUnlockJoints, this);

    // Store the pointer to the world.
    this->world = _world;

    ROS_INFO("Loaded PandaJointLockerPlugin");
  }

private:
  physics::WorldPtr world;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::ServiceServer rosService;
  std::map<std::string, double> oldLowerLimit;
  std::map<std::string, double> oldUpperLimit;

  /**
   * @brief Lock or unlock the specified joints.
   *
   * @param req The service request, which specifies the joints to lock or unlock.
   * @param res The service response, which indicates whether the operation was successful.
   * @return true if the service was handled successfully, false otherwise.
   */
  bool LockUnlockJoints(panda_gazebo::LockJoints::Request& req, panda_gazebo::LockJoints::Response& res)
  {
    auto model = this->world->ModelByName(MODEL_NAME);
    if (!model)
    {
      res.success = false;
      res.message = "Model not found: " + MODEL_NAME + ". Unable to lock/unlock joints.";
      return true;
    }

    for (const std::string& joint_name : req.joint_names)
    {
      auto joint = model->GetJoint(joint_name);
      if (!joint)
      {
        res.success = false;
        res.message = "Joint not found: " + joint_name;
        return true;
      }

      if (req.lock)
      {
        double current_position = joint->Position(JOINT_AXIS_INDEX);

        // Store old limits and lock joint at current position.
        this->oldLowerLimit[joint_name] = joint->LowerLimit(JOINT_AXIS_INDEX);
        this->oldUpperLimit[joint_name] = joint->UpperLimit(JOINT_AXIS_INDEX);
        joint->SetLowerLimit(JOINT_AXIS_INDEX, current_position);
        joint->SetUpperLimit(JOINT_AXIS_INDEX, current_position);
      }
      else
      {
        // Unlock the joint by restoring the old joint limits if they exist.
        if (this->oldLowerLimit.find(joint_name) != this->oldLowerLimit.end() &&
            this->oldUpperLimit.find(joint_name) != this->oldUpperLimit.end())
        {
          joint->SetLowerLimit(JOINT_AXIS_INDEX, this->oldLowerLimit[joint_name]);
          joint->SetUpperLimit(JOINT_AXIS_INDEX, this->oldUpperLimit[joint_name]);
        }
      }
    }

    res.success = true;
    res.message = std::string("Joint ") + (req.lock ? "locking" : "unlocking") + " successful";
    return true;
  }
};

GZ_REGISTER_WORLD_PLUGIN(PandaJointLockerPlugin)
}  // namespace gazebo
