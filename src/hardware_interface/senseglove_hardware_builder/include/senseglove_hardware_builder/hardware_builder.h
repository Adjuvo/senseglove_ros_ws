// Copyright 2020 Senseglove
#ifndef ROS_WORKSPACE_HARDWARE_BUILDER_H
#define ROS_WORKSPACE_HARDWARE_BUILDER_H
#include "senseglove_hardware_builder/allowed_robot.h"

#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <senseglove_hardware/actuation_mode.h>
#include <senseglove_hardware/joint.h>
#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware/senseglove_setup.h>


// Creates a SenseGloveRobot from a robot yaml and URDF.
class HardwareBuilder
{
public:

  // Initialises a HardwareBuilder with a robotName enumerator. Grabs the .yaml file associated with the robot name.
  explicit HardwareBuilder(AllowedRobot robot, int gloveIndex, bool isRight);
  HardwareBuilder(AllowedRobot robot, urdf::Model urdfModel);

  // Initialises a HardwareBuilder with a path to a .yaml file.
  explicit HardwareBuilder(const std::string& yamlPath, int gloveIndex, bool isRight);
  HardwareBuilder(const std::string& yamlPath, urdf::Model urdfModel);

  // Creates a SenseGloveRobot. Loads a URDF from the parameter server
  std::unique_ptr<SGHardware::SenseGloveSetup> createSenseGloveSetup();

  // Loops over all keys in the keyList and check if they exist in the config.
  static void validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& keyList, const std::string& objectName);

  static SGHardware::Joint createJoint(const YAML::Node& jointConfig, const std::string& jointName, const urdf::JointConstSharedPtr& urdfJoint);
  static SGHardware::SenseGloveRobot createRobot(const YAML::Node& jointConfig, urdf::Model urdf, std::vector<SGHardware::Joint> joints, std::shared_ptr<HapticGlove> glove, int robotIndex, bool isArgRight);

  static const std::vector<std::string> JOINT_REQUIRED_KEYS;
  static const std::vector<std::string> ROBOT_REQUIRED_KEYS;

private:
  // Initializes the URDF if necessary.

  void initUrdf(SGCore::EDeviceType deviceType, bool isRight);

  // Returns all joints found in the given config.
  std::vector<SGHardware::Joint> createJoints(const YAML::Node& joints_config) const;
  std::vector<SGHardware::SenseGloveRobot> createRobots(const YAML::Node& allRobotConfig, urdf::Model urdfModel, std::vector<SGHardware::Joint> joints, std::vector<std::shared_ptr<HapticGlove>> allGloves) const;
  std::shared_ptr<HapticGlove> correctGlove(std::vector<std::shared_ptr<HapticGlove>> gloves) const;

  YAML::Node robotConfig;
  urdf::Model urdfModel;
  bool urdfInitialize = true;
  int gloveIndex;
  bool isRight;
};
#endif  // ROS_WORKSPACE_HARDWARE_BUILDER_H
