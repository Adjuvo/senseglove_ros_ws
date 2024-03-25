// Copyright 2020 Senseglove
#include "senseglove_hardware_builder/hardware_builder.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include "HandLayer.hpp"
#include "HapticGlove.hpp"

const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation", "jointIndex", "minPosition", "maxPosition" };
const std::vector<std::string> HardwareBuilder::ROBOT_REQUIRED_KEYS = { "deviceType" };

HardwareBuilder::HardwareBuilder(AllowedRobot robot, int gloveIndex, bool isRight)
  : HardwareBuilder(robot.getFilePath(), gloveIndex, isRight)
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot, urdf::Model urdfModel)
  : robotConfig(YAML::LoadFile(robot.getFilePath())), urdfModel(std::move(urdfModel)), urdfInitialize(false)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yamlPath, int gloveIndex, bool isRight)
  : robotConfig(YAML::LoadFile(yamlPath)), gloveIndex(gloveIndex), isRight(isRight)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yamlPath, urdf::Model urdfModel)
  : robotConfig(YAML::LoadFile(yamlPath)), urdfModel(std::move(urdfModel)), urdfInitialize(false)
{
}

// Initializes connection to SenseGloves, selects appropriate glove based on configuration. Initializes the Robot's Joints and URDF modeURDF model based on the configuration.
std::unique_ptr<SGHardware::SenseGloveSetup> HardwareBuilder::createSenseGloveSetup()
{
  if (!DeviceList::SenseComRunning())
  {
    ROS_ERROR_STREAM("Hardware Builder: SenseCom is not running. Ensure that the SenseGlove communication service is active.");
    throw std::runtime_error("SenseCom is not running");
  }

  const auto robotName = this->robotConfig.begin()->first.as<std::string>();
  ROS_INFO_STREAM("Hardware Builder: Starting creation of robot " << robotName);

  // Remove top level robot name key
  YAML::Node config = this->robotConfig[robotName];
  ROS_INFO_STREAM("Hardware Builder: Size of robot config " << this->robotConfig.size());

  std::vector<std::shared_ptr<HapticGlove>> allGloves = SenseGlove::GetHapticGloves(true);
  auto currentGlove = allGloves[gloveIndex];

  ROS_INFO_STREAM("Hardware_Builder: Creating senseglove robots");
  ROS_INFO_STREAM("Hardware Builder: Obtained the following gloves: ");
  for (auto& glove : allGloves)
  {
    ROS_INFO_STREAM(glove->GetDeviceId());
  }
  
  if (DeviceList::SenseComRunning())
  {
    this->initUrdf(currentGlove->GetDeviceType(), currentGlove->IsRight());
  }
  else
  {
    ROS_ERROR_STREAM("Hardware Builder: No Sensegloves connected!");
    std::exit(1);
  }

  std::vector<SGHardware::Joint> joints = this->createJoints(config["joints"]);
  ROS_INFO_STREAM("Hardware Builder: Created joints: " << joints.size());

  SGHardware::SenseGloveRobot SGRobot =  HardwareBuilder::createRobot(config, this->urdfModel, std::move(joints), currentGlove, gloveIndex, isRight);

  ROS_INFO_STREAM("Hardware Builder: Created Robot is a " << SGRobot.getRobotName() << "/ with Right: "  << SGRobot.getRight() << " and URDF-Right: " << currentGlove->IsRight());
  ROS_INFO_STREAM("Hardware Builder: Robot config:\n" << config);

  return std::make_unique<SGHardware::SenseGloveSetup>(std::move(SGRobot));
}


// Initializes and returns a senseglove::Joint object based on the provided configuration. Parses the YAML node for joint configuration, validating the presence of required keys, and setting up actuation modes.
SGHardware::Joint HardwareBuilder::createJoint(const YAML::Node& jointConfig, const std::string& jointName, const urdf::JointConstSharedPtr& urdfJoint)
{
  ROS_DEBUG_STREAM("Starting creation of joint: " << jointName);
  
  if (!urdfJoint)
  {
    throw std::runtime_error("No URDF joint found for joint: " + jointName);
  }
  HardwareBuilder::validateRequiredKeysExist(jointConfig, HardwareBuilder::JOINT_REQUIRED_KEYS, jointName);

  int jointIndex = jointConfig["jointIndex"] ? jointConfig["jointIndex"].as<int>() : -1;
  bool allowActuation = jointConfig["allowActuation"].as<bool>(false); // Default to false if not specified
  
  if (!jointConfig["jointIndex"])
  {
    ROS_WARN_STREAM("Joint: " << jointName << " does not have a netNumber");
  }

  SGHardware::ActuationMode actuationMode = SGHardware::ActuationMode::position;
  if (jointConfig["actuationMode"])
  {
    actuationMode = SGHardware::ActuationMode(jointConfig["actuationMode"].as<std::string>());
  }

  return {jointName, jointIndex, actuationMode, allowActuation};
}

// Constructs a SenseGloveRobot object by combining information about the glove, joint configurations, and the URDF model. Ensures that the glove's handedness matches the expected configuration.
SGHardware::SenseGloveRobot HardwareBuilder::createRobot(const YAML::Node& robotConfig, urdf::Model urdfModel, std::vector<SGHardware::Joint> jointList, std::shared_ptr<HapticGlove> glove, int robotIndex, bool isArgRight)
{
  ROS_DEBUG_STREAM("Starting creation of glove: " << robotIndex);
  HardwareBuilder::validateRequiredKeysExist(robotConfig, HardwareBuilder::ROBOT_REQUIRED_KEYS, "glove");

  bool isGloveRight = glove->IsRight();

  if (isGloveRight xor isArgRight)
  {
    ROS_ERROR_STREAM("Robot Index/Glove Number and right-handedness do not match! Please launch with correct gloveIndex argument (1 for left, 2 for right)");
    std::exit(1);
  }

  return { glove, std::move(jointList), std::move(urdfModel), robotIndex, isGloveRight };
}

// Utility function to ensure that all necessary keys are present in a given YAML node. Throws an error if any key is missing.
void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& keyList, const std::string& /*object_name*/)
{
  for (const std::string& key : keyList)
  {
    if (!config[key])
    {
      ROS_ERROR_STREAM("Missing Key");
    }
  }
}

// Initializes the URDF model based on the specified device type and handedness. This function sets up the URDF parameters to match the SenseGlove used.
void HardwareBuilder::initUrdf(SGCore::EDeviceType deviceType, bool isRight)
{
  std::string deviceTypeString;
  if (this->urdfInitialize)
  {
    switch (deviceType)
    {
      case EDeviceType::Unknown:
        deviceTypeString = "unknown";
        break;
      case EDeviceType::BetaDevice:
        deviceTypeString = "beta_device";
        break;
      case EDeviceType::SenseGlove:
        deviceTypeString = "dk1";
        break;
      case EDeviceType::Nova:
        deviceTypeString = "nova";
        break;
      case EDeviceType::Nova2:
        deviceTypeString = "nova2";
        break;
    }

    std::string handedness[2] = { "/lh", "/rh" };
              
    std::string robotDescriptor = "/senseglove/" + std::to_string(int((gloveIndex) / 2)) + handedness[int(isRight)] + "/robot_description";
    ROS_INFO_STREAM("Hardware Builder: Looking for robot description: " << robotDescriptor);

    if (!this->urdfModel.initParam(robotDescriptor))
    {
      ROS_ERROR_STREAM("Hardware Builder: Failed initializing the URDF: " << deviceTypeString);
    }
    this->urdfInitialize = false;
  }
}

// Parses the joint configurations from the YAML file. Creates a list of SGHardware::Joint objects that match the specifications in the URDF model.
std::vector<SGHardware::Joint> HardwareBuilder::createJoints(const YAML::Node& jointsConfig) const
{
  ROS_INFO_STREAM("Hardware Builder: Creating Joints");
  std::vector<SGHardware::Joint> joints;
  for (const YAML::Node& jointConfig : jointsConfig)
  {
    const auto jointName = jointConfig.begin()->first.as<std::string>();
    const auto urdfJoint = this->urdfModel.getJoint(jointName);
    if (urdfJoint->type == urdf::Joint::FIXED)
    {
      ROS_WARN_STREAM("Joint: " << jointName << " is fixed in the URDF, but defined in the robot yaml");
    }
    joints.push_back(HardwareBuilder::createJoint(jointConfig[jointName], jointName, urdfJoint));
  }

  for (const auto& urdfJoint : this->urdfModel.joints_)
  {
    if (urdfJoint.second->type != urdf::Joint::FIXED)
    {
      auto equalsJointName = [&](const auto& joint) { return joint.getName() == urdfJoint.first; };
      auto result = std::find_if(joints.begin(), joints.end(), equalsJointName);
      if (result == joints.end())
      {
        ROS_WARN_STREAM("Joint: " <<  urdfJoint.first << " in URDF is not defined in robot yaml");
      }
    }
  }

  joints.shrink_to_fit();
  return joints;
}

// This function creates a SenseGloveRobot for each glove, from the list of gloves and their configurations. Ensures correct set-up based on the YAML and URDF configurations.
std::vector<SGHardware::SenseGloveRobot> HardwareBuilder::createRobots(const YAML::Node& robotsConfig, urdf::Model urdfModel, std::vector<SGHardware::Joint> jointList, std::vector<std::shared_ptr<HapticGlove>> allGloves) const
{
  std::vector<SGHardware::SenseGloveRobot> robots;

  int i = 0;
  for (auto& glove : allGloves)
  {
    robots.push_back(HardwareBuilder::createRobot(robotsConfig, urdfModel, std::move(jointList), glove, i, true));  // dubious
                                                                                                                // fix
    i++;
  }

  robots.shrink_to_fit();
  return robots;
}

// A helper function to select the correct glove from a list, based on the specified hand orientation and glove number.

std::shared_ptr<HapticGlove> HardwareBuilder::correctGlove(std::vector<std::shared_ptr<HapticGlove>> gloves) const
{
  int mod = gloveIndex % 2;
  auto choiceA = gloves[gloveIndex];
  bool notEqual = choiceA->IsRight() xor isRight;

  if (mod == 0 and notEqual)
  {
    return gloves[gloveIndex + 1];
  }
  else if (mod == 1 and notEqual)
  {
    return gloves[gloveIndex - 1];
  }
  return choiceA;
}
