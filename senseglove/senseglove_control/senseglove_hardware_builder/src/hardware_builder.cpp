// Copyright (c) 2020 - 2025 SenseGlove
#include "senseglove_hardware_builder/hardware_builder.hpp"

#include "rclcpp/rclcpp.hpp"

const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation", "jointIndex", "minPosition", "maxPosition" };
const std::vector<std::string> HardwareBuilder::ROBOT_REQUIRED_KEYS = { "deviceType" };

HardwareBuilder::HardwareBuilder(AllowedRobot robot, int gloveIndex, bool isRight)
  : HardwareBuilder(robot.getFilePath(), gloveIndex, isRight)
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot, urdf::Model urdfModel)
  : robotConfig(YAML::LoadFile(robot.getFilePath())), urdfModel(std::move(urdfModel))
{
}

HardwareBuilder::HardwareBuilder(const std::string& yamlPath, int gloveIndex, bool isRight)
  : robotConfig(YAML::LoadFile(yamlPath)), gloveIndex(gloveIndex), isRight(isRight)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yamlPath, urdf::Model urdfModel)
  : robotConfig(YAML::LoadFile(yamlPath)), urdfModel(std::move(urdfModel))
{
}


void HardwareBuilder::setUrdfModel(urdf::Model urdfModel)
{
    this->urdfModel = std::move(urdfModel);
}

// Create SenseGloveSetup
std::unique_ptr<SGHardware::SenseGloveSetup> HardwareBuilder::createSenseGloveSetup()
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  if (!DeviceList::SenseComRunning())
  {
    RCLCPP_ERROR_STREAM(logger, "SenseCom is not running. Ensure that the SenseGlove communication service is active.");
    throw std::runtime_error("SenseCom is not running");
  }

  const auto robotName = this->robotConfig.begin()->first.as<std::string>();
  YAML::Node config = this->robotConfig[robotName];

  auto allGloves = SGCore::SG::SenseGlove::GetHapticGloves(true);
  auto currentGlove = allGloves[gloveIndex];

  RCLCPP_INFO_STREAM(logger, "Obtained the following gloves:");
  for (auto& glove : allGloves)
  {
    RCLCPP_INFO_STREAM(logger, " - " << glove->GetDeviceId());
  }

  if (DeviceList::SenseComRunning())
  {
    auto node = std::make_shared<rclcpp::Node>("senseglove_hardware_builder");
    std::string robot_namespace = "/senseglove/glove" + std::to_string(gloveIndex) + (isRight ? "/rh" : "/lh");
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger, "No SenseGloves connected!");
    std::exit(1);
  }

  auto joints = this->createJoints(config["joints"]);
  RCLCPP_INFO_STREAM(logger, "Created Joints: " << joints.size());

  auto SGRobot = HardwareBuilder::createRobot(config, this->urdfModel, std::move(joints), currentGlove, gloveIndex, isRight);

  RCLCPP_INFO_STREAM(logger, "Created Robot is a " << SGRobot.getRobotName()
                     << " / Right: " << SGRobot.getRight()
                     << " / URDF-Right: " << currentGlove->IsRight());
  RCLCPP_INFO_STREAM(logger, "Robot config:\n" << config);

  auto robotPtr = std::make_unique<SGHardware::SenseGloveRobot>(std::move(SGRobot));
  return std::make_unique<SGHardware::SenseGloveSetup>(std::move(robotPtr));
}


// Create Joint
SGHardware::Joint HardwareBuilder::createJoint(const YAML::Node& jointConfig, const std::string& jointName, const urdf::JointConstSharedPtr& urdfJoint)
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  RCLCPP_DEBUG_STREAM(logger, "Starting creation of joint: " << jointName);

  if (!urdfJoint)
  {
    throw std::runtime_error("No URDF joint found for joint: " + jointName);
  }
  validateRequiredKeysExist(jointConfig, HardwareBuilder::JOINT_REQUIRED_KEYS, jointName);

  int jointIndex = jointConfig["jointIndex"] ? jointConfig["jointIndex"].as<int>() : -1;
  bool allowActuation = jointConfig["allowActuation"].as<bool>(false);

  if (!jointConfig["jointIndex"])
  {
    RCLCPP_WARN_STREAM(logger, "Joint: " << jointName << " does not have a netNumber");
  }

  SGHardware::ActuationMode actuationMode = SGHardware::ActuationMode::position;
  SGHardware::ActuationType actuationType = SGHardware::ActuationType::brake;

  if (jointConfig["actuationMode"])
  {
    actuationMode = SGHardware::ActuationMode(jointConfig["actuationMode"].as<std::string>());
  }

  if (jointConfig["actuationType"])
  {
    actuationType = SGHardware::ActuationType(jointConfig["actuationType"].as<std::string>());
  }

  return {jointName, jointIndex, actuationType, actuationMode, allowActuation};
}

// Construct SenseGloveRobot object
SGHardware::SenseGloveRobot HardwareBuilder::createRobot(
  const YAML::Node& robotConfig, urdf::Model urdfModel, std::vector<SGHardware::Joint> jointList,
  std::shared_ptr<HapticGlove> glove, int robotIndex, bool isArgRight)
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  RCLCPP_DEBUG_STREAM(logger, "Starting creation of glove: " << robotIndex);
  validateRequiredKeysExist(robotConfig, ROBOT_REQUIRED_KEYS, "glove");

  bool isGloveRight = glove->IsRight();
  if (isGloveRight xor isArgRight)
  {
    RCLCPP_ERROR_STREAM(logger, "Robot Index / Glove Number and right-handedness do not match! "
                        "Please launch with correct gloveIndex argument.");
    std::exit(1);
  }

  return { glove, std::move(jointList), std::move(urdfModel), robotIndex, isGloveRight };
}

// Utility function to ensure that all necessary keys are present in a given YAML node
void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& keyList, const std::string& /*object_name*/)
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  for (const auto& key : keyList)
  {
    if (!config[key])
    {
      RCLCPP_ERROR_STREAM(logger, "Missing Key: " << key);
    }
  }
}

// Creates a list of SGHardware::Joint objects
std::vector<SGHardware::Joint> HardwareBuilder::createJoints(const YAML::Node& jointsConfig) const
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  std::vector<SGHardware::Joint> joints;
  for (const auto& jointConfig : jointsConfig)
  {
    const auto jointName = jointConfig.begin()->first.as<std::string>();
    const auto urdfJoint = this->urdfModel.getJoint(jointName);
    if (urdfJoint && urdfJoint->type == urdf::Joint::FIXED)
    {
      RCLCPP_WARN_STREAM(logger, "Joint: " << jointName << " is fixed in the URDF, but defined in the robot yaml.");
    }
    joints.push_back(createJoint(jointConfig[jointName], jointName, urdfJoint));
  }

  for (const auto& urdfJoint : this->urdfModel.joints_)
  {
    if (urdfJoint.second->type != urdf::Joint::FIXED)
    {
      auto equalsJointName = [&](const auto& joint) { return joint.getName() == urdfJoint.first; };
      auto result = std::find_if(joints.begin(), joints.end(), equalsJointName);
      if (result == joints.end())
      {
        RCLCPP_WARN_STREAM(logger, "Joint: " << urdfJoint.first << " in URDF not defined in robot yaml");
      }
    }
  }

  joints.shrink_to_fit();
  return joints;
}

// Creates a SenseGloveRobot for each glove,
std::vector<SGHardware::SenseGloveRobot> HardwareBuilder::createRobots(
  const YAML::Node& robotsConfig, urdf::Model urdfModel, std::vector<SGHardware::Joint> jointList,
  std::vector<std::shared_ptr<HapticGlove>> allGloves) const
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  std::vector<SGHardware::SenseGloveRobot> robots;
  int i = 0;
  for (auto& glove : allGloves)
  {
    robots.push_back(createRobot(robotsConfig, urdfModel, std::move(jointList), glove, i, true));
    i++;
  }
  robots.shrink_to_fit();
  return robots;
}

// Helper function to select the correct glove from a list, based on the specified hand orientation and glove number
std::shared_ptr<HapticGlove> HardwareBuilder::correctGlove(std::vector<std::shared_ptr<HapticGlove>> gloves) const
{
  int mod = gloveIndex % 2;
  auto choiceA = gloves[gloveIndex];
  bool notEqual = choiceA->IsRight() xor isRight;

  if (mod == 0 && notEqual)
  {
    return gloves[gloveIndex + 1];
  }
  else if (mod == 1 && notEqual)
  {
    return gloves[gloveIndex - 1];
  }
  return choiceA;
}