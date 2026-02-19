// Copyright (c) 2020 - 2026 SenseGlove

#include "senseglove_hardware_builder/hardware_builder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>

#include <SenseGlove.hpp>

namespace senseglove_hardware_builder
{

const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = {
  "allowActuation", "jointIndex", "minPosition", "maxPosition"};
const std::vector<std::string> HardwareBuilder::ROBOT_REQUIRED_KEYS = {"deviceType"};

HardwareBuilder::HardwareBuilder(AllowedRobot robot, const std::string& serial, bool isRight)
  : robotType_(robot.getName()), serial_(serial), isRight_(isRight)
{
}

void HardwareBuilder::setUrdfModel(std::shared_ptr<urdf::Model> urdfModel)
{
  urdfModel_ = std::move(urdfModel);
}

YAML::Node HardwareBuilder::loadRobotConfig() const
{
  std::string package_share =
    ament_index_cpp::get_package_share_directory("senseglove_hardware_builder");
  std::string yaml_path = package_share + "/robots/" + robotType_ + ".yaml";
  return YAML::LoadFile(yaml_path);
}

std::unique_ptr<SGHardware::SenseGloveRobot> HardwareBuilder::createRobot()
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");

  // Check SenseCom is running
  if (!SGCore::DeviceList::SenseComRunning())
    throw std::runtime_error("SenseCom is not running. Start SenseCom before launching.");

  // Get connected gloves
  auto allGloves = SGCore::SG::SenseGlove::GetHapticGloves(true);
  if (allGloves.empty())
    throw std::runtime_error("No SenseGloves detected via SenseCom.");

  RCLCPP_DEBUG(logger, "Connected gloves (%zu):", allGloves.size());
  for (size_t i = 0; i < allGloves.size(); ++i)
  {
    RCLCPP_DEBUG(logger,
                 "  [%zu] %s (%s)",
                 i,
                 allGloves[i]->GetDeviceId().c_str(),
                 allGloves[i]->IsRight() ? "right" : "left");
  }

  // Find glove by serial
  auto hardwareGlove = findGloveBySerial(allGloves);
  if (!hardwareGlove)
  {
    std::string available;
    for (const auto& g : allGloves)
      available += "\n  - " + g->GetDeviceId();
    throw std::runtime_error("Glove with serial '" + serial_ +
                             "' not found. Available:" + available);
  }

  // Validate handedness
  bool actualIsRight = hardwareGlove->IsRight();
  if (actualIsRight != isRight_)
  {
    std::string error = "Handedness mismatch for glove " + serial_ + ": expected " +
                        (isRight_ ? "RIGHT" : "LEFT") + ", got " +
                        (actualIsRight ? "RIGHT" : "LEFT");
    RCLCPP_ERROR(logger, "%s", error.c_str());
    throw std::runtime_error(error);
  }

  // Check URDF
  if (!urdfModel_)
    throw std::runtime_error("URDF model not set");

  // Load robot configuration
  YAML::Node robotYaml = loadRobotConfig();
  const auto robotName = robotYaml.begin()->first.as<std::string>();
  YAML::Node config = robotYaml[robotName];

  validateRequiredKeysExist(config, ROBOT_REQUIRED_KEYS, "robot");

  // Create joints
  auto joints = createJoints(config["joints"]);
  RCLCPP_DEBUG(logger, "Created %zu joints", joints.size());

  // Create and return robot
  auto robot = std::make_unique<SGHardware::SenseGloveRobot>(
    hardwareGlove, std::move(joints), urdfModel_, serial_, actualIsRight);

  RCLCPP_DEBUG(logger,
               "Robot ready: %s (%s)",
               robot->getRobotName().c_str(),
               actualIsRight ? "right" : "left");

  return robot;
}

std::shared_ptr<SGCore::HapticGlove> HardwareBuilder::findGloveBySerial(
  const std::vector<std::shared_ptr<SGCore::HapticGlove>>& gloves) const
{
  for (const auto& glove : gloves)
  {
    if (glove->GetDeviceId().find(serial_) != std::string::npos)
      return glove;
  }
  return nullptr;
}

SGHardware::Joint HardwareBuilder::createJoint(const YAML::Node& jointConfig,
                                               const std::string& jointName,
                                               const urdf::JointConstSharedPtr& urdfJoint)
{
  if (!urdfJoint)
    throw std::runtime_error("No URDF joint found for: " + jointName);

  validateRequiredKeysExist(jointConfig, JOINT_REQUIRED_KEYS, jointName);

  int jointIndex = jointConfig["jointIndex"] ? jointConfig["jointIndex"].as<int>() : -1;
  bool allowActuation = jointConfig["allowActuation"].as<bool>(false);

  SGHardware::ActuationMode actuationMode = SGHardware::ActuationMode::effort;
  SGHardware::ActuationType actuationType = SGHardware::ActuationType::brake;

  if (jointConfig["actuationMode"])
    actuationMode = SGHardware::ActuationMode(jointConfig["actuationMode"].as<std::string>());
  if (jointConfig["actuationType"])
    actuationType = SGHardware::ActuationType(jointConfig["actuationType"].as<std::string>());

  return {jointName, jointIndex, actuationType, actuationMode, allowActuation};
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config,
                                                const std::vector<std::string>& keyList,
                                                const std::string& objectName)
{
  std::vector<std::string> missingKeys;
  for (const auto& key : keyList)
    if (!config[key])
      missingKeys.push_back(key);

  if (!missingKeys.empty())
  {
    std::string error = "Missing required keys in '" + objectName + "': ";
    for (size_t i = 0; i < missingKeys.size(); ++i)
    {
      if (i > 0)
        error += ", ";
      error += missingKeys[i];
    }
    throw std::runtime_error(error);
  }
}

std::vector<SGHardware::Joint> HardwareBuilder::createJoints(const YAML::Node& jointsConfig) const
{
  auto logger = rclcpp::get_logger("senseglove.hardware_builder");
  std::vector<SGHardware::Joint> joints;
  joints.reserve(jointsConfig.size());

  for (const auto& jointNode : jointsConfig)
  {
    const auto jointName = jointNode.begin()->first.as<std::string>();
    const auto urdfJoint = urdfModel_->getJoint(jointName);

    if (urdfJoint && urdfJoint->type == urdf::Joint::FIXED)
      RCLCPP_WARN(
        logger, "Joint '%s' is fixed in URDF but defined in robot config", jointName.c_str());

    joints.push_back(createJoint(jointNode[jointName], jointName, urdfJoint));
  }

  // Warn about URDF joints not in config
  for (const auto& [urdfJointName, urdfJoint] : urdfModel_->joints_)
  {
    if (urdfJoint->type != urdf::Joint::FIXED)
    {
      auto it = std::find_if(joints.begin(), joints.end(), [&](const SGHardware::Joint& j) {
        return j.getName() == urdfJointName;
      });
      if (it == joints.end())
        RCLCPP_WARN(logger, "URDF joint '%s' not defined in robot config", urdfJointName.c_str());
    }
  }

  return joints;
}

}  // namespace senseglove_hardware_builder