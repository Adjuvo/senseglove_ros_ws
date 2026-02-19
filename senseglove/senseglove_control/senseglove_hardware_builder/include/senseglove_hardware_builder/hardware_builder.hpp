#ifndef SENSEGLOVE_HARDWARE_BUILDER_HPP
#define SENSEGLOVE_HARDWARE_BUILDER_HPP

#include <urdf/model.h>

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

#include <HapticGlove.hpp>
#include <senseglove_hardware/joint.hpp>
#include <senseglove_hardware/senseglove_robot.hpp>
#include <senseglove_hardware_builder/allowed_robot.hpp>

namespace senseglove_hardware_builder
{

// Builds a single SenseGloveRobot instance from configuration
class HardwareBuilder
{
public:
  static const std::vector<std::string> JOINT_REQUIRED_KEYS;
  static const std::vector<std::string> ROBOT_REQUIRED_KEYS;

  HardwareBuilder() = default;

  // Construct builder for a specific robot configuration
  HardwareBuilder(AllowedRobot robot, const std::string& serial, bool isRight);

  // Set the URDF model for the robot
  void setUrdfModel(std::shared_ptr<urdf::Model> urdfModel);

  // Create the SenseGloveRobot instance
  std::unique_ptr<SGHardware::SenseGloveRobot> createRobot();

  // Static helpers
  static void validateRequiredKeysExist(const YAML::Node& config,
                                        const std::vector<std::string>& keyList,
                                        const std::string& objectName);

  static SGHardware::Joint createJoint(const YAML::Node& jointConfig,
                                       const std::string& jointName,
                                       const urdf::JointConstSharedPtr& urdfJoint);

private:
  std::string robotType_;
  std::string serial_;
  bool isRight_ = false;
  std::shared_ptr<urdf::Model> urdfModel_;

  // Load robot configuration YAML
  YAML::Node loadRobotConfig() const;

  // Create joints from YAML configuration
  std::vector<SGHardware::Joint> createJoints(const YAML::Node& jointsConfig) const;

  // Find a connected glove by serial number
  std::shared_ptr<SGCore::HapticGlove> findGloveBySerial(
    const std::vector<std::shared_ptr<SGCore::HapticGlove>>& gloves) const;
};

}  // namespace senseglove_hardware_builder

#endif  // SENSEGLOVE_HARDWARE_BUILDER_HPP