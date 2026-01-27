#ifndef ROS_WORKSPACE_HARDWARE_BUILDER_H
#define ROS_WORKSPACE_HARDWARE_BUILDER_H

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>

#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <senseglove_hardware/actuation_mode.hpp>
#include <senseglove_hardware/joint.hpp>
#include <senseglove_hardware/senseglove_robot.hpp>
#include <senseglove_hardware/senseglove_setup.hpp>

#include <senseglove_hardware_builder/allowed_robot.hpp>

// SenseGlove API headers
#include <SenseGlove.hpp>
#include <HapticGlove.hpp>

// Creates a SenseGloveRobot from a robot yaml and URDF
class HardwareBuilder
{
public:
  // Required keys for YAML validation
  static const std::vector<std::string> JOINT_REQUIRED_KEYS;
  static const std::vector<std::string> ROBOT_REQUIRED_KEYS;

  // Constructors
  explicit HardwareBuilder(AllowedRobot robot, int gloveIndex, bool isRight);
  HardwareBuilder(AllowedRobot robot, std::shared_ptr<urdf::Model> urdfModel);
  explicit HardwareBuilder(const std::string& yamlPath, int gloveIndex, bool isRight);
  HardwareBuilder(const std::string& yamlPath, std::shared_ptr<urdf::Model> urdfModel);
  explicit HardwareBuilder(AllowedRobot robot, int gloveIndex, bool isRight, const std::string& gloveSerial);

  void setUrdfModel(std::shared_ptr<urdf::Model> urdfModel);

  // Build SenseGlove robot setup
  std::unique_ptr<SGHardware::SenseGloveSetup> createSenseGloveSetup();

  // Static helpers
  static void validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& keyList, const std::string& objectName);
  static SGHardware::Joint createJoint(const YAML::Node& jointConfig, const std::string& jointName, const urdf::JointConstSharedPtr& urdfJoint);
  static SGHardware::SenseGloveRobot createRobot(const YAML::Node& robotConfig,
                                                 std::shared_ptr<urdf::Model> urdfModel,
                                                 std::vector<SGHardware::Joint> joints,
                                                 std::shared_ptr<SGCore::HapticGlove> glove,
                                                 int robotIndex, bool isArgRight);
private:
  // Internal helpers
  std::vector<SGHardware::Joint> createJoints(const YAML::Node& joints_config) const;
  std::vector<SGHardware::SenseGloveRobot> createRobots(const YAML::Node& allRobotConfig, 
                                                        std::shared_ptr<urdf::Model> urdfModel,
                                                        std::vector<SGHardware::Joint> joints,
                                                        std::vector<std::shared_ptr<SGCore::HapticGlove>> allGloves) const;
  // Current Glove
  std::shared_ptr<SGCore::HapticGlove> getGloveBySerial(std::vector<std::shared_ptr<SGCore::HapticGlove>> gloves, const std::string& targetSerial) const;

  // Member data
  YAML::Node robotConfig;
  std::shared_ptr<urdf::Model> urdfModel;
  int gloveIndex;
  bool isRight;
  std::string gloveSerial = "";
};

#endif  // ROS_WORKSPACE_HARDWARE_BUILDER_H
