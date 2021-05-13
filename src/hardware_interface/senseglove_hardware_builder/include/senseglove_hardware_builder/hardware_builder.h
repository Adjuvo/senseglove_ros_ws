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

/**
 * @brief Creates a SenseGloveRobot from a robot yaml and URDF.
 */
class HardwareBuilder
{
public:
  /**
   * @brief Initialises a HardwareBuilder with a robotName enumerator.
   * @details Grabs the .yaml file associated with the robot name.
   */
  explicit HardwareBuilder(AllowedRobot robot, int nr_of_glove, bool is_right);

  /**
   * @brief Initialises with a robot name and URDF.
   */
  HardwareBuilder(AllowedRobot robot, urdf::Model urdf);

  /**
   * @brief Initialises a HardwareBuilder with a path to a .yaml file.
   */
  explicit HardwareBuilder(const std::string& yaml_path, int nr_of_glove, bool is_right);

  /**
   * @brief Initialises with a path to yaml and URDF.
   */
  HardwareBuilder(const std::string& yaml_path, urdf::Model urdf);

  /**
   * @brief Creates a SenseGloveRobot. Loads a URDF from the parameter server
   * on topic `/robot_description` when no URDF was given in the constructor.
   *
   * @throws HardwareConfigException When the urdf could not be loaded from the parameter server
   * @throws MissingKeyException When a required key is missing from the given config
   */
  std::unique_ptr<senseglove::SenseGloveSetup> createSenseGloveSetup();

  /**
   * @brief Loops over all keys in the keyList and check if they exist in the
   * config.
   *
   * @throws MissingKeyException when required keys are missing.
   */
  static void validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& key_list,
                                        const std::string& object_name);

  static senseglove::Joint createJoint(const YAML::Node& joint_config, const std::string& joint_name,
                                       const urdf::JointConstSharedPtr& urdf_joint);
  static senseglove::SenseGloveRobot createRobot(const YAML::Node& joint_config, urdf::Model urdf,
                                                 std::vector<senseglove::Joint> joints, SGCore::SG::SenseGlove glove,
                                                 int robot_index, bool is_arg_right);

  static const std::vector<std::string> JOINT_REQUIRED_KEYS;
  static const std::vector<std::string> ROBOT_REQUIRED_KEYS;

private:
  /**
   * Initializes the URDF if necessary.
   */
  void initUrdf(SGCore::DeviceType type, bool is_right);

  /**
   * Returns all joints found in the given config.
   * Warns when joints are defined as FIXED in the URDF and when a non-FIXED
   * joint is not contained in the config.
   * @param joints_config YAML node that contains a sequence of joint objects
   * @return list of created joints
   */
  std::vector<senseglove::Joint> createJoints(const YAML::Node& joints_config) const;
  std::vector<senseglove::SenseGloveRobot> createRobots(const YAML::Node& robots_config, urdf::Model urdf,
                                                        std::vector<senseglove::Joint> joints,
                                                        std::vector<SGCore::SG::SenseGlove> all_gloves) const;
  SGCore::SG::SenseGlove correct_glove(std::vector<SGCore::SG::SenseGlove> gloves) const;

  YAML::Node robot_config_;
  urdf::Model urdf_;
  bool init_urdf_ = true;
  int nr_of_glove_;
  bool is_right_;
};

#endif  // ROS_WORKSPACE_HARDWARE_BUILDER_H
