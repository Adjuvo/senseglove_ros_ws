// Copyright 2020 Senseglove
#include "senseglove_hardware_builder/hardware_builder.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include <ros/ros.h>

#include <SGConnect.h>
#include "SenseGlove.h"

const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation", "jointIndex", "minPosition",
                                                                        "maxPosition" };
const std::vector<std::string> HardwareBuilder::ROBOT_REQUIRED_KEYS = { "deviceType" };

HardwareBuilder::HardwareBuilder(AllowedRobot robot, int nr_of_glove, bool is_right)
  : HardwareBuilder(robot.getFilePath(), nr_of_glove, is_right)
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot, urdf::Model urdf)
  : robot_config_(YAML::LoadFile(robot.getFilePath())), urdf_(std::move(urdf)), init_urdf_(false)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path, int nr_of_glove, bool is_right)
  : robot_config_(YAML::LoadFile(yaml_path)), nr_of_glove_(nr_of_glove), is_right_(is_right)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path, urdf::Model urdf)
  : robot_config_(YAML::LoadFile(yaml_path)), urdf_(std::move(urdf)), init_urdf_(false)
{
}

std::unique_ptr<senseglove::SenseGloveSetup> HardwareBuilder::createSenseGloveSetup()
{
  const auto robot_name = this->robot_config_.begin()->first.as<std::string>();
  ROS_DEBUG_STREAM("Starting creation of robot " << robot_name);

  // Remove top level robot name key
  YAML::Node config = this->robot_config_[robot_name];
  ROS_DEBUG_STREAM("Size of robot config " << this->robot_config_.size());
  std::vector<SGCore::SG::SenseGlove> all_gloves = SGCore::SG::SenseGlove::GetSenseGloves();
  auto current_glove = all_gloves[nr_of_glove_];  // Will be update later
  ROS_DEBUG_STREAM("creating sensegloves");

  if (SGCore::DeviceList::SenseCommRunning())
  {
    ROS_INFO("Obtained the following gloves: ");
    for (auto& all_glove : all_gloves)
    {
      ROS_INFO("%s", all_glove.ToString().c_str());
    }
    current_glove = correct_glove(all_gloves);
    this->initUrdf(current_glove.GetDeviceType(), current_glove.IsRight());
    ROS_DEBUG_STREAM("Obtained devicetype from glove");
  }
  else
  {
    ROS_ERROR("No Sensegloves connected");
    std::exit(1);
  }

  std::vector<senseglove::Joint> joints = this->createJoints(config["joints"]);
  ROS_INFO_STREAM("Created joints " << nr_of_glove_);
  senseglove::SenseGloveRobot sensegloves =
      HardwareBuilder::createRobot(config, this->urdf_, std::move(joints), current_glove, nr_of_glove_, is_right_);
  ROS_INFO_STREAM("Created Robots " << sensegloves.getName() << ", " << nr_of_glove_ << ", is right: "
                                    << sensegloves.getRight() << " is urdfright: " << current_glove.IsRight());
  ROS_INFO_STREAM("Robot config:\n" << config);
  return std::make_unique<senseglove::SenseGloveSetup>(std::move(sensegloves));
}

senseglove::Joint HardwareBuilder::createJoint(const YAML::Node& joint_config, const std::string& joint_name,
                                               const urdf::JointConstSharedPtr& urdf_joint)
{
  ROS_DEBUG("Starting creation of joint %s", joint_name.c_str());
  if (!urdf_joint)
  {
    ROS_ERROR("No URDF joint given for joint %s", joint_name.c_str());
  }
  HardwareBuilder::validateRequiredKeysExist(joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint");

  auto joint_index = -1;
  if (joint_config["jointIndex"])
  {
    joint_index = joint_config["jointIndex"].as<int>();
  }
  else
  {
    ROS_WARN("Joint %s does not have a netNumber", joint_name.c_str());
  }

  const auto allow_actuation = joint_config["allowActuation"].as<bool>();

  senseglove::ActuationMode mode;
  if (joint_config["actuationMode"])
  {
    mode = senseglove::ActuationMode(joint_config["actuationMode"].as<std::string>());
  }

  return { joint_name, joint_index, allow_actuation, mode };
}

senseglove::SenseGloveRobot HardwareBuilder::createRobot(const YAML::Node& robot_config, urdf::Model urdf,
                                                         std::vector<senseglove::Joint> jointList,
                                                         SGCore::SG::SenseGlove glove, int robot_index,
                                                         bool is_arg_right)
{
  ROS_DEBUG("Starting creation of glove %d", robot_index);
  HardwareBuilder::validateRequiredKeysExist(robot_config, HardwareBuilder::ROBOT_REQUIRED_KEYS, "glove");
  bool is_glove_right = glove.IsRight();

  if (is_glove_right xor is_arg_right)
  {
    ROS_ERROR("robot_index/ glove_nr and right-handedness do not match!\n %d, %d"
              "\nPlease launch with correct nr_of_glove argument (1 for left, 2 for right).",
              is_glove_right, is_arg_right);
    std::exit(1);
  }

  return { glove, std::move(jointList), std::move(urdf), robot_index, is_glove_right };
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& key_list,
                                                const std::string& /*object_name*/)
{
  for (const std::string& key : key_list)
  {
    if (!config[key])
    {
      ROS_ERROR("Missing Key");
      //            throw MissingKeyException(key, object_name);
    }
  }
}

void HardwareBuilder::initUrdf(SGCore::DeviceType type, bool is_right)
{
  std::string type_string;
  if (this->init_urdf_)
  {
    switch (type)
    {
      case SGCore::DeviceType::UNKNOWN:
        type_string = "unknown";
        break;
      case SGCore::DeviceType::BETADEVICE:
        type_string = "beta_device";
        break;
      case SGCore::DeviceType::SENSEGLOVE:
        type_string = "dk1";
        break;
      case SGCore::DeviceType::FINO:
        type_string = "fino";
        break;
    }
    std::string handedness[2] = { "/lh", "/rh" };
    std::string robot_descriptor =
        "/senseglove/" + std::to_string(int((nr_of_glove_) / 2)) + handedness[int(is_right)] + "/robot_description";
    ROS_INFO_STREAM("Looking for robot description: " << robot_descriptor);
    if (!this->urdf_.initParam(robot_descriptor))
    {
      ROS_ERROR("Failed initializing the URDF: %s", type_string.c_str());
    }
    this->init_urdf_ = false;
  }
}

std::vector<senseglove::Joint> HardwareBuilder::createJoints(const YAML::Node& joints_config) const
{
  std::vector<senseglove::Joint> joints;
  for (const YAML::Node& joint_config : joints_config)
  {
    const auto joint_name = joint_config.begin()->first.as<std::string>();
    const auto urdf_joint = this->urdf_.getJoint(joint_name);
    if (urdf_joint->type == urdf::Joint::FIXED)
    {
      ROS_WARN("Joint %s is fixed in the URDF, but defined in the robot yaml", joint_name.c_str());
    }
    joints.push_back(HardwareBuilder::createJoint(joint_config[joint_name], joint_name, urdf_joint));
  }

  for (const auto& urdf_joint : this->urdf_.joints_)
  {
    if (urdf_joint.second->type != urdf::Joint::FIXED)
    {
      auto equals_joint_name = [&](const auto& joint) { return joint.getName() == urdf_joint.first; };
      auto result = std::find_if(joints.begin(), joints.end(), equals_joint_name);
      if (result == joints.end())
      {
        ROS_WARN("Joint %s in URDF is not defined in robot yaml", urdf_joint.first.c_str());
      }
    }
  }

  joints.shrink_to_fit();
  return joints;
}

std::vector<senseglove::SenseGloveRobot> HardwareBuilder::createRobots(
    const YAML::Node& robots_config, urdf::Model urdf, std::vector<senseglove::Joint> jointList,
    std::vector<SGCore::SG::SenseGlove> all_gloves) const
{
  std::vector<senseglove::SenseGloveRobot> robots;

  int i = 0;
  for (auto& glove : all_gloves)
  {
    robots.push_back(HardwareBuilder::createRobot(robots_config, urdf, std::move(jointList), glove, i, true));  // dubious
                                                                                                                // fix
    i++;
  }

  robots.shrink_to_fit();
  return robots;
}

SGCore::SG::SenseGlove HardwareBuilder::correct_glove(std::vector<SGCore::SG::SenseGlove> gloves) const
{
  int mod = nr_of_glove_ % 2;
  auto choice_a = gloves[nr_of_glove_];
  bool not_equal = choice_a.IsRight() xor is_right_;
  if (mod == 0 and not_equal)
  {
    return gloves[nr_of_glove_ + 1];
  }
  else if (mod == 1 and not_equal)
  {
    return gloves[nr_of_glove_ - 1];
  }
  return choice_a;
}
