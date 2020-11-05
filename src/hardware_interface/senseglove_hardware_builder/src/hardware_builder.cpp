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

// clang-format off
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation", "jointIndex", "minPosition", "maxPosition" };
const std::vector<std::string> HardwareBuilder::ROBOT_REQUIRED_KEYS = { "deviceType" };
// clang-format on

HardwareBuilder::HardwareBuilder(AllowedRobot robot) : HardwareBuilder(robot.getFilePath())
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot, urdf::Model urdf)
        : robot_config_(YAML::LoadFile(robot.getFilePath())), urdf_(std::move(urdf)), init_urdf_(false)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path) : robot_config_(YAML::LoadFile(yaml_path))
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
//    const auto cycle_time = config["communicationCycleTime"].as<int>();
//    const auto device_type = config["deviceType"].as<int>();

    if (!SGConnect::ScanningActive())
    {
      if(SGConnect::Init() != 1)
      {
        ROS_WARN("Something went wrong trying to initiate SGConnect");
      }
    }
    else
    {
      ROS_WARN("SGConnect Scanning is already Active! Will not instantiate a new SGConnect object");
    }
    std::vector<SGCore::SG::SenseGlove> all_gloves = SGCore::SG::SenseGlove::GetSenseGloves();

    this->initUrdf(all_gloves[0].GetDeviceType());

    std::vector<senseglove::Joint> joints = this->createJoints(config["joints"]);

    std::vector<senseglove::SenseGloveRobot> sensegloves = this->createRobots(config, this->urdf_, std::move(joints), all_gloves);

    ROS_INFO_STREAM("Robot config:\n" << config);
    return std::make_unique<senseglove::SenseGloveSetup>(std::move(sensegloves));
}

senseglove::Joint HardwareBuilder::createJoint(const YAML::Node& joint_config, const std::string& joint_name,
                                          const urdf::JointConstSharedPtr& urdf_joint)
{
    ROS_DEBUG("Starting creation of joint %s", joint_name.c_str());
    if (!urdf_joint)
    {
        ROS_ERROR("No URDF joint given for joint %s", joint_name);
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

    return { joint_name, joint_index, allow_actuation };
}

senseglove::SenseGloveRobot HardwareBuilder::createRobot(const YAML::Node& robot_config, urdf::Model urdf, std::vector<senseglove::Joint> jointList, SGCore::SG::SenseGlove glove, int robot_index)
{
    ROS_DEBUG("Starting creation of glove %d", robot_index);
    HardwareBuilder::validateRequiredKeysExist(robot_config, HardwareBuilder::ROBOT_REQUIRED_KEYS, "glove");

    return { glove, std::move(jointList), std::move(urdf), robot_index};
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

void HardwareBuilder::initUrdf(SGCore::DeviceType type)
{
    std::string type_string;
    if (this->init_urdf_)
    {
        switch(type)
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

        if (!this->urdf_.initParam("/robot_description_" + type_string))
        {
            ROS_ERROR("Failed initializing the URDF: %s", type_string);
//            throw senseglove::error::HardwareException(senseglove::error::ErrorType::INIT_URDF_FAILED);
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
        joints.push_back(
                HardwareBuilder::createJoint(joint_config[joint_name], joint_name, urdf_joint));
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


std::vector<senseglove::SenseGloveRobot> HardwareBuilder::createRobots(const YAML::Node& robots_config, urdf::Model urdf, std::vector<senseglove::Joint> jointList, std::vector<SGCore::SG::SenseGlove> all_gloves) const
{
    std::vector<senseglove::SenseGloveRobot> robots;

    int i = 0;
    for (auto& glove : all_gloves)
    {
//        const auto robot_name = robots_config.begin()->first.as<std::string>();
        robots.push_back(
                HardwareBuilder::createRobot(robots_config, urdf, std::move(jointList), glove, i));
        i++;
    }

    robots.shrink_to_fit();
    return robots;
}
