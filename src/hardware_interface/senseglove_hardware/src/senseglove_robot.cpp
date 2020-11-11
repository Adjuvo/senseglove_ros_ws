// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace senseglove
{
SenseGloveRobot::SenseGloveRobot(SGCore::SG::SenseGlove glove, ::std::vector<Joint> jointList, urdf::Model urdf, int robotIndex)
  : senseglove_(glove), joint_list_(std::move(jointList)), urdf_(std::move(urdf)), device_type_(this->senseglove_.GetDeviceType()), robot_index_(robotIndex)
{
}

std::string SenseGloveRobot::getName() const
{
  return this->name_;
}

Joint& SenseGloveRobot::getJoint(::std::string jointName)
{
  for (auto& joint : joint_list_)
  {
    if (joint.getName() == jointName)
    {
      return joint;
    }
  }
  throw std::out_of_range("Could not find joint with name " + jointName);
}

Joint& SenseGloveRobot::getJoint(size_t index)
{
  return this->joint_list_.at(index);
}

size_t SenseGloveRobot::size() const
{
  return this->joint_list_.size();
}

SenseGloveRobot::iterator SenseGloveRobot::begin()
{
  return this->joint_list_.begin();
}

SenseGloveRobot::iterator SenseGloveRobot::end()
{
  return this->joint_list_.end();
}

SenseGloveRobot::~SenseGloveRobot()
{
}

void SenseGloveRobot::updateGloveData(const ros::Duration period)
{
  if (senseglove_.GetSensorData(sensor_data_))  // if GetSensorData is true, we have sucesfully recieved data
  {
    // ROS_DEBUG("successfully update glove sensor data");
    for (auto& joint : joint_list_)
    {
      joint.position_ = sensor_data_.sensorAngles[joint.joint_index_ / 4][joint.joint_index_ % 4];
      double intermediate_vel = (sensor_data_.sensorAngles[joint.joint_index_ / 4][joint.joint_index_ % 4] - joint.velocity_);
      if (intermediate_vel != 0.0 and period.toSec() != 0.0)
      {
        joint.velocity_ = intermediate_vel / 1;
      }
      else
      {
        joint.velocity_ = 0.0;
      }
    }
  }
  if (!senseglove_.GetGlovePose(glove_pose_))
  {
    ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data");
  }
}

const urdf::Model& SenseGloveRobot::getUrdf() const
{
  return this->urdf_;
}

}  // namespace senseglove
