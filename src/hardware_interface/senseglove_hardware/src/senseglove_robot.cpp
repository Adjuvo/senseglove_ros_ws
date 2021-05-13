// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include "SenseGlove.h"
#include "BasicHandModel.h"
#include "HandPose.h"
#include "DeviceList.h"

namespace senseglove
{
SenseGloveRobot::SenseGloveRobot(SGCore::SG::SenseGlove glove, ::std::vector<Joint> jointList, urdf::Model urdf,
                                 int robotIndex, bool is_right)
  : senseglove_(glove)
  , hand_profile_(SGCore::SG::SG_HandProfile::Default(is_right))
  , hand_model_(SGCore::Kinematics::BasicHandModel::Default(is_right))
  , joint_list_(std::move(jointList))
  , urdf_(std::move(urdf))
  , name_("senseglove/" + std::to_string(int((robotIndex) / 2)))
  , device_type_(this->senseglove_.GetDeviceType())
  , robot_index_(robotIndex)
  , is_right_(is_right)
  , updated_(false)
{
}

std::string SenseGloveRobot::getName() const
{
  return this->name_;
}

int SenseGloveRobot::getIndex() const
{
  return this->robot_index_;
}

bool SenseGloveRobot::getRight()
{
  return this->senseglove_.IsRight();
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

SGCore::Kinematics::Vect3D SenseGloveRobot::getHandPos(int i)
{
  // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
  // SG uses vector of vectors and ROS uses one long array
  return hand_pose_.jointPositions[std::floor(i / 4)][i % 4];
}

SGCore::Kinematics::Vect3D SenseGloveRobot::getFingerTip(int i)
{
  // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
  // SG uses vector of vectors and ROS uses one long array
  return glove_pose_.CalculateFingerTips(hand_profile_)[i];
}

void SenseGloveRobot::actuateEffort(std::vector<double> effort_command)
{
  if (SGCore::DeviceList::SenseCommRunning())  // check if the Sense Comm is running. If not, warn the end user.
  {
    std::vector<int> int_effort(effort_command.begin(), effort_command.end());
    if (effort_command[0] + effort_command[1] + effort_command[2] + effort_command[3] + effort_command[4] <
        10.0)  // less than noticable ffb
    {
      this->senseglove_.SendHaptics(SGCore::Haptics::SG_FFBCmd(SGCore::Haptics::SG_FFBCmd::off));
    }
    else
    {
      this->senseglove_.SendHaptics(
          SGCore::Haptics::SG_FFBCmd(int_effort[0], int_effort[1], int_effort[2], int_effort[3], int_effort[4]));
    }
  }
}

void SenseGloveRobot::actuateEffort(double e_0, double e_1, double e_2, double e_3, double e_4)
{
  std::vector<double> efforts = { e_0, e_1, e_2, e_3, e_4 };
  this->actuateEffort(efforts);
}

void SenseGloveRobot::actuateBuzz(std::vector<double> buzz_command)
{
  std::vector<int> int_buzz(buzz_command.begin(), buzz_command.end());
  if (buzz_command[0] + buzz_command[1] + buzz_command[2] + buzz_command[3] + buzz_command[4] < 10.0)  // less than
                                                                                                       // noticable buzz
  {
    this->senseglove_.SendHaptics(SGCore::Haptics::SG_BuzzCmd(SGCore::Haptics::SG_BuzzCmd::off));
  }
  else
  {
    this->senseglove_.SendHaptics(
        SGCore::Haptics::SG_BuzzCmd(int_buzz[0], int_buzz[1], int_buzz[2], int_buzz[3], int_buzz[4]));
  }
}
void SenseGloveRobot::actuateBuzz(double b_0, double b_1, double b_2, double b_3, double b_4)
{
  std::vector<double> buzzes = { b_0, b_1, b_2, b_3, b_4 };
  this->actuateBuzz(buzzes);
}

void SenseGloveRobot::stopActuating()
{
  this->senseglove_.StopHaptics();
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

bool SenseGloveRobot::updateGloveData(const ros::Duration period)
{
  bool glove_update = false;
  bool hand_update = false;
  if (senseglove_.GetSensorData(sensor_data_))  // if GetSensorData is true, we have sucesfully recieved data
  {
    // ROS_DEBUG("successfully update glove sensor data");
    for (auto& joint : joint_list_)
    {
      joint.position_ = sensor_data_.sensorAngles[joint.joint_index_ / 4][joint.joint_index_ % 4];
      double intermediate_vel =
          (sensor_data_.sensorAngles[joint.joint_index_ / 4][joint.joint_index_ % 4] - joint.velocity_);
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
  else
  {
    this->tip_positions_ = this->glove_pose_.CalculateFingerTips(this->hand_profile_);
    glove_update = true;
  }
  if (!senseglove_.GetHandPose(this->hand_model_, this->hand_profile_, this->hand_pose_))
  {
    ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data");
  }
  else
  {
    hand_update = true;
  }
  updated_ |= (glove_update and hand_update);
  return updated_;
}

const urdf::Model& SenseGloveRobot::getUrdf() const
{
  return this->urdf_;
}

}  // namespace senseglove
