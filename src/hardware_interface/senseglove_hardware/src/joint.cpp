// Copyright 2019 Project March.
#include "senseglove_hardware/joint.h"

#include <ros/ros.h>

#include <bitset>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace senseglove
{
Joint::Joint(std::string name, int joint_index) : name_(std::move(name)), joint_index_(joint_index)
{
}

Joint::Joint(std::string name, int joint_index, bool allow_actuation, ActuationMode mode)
  : name_(std::move(name)), joint_index_(joint_index), allow_actuation_(allow_actuation), actuation_mode_(mode)
{
}

Joint::Joint(std::string name, int joint_index, bool allow_actuation, ActuationMode mode,
             std::unique_ptr<SGCore::Finger> finger)
  : name_(std::move(name))
  , joint_index_(joint_index)
  , allow_actuation_(allow_actuation)
  , actuation_mode_(mode)
  , finger_(std::move(finger))
{
}

bool Joint::initialize()
{
  return false;
}

void Joint::prepareActuation()
{
  if (!this->canActuate())
  {
    ROS_ERROR("Failed to prepare joint %s for actuation", this->name_.c_str());
  }
  ROS_INFO("[%s] Preparing for actuation", this->name_.c_str());
  this->position_ = this->readAngle();
  this->velocity_ = 0;
  ROS_INFO("[%s] Successfully prepared for actuation", this->name_.c_str());
}

double Joint::readAngle(/*const ros::Duration& elapsed_time*/)
{
  // get angle from finger array at correct index
  return 0.0;
}

double Joint::getPosition() const
{
  return this->position_;
}

double Joint::getVelocity() const
{
  return this->velocity_;
}

double Joint::getTorque()
{
  return 0.0;
}

void Joint::setAllowActuation(bool allow_actuation)
{
  this->allow_actuation_ = allow_actuation;
}

int Joint::getNetNumber() const
{
  return this->joint_index_;
}

std::string Joint::getName() const
{
  return this->name_;
}

bool Joint::canActuate() const
{
  return this->allow_actuation_;
}

ActuationMode Joint::getActuationMode() const
{
  return actuation_mode_;
}
}  // namespace senseglove
