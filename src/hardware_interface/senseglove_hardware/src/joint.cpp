// Copyright 2019 Project March.
#include "senseglove_hardware/joint.h"

#include <ros/ros.h>

#include <bitset>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace SGHardware
{
  Joint::Joint(std::string jointName, int jointIndex) : jointName(std::move(jointName)), jointIndex(jointIndex)
  {
  }

  Joint::Joint(std::string jointName, int jointIndex, ActuationMode actuationMode, bool allowActuation)
    : jointName(std::move(jointName)) 
    , jointIndex(jointIndex)
    , actuationMode(actuationMode)
    , allowActuation(allowActuation)
  {
  }

  Joint::Joint(std::string jointName, int jointIndex, ActuationMode actuationMode, bool allowActuation, std::unique_ptr<EFinger> finger)
    : jointName(std::move(jointName)) 
    , jointIndex(jointIndex)
    , actuationMode(actuationMode)
    , allowActuation(allowActuation)
    , finger(std::move(finger))
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
      ROS_ERROR_STREAM("Failed to prepare joint " << this->jointName << " for actuation");
    }

    ROS_INFO_STREAM("Preparing " << this->jointName  << " for actuation");
    this->position = this->readAngle();
    this->velocity = 0;
    ROS_INFO_STREAM("Successfully prepared " << this->jointName  << " for actuation");
  }

  double Joint::readAngle(/*const ros::Duration& elapsed_time*/)
  {
    // get angle from finger array at correct index
    return 0.0;
  }

  double Joint::getPosition() const
  {
    return this->position;
  }

  double Joint::getVelocity() const
  {
    return this->velocity;
  }

  double Joint::getTorque()
  {
    return 0.0;
  }

  void Joint::setAllowActuation(bool allowActuation)
  {
    this->allowActuation = allowActuation;
  }

  int Joint::getIndex() const
  {
    return this->jointIndex;
  }

  std::string Joint::getName() const
  {
    return this->jointName;
  }

  bool Joint::canActuate() const
  {
    return this->allowActuation;
  }

  ActuationMode Joint::getActuationMode() const
  {
    return actuationMode;
  }
}  // namespace SGHardware
