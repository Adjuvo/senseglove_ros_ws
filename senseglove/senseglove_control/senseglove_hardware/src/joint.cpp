// Copyright (c) 2020 - 2025 SenseGlove
#include <senseglove_hardware/joint.hpp>

#include "rcutils/logging_macros.h"

namespace SGHardware
{
  Joint::Joint(std::string jointName, int jointIndex)
    : jointName(std::move(jointName)), jointIndex(jointIndex)
  {
  }

  Joint::Joint(std::string jointName,
               int jointIndex, 
               ActuationType actuationType, 
               ActuationMode actuationMode, 
               bool allowActuation)
    : jointName(std::move(jointName)) 
    , jointIndex(jointIndex)
    , actuationType(actuationType)
    , actuationMode(actuationMode)
    , allowActuation(allowActuation)
  {
  }

  Joint::Joint(std::string jointName,
               int jointIndex, 
               ActuationType actuationType, 
               ActuationMode actuationMode, 
               bool allowActuation, 
               std::unique_ptr<EFinger> finger_ptr)
    : jointName(std::move(jointName)) 
    , jointIndex(jointIndex)
    , actuationType(actuationType)
    , actuationMode(actuationMode)
    , allowActuation(allowActuation)
    , finger(std::move(finger_ptr))
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
      RCUTILS_LOG_ERROR_NAMED(
        "senseglove.joint",
        "Failed to prepare joint '%s' for actuation", this->jointName.c_str());
      return;
    }

    RCUTILS_LOG_INFO_NAMED(
      "senseglove.joint",
      "Preparing '%s' for actuation", this->jointName.c_str());
    this->position = this->readAngle();
    this->velocity = 0.0;
    RCUTILS_LOG_INFO_NAMED(
      "senseglove.joint",
      "Successfully prepared '%s' for actuation", this->jointName.c_str());
  }

  double Joint::readAngle()
  {
    // get angle from finger array at correct index; placeholder:
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

  double Joint::getTorque() const
  {
    // Compute torque; placeholder:
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

  ActuationType Joint::getActuationType() const
  {
    return actuationType;
  }

  ActuationMode Joint::getActuationMode() const
  {
    return actuationMode;
  }
}  // namespace SGHardware
