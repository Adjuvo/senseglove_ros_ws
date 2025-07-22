// Copyright (c) 2020 - 2025 SenseGlove

#include <rclcpp/rclcpp.hpp>
#include <senseglove_hardware/joint.hpp>

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

  void Joint::prepareActuation()
  {
    auto logger = rclcpp::get_logger("senseglove.joint");

    if (!this->canActuate())
    {
      RCLCPP_WARN(logger, "Failed to prepare joint '%s' for actuation", this->jointName.c_str());
      return;
    }

    RCLCPP_INFO(logger, "Preparing '%s' for actuation", this->jointName.c_str());
    this->position = this->readAngle();
    this->velocity = 0.0;
    RCLCPP_INFO(logger, "Successfully prepared '%s' for actuation", this->jointName.c_str());
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
