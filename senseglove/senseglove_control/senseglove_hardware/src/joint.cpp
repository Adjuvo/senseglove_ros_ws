// Copyright (c) 2020 - 2026 SenseGlove

#include <rclcpp/rclcpp.hpp>

#include <senseglove_hardware/joint.hpp>

namespace SGHardware
{

Joint::Joint(std::string jointName, int jointIndex)
  : jointName_(std::move(jointName)), jointIndex_(jointIndex)
{
}

Joint::Joint(std::string jointName,
             int jointIndex,
             ActuationType actuationType,
             ActuationMode actuationMode,
             bool allowActuation)
  : jointName_(std::move(jointName))
  , jointIndex_(jointIndex)
  , actuationType_(actuationType)
  , actuationMode_(actuationMode)
  , allowActuation_(allowActuation)
{
}

Joint::Joint(std::string jointName,
             int jointIndex,
             ActuationType actuationType,
             ActuationMode actuationMode,
             bool allowActuation,
             std::unique_ptr<SGCore::EFinger> finger)
  : jointName_(std::move(jointName))
  , jointIndex_(jointIndex)
  , actuationType_(actuationType)
  , actuationMode_(actuationMode)
  , allowActuation_(allowActuation)
  , finger_(std::move(finger))
{
}

void Joint::prepareActuation()
{
  if (!canActuate())
  {
    RCLCPP_WARN(rclcpp::get_logger("senseglove.joint"),
                "Cannot prepare joint '%s' for actuation - actuation not allowed",
                jointName_.c_str());
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("senseglove.joint"),
               "Preparing joint '%s' for actuation",
               jointName_.c_str());

  position_ = 0.0;
  prevPosition_ = position_;
  velocity_ = 0.0;
}

}  // namespace SGHardware