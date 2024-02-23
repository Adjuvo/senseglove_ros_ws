/**
 * @file
 *
 * @author  Rogier
 * @author  Akshay Radhamohan Menon <akshay@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2024 SenseGlove
 *
 * @section DESCRIPTION
 *
 * A class to represent different actuation modes, with methods for 
 * conversion, comparison, and numerical & string representation of actuation modes.
 */

#ifndef SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
#define SENSEGLOVE_HARDWARE_ACTUATION_MODE_H

#include <string>
#include <ros/console.h>

namespace senseglove
{
  class ActuationMode
  {
  public:
    // <summary> Enum representing different actuation modes </summary>
    enum Value : int
    {
      position,
      torque,
      unknown,
    };

    // <summary> Constructors </summary>
    ActuationMode() : value_(unknown) {}
    ActuationMode(Value value) : value_(value) {}

    // <summary> Constructor from string, parsing the input string to set the mode </summary>
    explicit ActuationMode(const std::string& actuationMode)
    {
      if (actuationMode == "position")
      {
        this->value_ = position;
      }
      else if (actuationMode == "unknown")
      {
        this->value_ = unknown;
      }
      else if (actuationMode == "torque")
      {
        this->value_ = torque;
      }
      else
      {
        ROS_WARN("Actuation mode (%s) is not recognized; setting to unknown mode", actuationMode.c_str());
        this->value_ = ActuationMode::unknown;
      }
    }

    // <summary> Method for conversion to numerical representation </summary>
    uint8_t toModeNumber()
    {
      switch (this->value_)
      {
        case position:
          return 1;
        case torque:
          return 2;
        default:
          return 0;
      }
    }

    // <summary> Method for conversion to string representation </summary>
    std::string toString() const
    {
      switch (this->value_)
      {
        case position:
          return "position";
        case torque:
          return "torque";
        default:
          ROS_WARN("Actuationmode (%i) is neither 'torque' or 'position", this->value_);
          return "unknown";
      }
    }

    // <summary> Returns the current enum value </summary>
    int getValue() const
    {
      return this->value_;
    }

    // <summary> Comparison operators </summary>
    bool operator==(ActuationMode::Value a) const
    {
      return this->value_ == a;
    }

    bool operator!=(ActuationMode::Value a) const
    {
      return this->value_ != a;
    }

  private:
    // <summary> Current enum value </summary>
    Value value_ = unknown;
  };
}  // namespace senseglove

#endif  // SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
