/**
 * @file
 *
 * @author Akshay Radhamohan Menon <akshay@senseglove.com>
 * 
 * @section LICENSE
 * Copyright (c) 2020 - 2024 SenseGlove *
 * 
 * @section DESCRIPTION
 * 
 * A class to represent different actuation types with sg representatives
 */

#ifndef SENSEGLOVE_HARDWARE_ACTUATION_TYPE_H
#define SENSEGLOVE_HARDWARE_ACTUATION_TYPE_H

#include <string>
#include <ros/console.h>

namespace SGHardware
{
  class ActuationType
  {
  public:
    // Enum representing different actuation types
    enum Value : int
    {
      brake,
      vibration,
      squeeze,
      absent,
    };

    // Constructors
    ActuationType() : value_(absent) {}
    ActuationType(Value value) : value_(value) {}

    // Constructor from string, parsing the input string to set the Type
    explicit ActuationType(const std::string& actuationType)
    {
      if (actuationType == "brake")
      {
        this->value_ = brake;
      }
      else if (actuationType == "vibration")
      {
        this->value_ = vibration;
      }
      else if (actuationType == "squeeze")
      {
        this->value_ = squeeze;
      }
      else if (actuationType == "absent")
      {
        this->value_ = absent;
      }
      else
      {
        ROS_WARN("Actuation type (%s) is not recognized; setting to absent Type", actuationType.c_str());
        this->value_ = ActuationType::absent;
      }
    }

    // Method for conversion to numerical representation
    uint8_t toTypeNumber()
    {
      switch (this->value_)
      {
        case brake:
          return 1;
        case vibration:
          return 2;
        case squeeze:
          return 3;
        default:
          return 0;
      }
    }

    // Method for conversion to string representation
    std::string toString() const
    {
      switch (this->value_)
      {
        case brake:
          return "brake";
        case vibration:
          return "vibration";
        case squeeze:
          return "squeeze";
        default:
          ROS_WARN("ActuationType (%i) is neither 'brake' or 'vibration", this->value_);
          return "absent";
      }
    }

    // Returns the current enum value
    int getValue() const
    {
      return this->value_;
    }

    // Comparison operators
    bool operator==(ActuationType::Value a) const
    {
      return this->value_ == a;
    }

    bool operator!=(ActuationType::Value a) const
    {
      return this->value_ != a;
    }

  private:
    // Current enum value
    Value value_ = absent;
  };
}  // namespace SGHardware

#endif  // SENSEGLOVE_HARDWARE_ACTUATION_TYPE_H
