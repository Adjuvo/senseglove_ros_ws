#ifndef SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
#define SENSEGLOVE_HARDWARE_ACTUATION_MODE_H

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace SGHardware
{
  class ActuationMode
  {
  public:
    // Enum representing different actuation modes
    enum Value : int
    {
      position,
      torque,
      unknown,
    };

    // Constructors
    ActuationMode() : value_(unknown) {}
    ActuationMode(Value value) : value_(value) {}

    // Constructor from string, parsing the input string to set the mode
    explicit ActuationMode(const std::string& actuationMode)
      : value_(unknown)
    {
      if (actuationMode == "position")
      {
        value_ = position;
      }
      else if (actuationMode == "torque")
      {
        value_ = torque;
      }
      else if (actuationMode == "unknown")
      {
        value_ = unknown;
      }
      else
      {
        RCLCPP_WARN(
          rclcpp::get_logger("senseglove.actuation_mode"),
          "Actuation mode '%s' is not recognized; setting to unknown mode",
          actuationMode.c_str());
        value_ = unknown;
      }
    }

    // Method for conversion to numerical representation
    uint8_t toModeNumber() const
    {
      switch (value_)
      {
        case position:
          return 1;
        case torque:
          return 2;
        default:
          return 0;
      }
    }

    // Method for conversion to string representation
    std::string toString() const
    {
      switch (value_)
      {
        case position:
          return "position";
        case torque:
          return "torque";
        default:
          RCLCPP_WARN(
          rclcpp::get_logger("senseglove.actuation_mode"),
            "ActuationMode value '%d' is neither 'torque' nor 'position'; returning 'unknown'",
            static_cast<int>(value_));
          return "unknown";
      }
    }

    // Returns the current enum value
    int getValue() const
    {
      return static_cast<int>(value_);
    }

    // Comparison operators
    bool operator==(Value a) const
    {
      return value_ == a;
    }

    bool operator!=(Value a) const
    {
      return value_ != a;
    }

  private:
    Value value_;
  };
}  // namespace SGHardware

#endif  // SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
