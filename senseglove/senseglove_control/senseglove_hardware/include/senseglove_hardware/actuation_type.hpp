#ifndef SENSEGLOVE_HARDWARE_ACTUATION_TYPE_HPP
#define SENSEGLOVE_HARDWARE_ACTUATION_TYPE_HPP

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace SGHardware
{
class ActuationType
{
public:
  // Actuation types
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
  explicit ActuationType(const std::string& actuationType) : value_(absent)
  {
    if (actuationType == "brake")
    {
      value_ = brake;
    }
    else if (actuationType == "vibration")
    {
      value_ = vibration;
    }
    else if (actuationType == "squeeze")
    {
      value_ = squeeze;
    }
    else if (actuationType == "absent")
    {
      value_ = absent;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("senseglove.actuation_type"),
                  "Actuation type '%s' is not recognized; setting to 'absent'",
                  actuationType.c_str());
      value_ = absent;
    }
  }

  // Method for conversion to numerical representation
  uint8_t toTypeNumber() const
  {
    switch (value_)
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
    switch (value_)
    {
      case brake:
        return "brake";
      case vibration:
        return "vibration";
      case squeeze:
        return "squeeze";
      default:
        RCLCPP_WARN(
          rclcpp::get_logger("senseglove.actuation_type"),
          "ActuationType value '%d' is not 'brake', 'vibration', or 'squeeze'; returning 'absent'",
          static_cast<int>(value_));
        return "absent";
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

#endif  // SENSEGLOVE_HARDWARE_ACTUATION_TYPE_HPP