#ifndef SENSEGLOVE_HARDWARE_BUILDER_ALLOWED_ROBOT_HPP
#define SENSEGLOVE_HARDWARE_BUILDER_ALLOWED_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <string>

namespace senseglove_hardware_builder
{

class AllowedRobot
{
public:
  enum Value : int
  {
    dk1_left,
    dk1_right,
    nova_left,
    nova_right,
    nova2_left,
    nova2_right,
    unknown
  };

  AllowedRobot() = default;

  explicit AllowedRobot(const std::string& robotName)
  {
    if (robotName == "dk1_left")
      value_ = dk1_left;
    else if (robotName == "dk1_right")
      value_ = dk1_right;
    else if (robotName == "nova_left")
      value_ = nova_left;
    else if (robotName == "nova_right")
      value_ = nova_right;
    else if (robotName == "nova2_left")
      value_ = nova2_left;
    else if (robotName == "nova2_right")
      value_ = nova2_right;
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("allowed_robot"),
                         "Unknown robot '" << robotName << "', defaulting to 'unknown'");
      value_ = unknown;
    }
  }

  constexpr AllowedRobot(Value v) : value_(v) {}

  std::string getName() const
  {
    switch (value_)
    {
      case dk1_left:
        return "dk1_left";
      case dk1_right:
        return "dk1_right";
      case nova_left:
        return "nova_left";
      case nova_right:
        return "nova_right";
      case nova2_left:
        return "nova2_left";
      case nova2_right:
        return "nova2_right";
      default:
        return "unknown";
    }
  }

  std::string getFilePath() const
  {
    std::string basePath =
      ament_index_cpp::get_package_share_directory("senseglove_hardware_builder");
    return basePath + "/robots/" + getName() + ".yaml";
  }

  Value getValue() const
  {
    return value_;
  }

  bool operator==(AllowedRobot a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(AllowedRobot a) const
  {
    return value_ != a.value_;
  }

  friend std::ostream& operator<<(std::ostream& out, const AllowedRobot& r)
  {
    out << r.getName();
    return out;
  }

private:
  Value value_ = unknown;
};

}  // namespace senseglove_hardware_builder

#endif  // SENSEGLOVE_HARDWARE_BUILDER_ALLOWED_ROBOT_HPP