#ifndef ROS_WORKSPACE_ALLOWED_ROBOT_H
#define ROS_WORKSPACE_ALLOWED_ROBOT_H

#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

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
    rembrandt
  };

  AllowedRobot() = default;

  explicit AllowedRobot(const std::string& robotName)
  {
    if (robotName == "dk1_left")
    {
      this->value = dk1_left;
    }
    else if (robotName == "dk1_right")
    {
      this->value = dk1_right;
    }
    else if (robotName == "nova_left")
    {
      this->value = nova_left;
    }
    else if (robotName == "nova_right")
    {
      this->value = nova_right;
    }
    else if (robotName == "nova2_left")
    {
      this->value = nova2_left;
    }
    else if (robotName == "nova2_right")
    {
      this->value = nova2_right;
    }
    else
    {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("allowed_robot"),
        "Unknown robot '" << robotName << "', defaulting to 'dk1_left'");
      this->value = AllowedRobot::dk1_left;
    }
  }

  std::string getFilePath()
  {
    std::string basePath = ament_index_cpp::get_package_share_directory("senseglove_hardware_builder");
    if (this->value == AllowedRobot::dk1_left)
    {
      return basePath + "/robots/dk1_left.yaml";
    }
    else if (this->value == AllowedRobot::dk1_right)
    {
      return basePath + "/robots/dk1_right.yaml";
    }
    else if (this->value == AllowedRobot::nova_left)
    {
      return basePath + "/robots/nova_left.yaml";
    }
    else if (this->value == AllowedRobot::nova_right)
    {
      return basePath + "/robots/nova_right.yaml";
    }
    else if (this->value == AllowedRobot::nova2_left)
    {
      return basePath + "/robots/nova2_left.yaml";
    }
    else if (this->value == AllowedRobot::nova2_right)
    {
      return basePath + "/robots/nova2_right.yaml";
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("allowed_robot"),
        "AllowedRobot: Robot name not implemented. Using 'dk1_left.yaml'.");
      return basePath + "/robots/dk1_left.yaml";
    }
  }

  constexpr AllowedRobot(Value allowedRobot) : value(allowedRobot) {}

  bool operator==(AllowedRobot a) const
  {
    return value == a.value;
  }
  bool operator!=(AllowedRobot a) const
  {
    return value != a.value;
  }

  friend std::ostream& operator<<(std::ostream& out, const AllowedRobot& c)
  {
    switch (c.value)
    {
      case dk1_left: out << "dk1_left"; break;
      case dk1_right: out << "dk1_right"; break;
      case nova_left: out << "nova_left"; break;
      case nova_right: out << "nova_right"; break;
      case nova2_left: out << "nova2_left"; break;
      case nova2_right: out << "nova2_right"; break;
      default: out << "(Unknown)"; break;
    }
    return out;
  }

private:
  Value value;
};

#endif  // ROS_WORKSPACE_ALLOWED_ROBOT_H
