// Copyright 2020 Senseglove.
#ifndef ROS_WORKSPACE_ALLOWED_ROBOT_H
#define ROS_WORKSPACE_ALLOWED_ROBOT_H

#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

class AllowedRobot
{
public:
  enum Value : int
  {
    dk1,
    fino,
  };

  AllowedRobot() = default;
  explicit AllowedRobot(const std::string& robot_name)
  {
    if (robot_name == "dk1")
    {
      this->value = dk1;
    }
    else if (robot_name == "fino")
    {
      this->value = fino;
    }
    else
    {
      ROS_WARN_STREAM("Unknown robot " << robot_name);
      this->value = AllowedRobot::dk1;
    }
  }

  std::string getFilePath()
  {
    std::string base_path = ros::package::getPath("senseglove_hardware_builder");
    if (this->value == AllowedRobot::dk1)
    {
      return base_path.append("/robots/dk1.yaml");
    }
    else if (this->value == AllowedRobot::fino)
    {
      return base_path.append("/robots/fino.yaml");
    }
    ROS_ERROR("Robotname not implemented. Using controllers.yaml...");
    return base_path.append("/robots/dk1.yaml");
  }

  constexpr AllowedRobot(Value allowed_robot) : value(allowed_robot)
  {
  }

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
      case dk1:
        out << "dk1";
        break;
      case fino:
        out << "fino";
        break;
      default:
        out << "(Unknown)";
        break;
    }
    return out;
  }

private:
  Value value;
};

#endif  // ROS_WORKSPACE_ALLOWED_ROBOT_H
