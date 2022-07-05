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
    dk1_left,
    dk1_right,
    fino,
  };

  AllowedRobot() = default;
  explicit AllowedRobot(const std::string& robot_name)
  {
    if (robot_name == "dk1_left")
    {
      this->value = dk1_left;
    }
    else if (robot_name == "dk1_right")
    {
      this->value = dk1_right;
    }
    else if (robot_name == "fino")
    {
      this->value = fino;
    }
    else
    {
      ROS_WARN_STREAM("Unknown robot " << robot_name);
      this->value = AllowedRobot::dk1_left; //Defaulting to left because the left glove is initialized before the right glove 
    }
  }

  std::string getFilePath()
  {
    std::string base_path = ros::package::getPath("senseglove_hardware_builder");
    if (this->value == AllowedRobot::dk1_left)
    {
      return base_path.append("/robots/dk1_left.yaml");
    }
    else if (this->value == AllowedRobot::dk1_right)
    {
      return base_path.append("/robots/dk1_right.yaml");
    }
    else if (this->value == AllowedRobot::fino)
    {
      return base_path.append("/robots/fino.yaml");
    }
    ROS_ERROR("Robotname not implemented. Using controllers.yaml...");
    return base_path.append("/robots/dk1_left.yaml");
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
      case dk1_left:
        out << "dk1_left";
        break;
      case dk1_right:
        out << "dk1_right";
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
