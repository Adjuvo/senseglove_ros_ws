// Copyright 2020 Senseglove.
#ifndef ROS_WORKSPACE_ALLOWED_ROBOT_H
#define ROS_WORKSPACE_ALLOWED_ROBOT_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

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
        this->value = nova_right;
      }
      else
      {
        ROS_WARN_STREAM("Unknown robot " << robotName);
        this->value = AllowedRobot::dk1_left; //Defaulting to left because the left glove is initialized before the right glove 
      }
    }

    std::string getFilePath()
    {
      std::string basePath = ros::package::getPath("senseglove_hardware_builder");
      if (this->value == AllowedRobot::dk1_left)
      {
        return basePath.append("/robots/dk1_left.yaml");
      }
      else if (this->value == AllowedRobot::dk1_right)
      {
        return basePath.append("/robots/dk1_right.yaml");
      }
      else if (this->value == AllowedRobot::nova_left)
      {
        return basePath.append("/robots/nova_left.yaml");
      }
      else if (this->value == AllowedRobot::nova_right)
      {
        return basePath.append("/robots/nova_right.yaml");
      }
      else
      {
      ROS_ERROR_STREAM("Robotname not implemented. Using controllers.yaml.");
      return basePath.append("/robots/dk1_left.yaml");
      }
    }

    constexpr AllowedRobot(Value allowedRobot) : value(allowedRobot)
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
        case nova_left:
          out << "nova_left";
          break;
        case nova_right:
          out << "nova_right";
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
