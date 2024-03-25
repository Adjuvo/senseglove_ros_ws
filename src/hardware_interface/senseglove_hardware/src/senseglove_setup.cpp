// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"
#include "senseglove_hardware/senseglove_setup.h"

#include <algorithm>

#include <ros/ros.h>

namespace SGHardware
{
  SenseGloveSetup::SenseGloveSetup(std::vector<SGHardware::SenseGloveRobot> SGRobots)
    : SGRobots(std::move(SGRobots))
  {
  }

  SenseGloveSetup::SenseGloveSetup(SGHardware::SenseGloveRobot sensegloveRobot)
  {
    SGRobots.push_back(std::move(sensegloveRobot));
  }

  void SenseGloveSetup::startCommunication(bool /*reset*/)
  {
    if (DeviceList::SenseComRunning())
    {
      ROS_WARN_STREAM("SenseGloveSetup: Trying to start communication when Sensecom is already running.");
      return;
    }
  }

  void SenseGloveSetup::stopCommunication()
  {
    this->getSenseGloveRobot(0).stopActuating();
  }

  bool SenseGloveSetup::isCommunicationOperational()
  {
    if (DeviceList::SenseComRunning())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(::std::string gloveName)
  {
    if (!DeviceList::SenseComRunning())
    {
      ROS_WARN_STREAM("SenseGloveSetup: Trying to access SGRobot while SenseCom communication is not operational.");
    }
    for (auto& SGRobot : SGRobots)
    {
      if (SGRobot.getRobotName() == gloveName)
      {
        return SGRobot;
      }
    }

    throw std::out_of_range("Could not find glove with name " + gloveName);
  }

  SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(int index)
  {
    if (!SGCore::DeviceList::SenseComRunning())
    {
      ROS_WARN_STREAM("SenseGloveSetup: Trying to access SGRobot while SenseCom communication is not operational.");
    }
    return SGRobots.at(index);
  }

  size_t SenseGloveSetup::size() const
  {
    return SGRobots.size();
  }

  SenseGloveSetup::iterator SenseGloveSetup::begin()
  {
    if (!SGCore::DeviceList::SenseComRunning())
    {
      ROS_WARN_STREAM("SenseGloveSetup: Trying to begin SenseGloveSetup iterator while SenseCom communication is not operational.");
    }
    return this->SGRobots.begin();
  }

  SenseGloveSetup::iterator SenseGloveSetup::end()
  {
    return this->SGRobots.end();
  }

  SenseGloveSetup::~SenseGloveSetup()
  {
    stopCommunication();
  }

  const urdf::Model& SenseGloveSetup::getRobotUrdf(std::string robotName)
  {
    return this->getSenseGloveRobot(robotName).getUrdf();
  }

}  // namespace SGHardware
