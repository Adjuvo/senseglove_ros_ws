// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"
#include "senseglove_hardware/senseglove_setup.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace senseglove
{
SenseGloveSetup::SenseGloveSetup(std::vector<senseglove::SenseGloveRobot> sensegloves)
  : sensegloves_(std::move(sensegloves))
{
}

SenseGloveSetup::SenseGloveSetup(senseglove::SenseGloveRobot sensegloves)
{
  sensegloves_.push_back(std::move(sensegloves));
}

void SenseGloveSetup::startCommunication(bool /*reset*/)
{
  if (SGCore::DeviceList::SenseCommRunning())
  {
    ROS_WARN("Trying to start senseglove communication while it is already active.");
    return;
  }
}

void SenseGloveSetup::stopCommunication()
{
  this->getSenseGloveRobot(0).stopActuating();
}

bool SenseGloveSetup::isCommunicationOperational()
{
  if (SGCore::DeviceList::SenseCommRunning())
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
  if (!SGCore::DeviceList::SenseCommRunning())
  {
    ROS_WARN("Trying to access joints while communication is not operational. This "
             "may lead to incorrect sensor data.");
  }
  for (auto& glove : sensegloves_)
  {
    if (glove.getName() == gloveName)
    {
      return glove;
    }
  }

  throw std::out_of_range("Could not find glove with name " + gloveName);
}

SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(int index)
{
  if (!SGCore::DeviceList::SenseCommRunning())
  {
    ROS_WARN("Trying to access joints while communication is not operational. This "
             "may lead to incorrect sensor data.");
  }
  return sensegloves_.at(index);
}

size_t SenseGloveSetup::size() const
{
  return sensegloves_.size();
}

SenseGloveSetup::iterator SenseGloveSetup::begin()
{
  if (!SGCore::DeviceList::SenseCommRunning())
  {
    ROS_WARN("Trying to access sensegloves while communication is not operational. This "
             "may lead to incorrect sensor data.");
  }
  return this->sensegloves_.begin();
}

SenseGloveSetup::iterator SenseGloveSetup::end()
{
  return this->sensegloves_.end();
}

SenseGloveSetup::~SenseGloveSetup()
{
  stopCommunication();
}

const urdf::Model& SenseGloveSetup::getRobotUrdf(std::string glove_robot_name)
{
  return this->getSenseGloveRobot(glove_robot_name).getUrdf();
}

}  // namespace senseglove
