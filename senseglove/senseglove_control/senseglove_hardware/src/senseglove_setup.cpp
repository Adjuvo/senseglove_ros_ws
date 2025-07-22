// Copyright (c) 2020 - 2025 SenseGlove

#include <rclcpp/rclcpp.hpp>
#include <senseglove_hardware/senseglove_setup.hpp>

namespace SGHardware
{
  SenseGloveSetup::SenseGloveSetup(RobotPtr robot)
  {
    if (robot) {
      SGRobots.push_back(std::move(robot));
      buildMap();
    }
  }

  SenseGloveSetup::SenseGloveSetup(std::vector<RobotPtr> robots)
    : SGRobots(std::move(robots))
  {
    buildMap();
  }

  void SenseGloveSetup::buildMap()
  {
    nameMap.clear();
    for (size_t i = 0; i < SGRobots.size(); ++i) {
      const std::string & name = SGRobots[i]->getRobotName();
      nameMap[name] = i;
    }
  }

 void SenseGloveSetup::startCommunication(bool /*reset*/)
  {
    if (SGCore::DeviceList::SenseComRunning())
    {
      RCLCPP_WARN(
        rclcpp::get_logger("senseglove_setup"),
        "Trying to start communication when SenseCom is already running.");
      return;
    }
  }

  void SenseGloveSetup::stopCommunication()
  {
    if (SGRobots.empty()) 
    {
      RCLCPP_WARN(
        rclcpp::get_logger("senseglove_setup"),
        "No robots in setup to stop communication.");
      return;
    }

    try 
    {
      SGRobots.front()->stopHaptics();
      RCLCPP_INFO(
        rclcpp::get_logger("senseglove_setup"),
        "Stopped haptics on SenseGloveRobot.");
    } 
    catch (const std::exception & e) 
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("senseglove_setup"),
        "Error stopping haptics: %s", e.what());
    }
  }

  bool SenseGloveSetup::isCommunicationOperational()
  {
    bool running = SGCore::DeviceList::SenseComRunning();
    return running;
  }

  SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(const std::string & gloveName)
  {
    if (!SGCore::DeviceList::SenseComRunning())
    {
      RCLCPP_WARN(
        rclcpp::get_logger("senseglove_setup"),
        "Accessing robot '%s' while SenseCom is not operational.",
        gloveName.c_str());
    }

    auto it = nameMap.find(gloveName);
    if (it == nameMap.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("senseglove_setup"),
        "Could not find glove with name '%s'",
        gloveName.c_str());
      throw std::out_of_range("Could not find glove with name " + gloveName);
    }

    return *SGRobots[it->second];
  }


  SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(size_t index)
  {
    if (!SGCore::DeviceList::SenseComRunning())
    {
      RCLCPP_WARN(
        rclcpp::get_logger("senseglove_setup"),
        "Accessing robot at index %zu while SenseCom is not operational.",
        index);
    }
    if (index >= SGRobots.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("senseglove_setup"),
        "Index out of range: %zu (SGRobots size: %zu)",
        index, SGRobots.size());
      throw std::out_of_range("Index out of range in SenseGloveSetup");
    }
    return *SGRobots[index];
  }

  const urdf::Model& SenseGloveSetup::getRobotUrdf(const std::string & robotName)
  {
    return this->getSenseGloveRobot(robotName).getUrdf();
  }
} // namespace SGHardware
