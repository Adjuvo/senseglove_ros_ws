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
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Trying to start communication when SenseCom is already running.");
      return;
    }
  }

  void SenseGloveSetup::stopCommunication()
  {
    if (SGRobots.empty()) 
    {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "No robots in setup to stop communication.");
      return;
    }

    try 
    {
      SGRobots.front()->stopHaptics();
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Stopped haptics on SenseGloveRobot.");
    } 
    catch (const std::exception & e) 
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Error stopping haptics: " << e.what());
    }
  }

  bool SenseGloveSetup::isCommunicationOperational() const
  {
    bool running = SGCore::DeviceList::SenseComRunning();
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("senseglove_setup"),
      "SenseComRunning() = " << std::boolalpha << running);
    return running;
  }

  const SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(const std::string & gloveName) const
  {
    if (!SGCore::DeviceList::SenseComRunning())
    {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Accessing robot '" << gloveName
        << "' while SenseCom is not operational.");
    }

    auto it = nameMap.find(gloveName);
    if (it == nameMap.end()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Could not find glove with name '" << gloveName << "'");
      throw std::out_of_range("Could not find glove with name " + gloveName);
    }
    return *SGRobots[it->second];
  }

  const SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(size_t index) const
  {
    if (!SGCore::DeviceList::SenseComRunning())
    {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Accessing robot at index " << index
        << " while SenseCom is not operational.");
    }
    if (index >= SGRobots.size()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("senseglove_setup"),
        "Index out of range: " << index);
      throw std::out_of_range("Index out of range in SenseGloveSetup");
    }
    return *SGRobots[index];
  }

  const urdf::Model& SenseGloveSetup::getRobotUrdf(const std::string & robotName) const
  {
    return this->getSenseGloveRobot(robotName).getUrdf();
  }
} // namespace SGHardware
