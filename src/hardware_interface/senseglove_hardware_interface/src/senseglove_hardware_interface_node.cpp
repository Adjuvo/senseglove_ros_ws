// Copyright 2020 senseglove
#include "senseglove_hardware_interface/senseglove_hardware_interface.h"

#include <cstdlib>
#include <sstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <cctype>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware_builder/hardware_builder.h>

std::unique_ptr<SGHardware::SenseGloveSetup> build(AllowedRobot selectedRobot, int gloveIndex, bool isRight);
bool toBool(std::string str);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "senseglove_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);

  int publishRate;
  bool publishRateParameterFail = false;

  if (argc < 3)
  {
    ROS_FATAL_STREAM("Senseglove HW Interface Node: Missing robot arguments. Usage: senseglove_hardware_interface_node Robot gloveIndex isRight");
    return 1;
  }
  AllowedRobot selectedRobot = AllowedRobot(argv[1]);
  int gloveIndex = std::stoi(argv[2]);
  bool isRight = toBool(argv[3]);

  ROS_INFO_STREAM("Senseglove HW Interface Node: Selected robot: " << selectedRobot);
  spinner.start();

  SenseGloveHardwareInterface SenseGlove(build(selectedRobot, gloveIndex, isRight));
  ROS_INFO_STREAM("Senseglove HW Interface Node: Successfully built the robot");
  try
  {
    bool success = SenseGlove.init(nh, nh);
    if (!success)
    {
      std::exit(1);
    }
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Senseglove HW Interface Node: Hardware interface caught an exception during INITIALIZATION");
    ROS_FATAL_STREAM(e.what());
    std::exit(1);
  }
  controller_manager::ControllerManager controllerManager(&SenseGlove, nh);
  ros::Time lastUpdateTime = ros::Time::now(); 

  try
  {
    ros::param::get("/senseglove/0/lh/controller/hand_state/publish_rate", publishRate);
  }
  catch (...)
  {
    publishRateParameterFail = true;
    ROS_ERROR_STREAM("Senseglove HW Interface Node: Failed to obtain the left handed publish rate");
  }
  try
  {
    ros::param::get("/senseglove/0/rh/controller/hand_state/publish_rate", publishRate);
  }
  catch (...)
  {
    publishRateParameterFail = true;
    ROS_ERROR_STREAM("Senseglove HW Interface Node: Failed to obtain the right handed publish rate");
  }

  if (publishRateParameterFail)
  {
    ROS_FATAL_STREAM("Senseglove HW Interface Node: Publish rate for left and right hands is not published");
    std::exit(1);
  }

  ros::Rate rate(publishRate); // ROS Rate at 5Hz

  while (ros::ok())
  {
    try
    {
      const ros::Time now = ros::Time::now();
      ros::Duration elapsedTime = now - lastUpdateTime;
      lastUpdateTime = now;

      SenseGlove.read(now, elapsedTime);
      controllerManager.update(now, elapsedTime);
      SenseGlove.write(now, elapsedTime);
    }
    catch (const std::exception& e)
    {
      ROS_FATAL_STREAM("Senseglove HW Interface Node: Hardware interface caught an exception during UPDATE");
      ROS_FATAL_STREAM(e.what());
      return 1;
    }
    rate.sleep();
  }
  ros::waitForShutdown();
  return 0;
}

std::unique_ptr<SGHardware::SenseGloveSetup> build(AllowedRobot selectedRobot, int gloveIndex, bool isRight)
{
  HardwareBuilder builder(selectedRobot, gloveIndex, isRight);
  try
  {
    return builder.createSenseGloveSetup();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Senseglove HW Interface Node: Hardware interface caught an exception during BUILDING HARDWARE");
    ROS_FATAL_STREAM(e.what());
    std::exit(1);
  }
}

bool toBool(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::istringstream is(str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}
