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

std::unique_ptr<senseglove::SenseGloveSetup> build(AllowedRobot robot, int nr_of_glove, bool is_right);
bool to_bool(std::string str);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "senseglove_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  int publish_rate;
  bool pr_param_fail = false;

  if (argc < 3)
  {
    ROS_FATAL("Missing robot arguments\nusage: senseglove_hardware_interface_node ROBOT nr_of_glove is_right");
    return 1;
  }
  AllowedRobot selected_robot = AllowedRobot(argv[1]);
  int nr_of_glove = std::stoi(argv[2]);
  bool is_right = to_bool(argv[3]);
  ROS_INFO_STREAM("Selected robot: " << selected_robot);

  spinner.start();

  SenseGloveHardwareInterface SenseGlove(build(selected_robot, nr_of_glove, is_right));
  ROS_DEBUG_STREAM("Successfully built the robot");

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
    ROS_FATAL("Hardware interface caught an exception during init");
    ROS_FATAL("%s", e.what());
    std::exit(1);
  }

  controller_manager::ControllerManager controller_manager(&SenseGlove, nh);
  ros::Time last_update_time = ros::Time::now();

  try
  {
    ros::param::get("/senseglove/0/lh/controller/hand_state/publish_rate", publish_rate);
  }
  catch (...)
  {
    pr_param_fail = true;
    ROS_ERROR("Failed to obtain the left handed publish rate");
  }
  try
  {
    ros::param::get("/senseglove/0/rh/controller/hand_state/publish_rate", publish_rate);
  }
  catch (...)
  {
    pr_param_fail = true;
    ROS_ERROR("Failed to obtain the right handed publish rate");
  }

  if (pr_param_fail)
  {
    ROS_FATAL("publish rate for left and right hands is not published");
    std::exit(1);
  }

  ros::Rate rate(publish_rate); // ROS Rate at 5Hz

  while (ros::ok())
  {
    try
    {
      const ros::Time now = ros::Time::now();
      ros::Duration elapsed_time = now - last_update_time;
      last_update_time = now;

      SenseGlove.read(now, elapsed_time);
      controller_manager.update(now, elapsed_time);
      SenseGlove.write(now, elapsed_time);
    }
    catch (const std::exception& e)
    {
      ROS_FATAL("Hardware interface caught an exception during update");
      ROS_FATAL("%s", e.what());
      return 1;
    }
    rate.sleep();
  }
  ros::spin();
  return 0;
}

std::unique_ptr<senseglove::SenseGloveSetup> build(AllowedRobot robot, int nr_of_glove, bool is_right)
{
  HardwareBuilder builder(robot, nr_of_glove, is_right);
  try
  {
    return builder.createSenseGloveSetup();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Hardware interface caught an exception during building hardware");
    ROS_FATAL("%s", e.what());
    std::exit(1);
  }
}

bool to_bool(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::istringstream is(str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}
