// Copyright 2020 senseglove
#include "senseglove_hardware_interface/senseglove_hardware_interface.h"

#include <cstdlib>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware_builder/hardware_builder.h>

std::unique_ptr<senseglove::SenseGloveSetup> build(AllowedRobot robot);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "senseglove_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);

    if (argc < 2)
    {
        ROS_FATAL("Missing robot argument\nusage: SG_hardware_interface_node ROBOT");
        return 1;
    }
    AllowedRobot selected_robot = AllowedRobot(argv[1]);
    ROS_INFO_STREAM("Selected robot: " << selected_robot);

    spinner.start();

    SenseGloveHardwareInterface SenseGlove(build(selected_robot));

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

    while (ros::ok())
    {
        try
        {
            const ros::Time now = ros::Time::now();
            ros::Duration elapsed_time = now - last_update_time;
            last_update_time = now;

            SenseGlove.read(now, elapsed_time);
            SenseGlove.validate();
            controller_manager.update(now, elapsed_time);
            SenseGlove.write(now, elapsed_time);
        }
        catch (const std::exception& e)
        {
            ROS_FATAL("Hardware interface caught an exception during update");
            ROS_FATAL("%s", e.what());
            return 1;
        }
    }

    return 0;
}

std::unique_ptr<senseglove::SenseGloveSetup> build(AllowedRobot robot)
{
    HardwareBuilder builder(robot);
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

