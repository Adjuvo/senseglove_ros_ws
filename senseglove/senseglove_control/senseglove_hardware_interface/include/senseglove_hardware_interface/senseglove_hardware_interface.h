/**
 * @file
 *
 * @author Rogier
 * @author Akshay Radhamohan Menon <akshay@senseglove.com>
 * 
 * @section LICENSE
 * Copyright (c) 2020 - 2024 SenseGlove *
 * 
 * @section DESCRIPTION
 * 
 * ROS-SenseGlove Hardware Interface
 */

#ifndef ROS_WORKSPACE_SENSEGLOVE_HARDWARE_INTERFACE_H
#define ROS_WORKSPACE_SENSEGLOVE_HARDWARE_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <realtime_tools/realtime_publisher.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

// Senseglove
#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware/senseglove_setup.h>
#include <senseglove_hardware_builder/hardware_builder.h>
#include <senseglove_shared_resources_msgs/SenseGloveState.h>

#include <memory>
#include <vector>
#include <chrono>
#include <std_msgs/Float64MultiArray.h>

template <typename T>
using RtPublisherPtr = std::unique_ptr<realtime_tools::RealtimePublisher<T>>;

// HardwareInterface to allow ros_control to actuate our hardware. Register an interface for each joint such that they can be actuated via ros_control.

class SenseGloveHardwareInterface : public hardware_interface::RobotHW
{
public:
  SenseGloveHardwareInterface(std::unique_ptr<SGHardware::SenseGloveSetup> setup);

  // Initialize the HardwareInterface by registering position interfaces for each joint.
  bool init(ros::NodeHandle& nh, ros::NodeHandle& robot_hw_nh) override;

  // Perform all safety checks that might crash the sensegloves.
  void validate();

  // Reads (in realtime) the state from the sensegloves.
  void read(const std::chrono::steady_clock::time_point& /* time */, const std::chrono::duration<double>& /*elapsed_time*/);
  
  // Writes (in realtime) the commands from the controllers to the sensegloves.
  void write(const std::chrono::steady_clock::time_point& /* time */, const std::chrono::duration<double>& /*elapsed_time*/);
  
private:
  void resetHaptics();
  void initializeInterfaces();
  void initializeJointCommands(size_t glove_index, size_t joint_index, SGHardware::Joint& joint);
  void processJointCommands(size_t glove_index, size_t joint_index, size_t& command_index, SGHardware::Joint& joint);

  void uploadJointNames(ros::NodeHandle& nh) const;  
  void reserveMemory();  
  void updateSenseGloveState();

  // SenseGlove hardware
  std::unique_ptr<SGHardware::SenseGloveSetup> sensegloveSetup;

  // Hardware Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  // Configuration
  size_t num_gloves_ = 0;
  size_t num_joints_ = 0;
  size_t effort_joints_ = 0;
  size_t vibration_joints_ = 0;
  std::string handedness[2] = { "/lh", "/rh" };

  // States
  std::vector<std::vector<double>> jointPosition;
  std::vector<std::vector<double>> jointVelocity;
  std::vector<std::vector<double>> jointEffort;

  // Commands
  std::vector<std::vector<double>> jointPositionCommand;
  std::vector<std::vector<double>> jointVelocityCommand;
  std::vector<std::vector<double>> jointEffortCommand;

  std::vector<std::vector<double>> jointLastPositionCommand;  
  std::vector<std::vector<double>> jointLastEffortCommand;
  std::vector<std::vector<double>> jointLastVibrationCommand;  // inherited from effort_command

  std::vector<float> normalized_values;
  
  // IMU
  Kinematics::Quat imuRotation;

  // Modes
  bool master_shutdown_allowed_command_ = false;
  bool hasActuated = false;

  RtPublisherPtr<senseglove_shared_resources_msgs::SenseGloveState> senseglove_state_pub_;
};
#endif  // ROS_WORKSPACE_SG_HARDWARE_INTERFACE_H
