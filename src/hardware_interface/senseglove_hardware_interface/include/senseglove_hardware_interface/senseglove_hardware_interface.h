// Copyright 2020 senseglove
#ifndef ROS_WORKSPACE_SENSEGLOVE_HARDWARE_INTERFACE_H
#define ROS_WORKSPACE_SENSEGLOVE_HARDWARE_INTERFACE_H

#include <memory>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware/senseglove_setup.h>
#include <senseglove_hardware_builder/hardware_builder.h>
#include <senseglove_shared_resources/SenseGloveState.h>

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
  void read(const ros::Time& /*time*/, const ros::Duration& /*elapsed_time*/) override;

  // Writes (in realtime) the commands from the controllers to the sensegloves.

  void write(const ros::Time& /*time*/, const ros::Duration& /*elapsed_time*/) override;

private:

  std::string handedness[2] = { "/lh", "/rh" };
  
  void uploadJointNames(ros::NodeHandle& nh) const;
  
  void reserveMemory();  

  void updateSenseGloveState();

  /* SenseGlove hardware */
  std::unique_ptr<SGHardware::SenseGloveSetup> sensegloveSetup;

  /* Interfaces */
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  /* Shared memory */
  size_t num_gloves_ = 0;
  size_t num_glove_joints_ = 0;
  size_t num_hand_joints_ = 0;
  int num_effort_joints_ = 0;

  std::vector<std::vector<double>> jointPosition;
  std::vector<std::vector<double>> jointPositionCommand;
  std::vector<std::vector<double>> jointLastPositionCommand;

  std::vector<std::vector<double>> jointVelocity;
  std::vector<std::vector<double>> jointVelocityCommand;

  std::vector<std::vector<double>> jointEffort;
  std::vector<std::vector<double>> jointEffortCommand;
  std::vector<std::vector<double>> jointLastEffortCommand;
  std::vector<std::vector<double>> jointLastVibrationCommand;  // inherited from effort_command

  bool master_shutdown_allowed_command_ = false;

  bool hasActuated = false;

  RtPublisherPtr<senseglove_shared_resources::SenseGloveState> senseglove_state_pub_;
};

#endif  // ROS_WORKSPACE_SG_HARDWARE_INTERFACE_H
