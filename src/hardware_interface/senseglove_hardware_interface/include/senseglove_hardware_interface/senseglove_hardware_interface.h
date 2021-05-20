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

/**
 * @brief HardwareInterface to allow ros_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via ros_control.
 */
class SenseGloveHardwareInterface : public hardware_interface::RobotHW
{
public:
  SenseGloveHardwareInterface(std::unique_ptr<senseglove::SenseGloveSetup> setup);

  /**
   * @brief Initialize the HardwareInterface by registering position interfaces
   * for each joint.
   */
  bool init(ros::NodeHandle& nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * @brief Perform all safety checks that might crash the sensegloves.
   */
  void validate();

  /**
   * Reads (in realtime) the state from the sensegloves.
   *
   * @param time Current time
   * @param elapsed_time Duration since last write action
   */
  void read(const ros::Time& /*time*/, const ros::Duration& /*elapsed_time*/) override;

  /**
   * Writes (in realtime) the commands from the controllers to the sensegloves.
   *
   * @param time Current time
   * @param elapsed_time Duration since last write action
   */
  void write(const ros::Time& /*time*/, const ros::Duration& /*elapsed_time*/) override;

private:
  void uploadJointNames(ros::NodeHandle& nh) const;
  /**
   * Uses the num_joints_ member to resize all vectors
   * in order to avoid allocation at runtime.
   */
  void reserveMemory();

  void updateSenseGloveState();

  /* SenseGlove hardware */
  std::unique_ptr<senseglove::SenseGloveSetup> senseglove_setup_;

  /* Interfaces */
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  /* Shared memory */
  size_t num_gloves_ = 0;
  size_t num_joints_ = 0;

  std::vector<std::vector<double>> joint_position_;
  std::vector<std::vector<double>> joint_position_command_;
  std::vector<std::vector<double>> joint_last_position_command_;

  std::vector<std::vector<double>> joint_velocity_;
  std::vector<std::vector<double>> joint_velocity_command_;

  std::vector<std::vector<double>> joint_effort_;
  std::vector<std::vector<double>> joint_effort_command_;
  std::vector<std::vector<double>> joint_last_effort_command_;
  std::vector<std::vector<double>> joint_last_buzz_command_;  // inherited from effort_command

  bool master_shutdown_allowed_command_ = false;

  bool has_actuated_ = false;

  RtPublisherPtr<senseglove_shared_resources::SenseGloveState> senseglove_state_pub_;
};

#endif  // ROS_WORKSPACE_SG_HARDWARE_INTERFACE_H
