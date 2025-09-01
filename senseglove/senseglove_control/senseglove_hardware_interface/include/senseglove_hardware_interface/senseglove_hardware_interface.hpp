#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_map>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ROS 2 Control
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

// Senseglove
#include <senseglove_hardware/senseglove_robot.hpp>
#include <senseglove_hardware/senseglove_setup.hpp>
#include <senseglove_hardware_builder/hardware_builder.hpp>
#include <senseglove_msgs/msg/sense_glove_state.hpp>

namespace senseglove_hardware_interface
{
  
class SenseGloveHardwareInterface : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(SenseGloveHardwareInterface)

  SenseGloveHardwareInterface();
  ~SenseGloveHardwareInterface() override = default;
  
  SenseGloveHardwareInterface(std::unique_ptr<SGHardware::SenseGloveSetup> setup);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  // Configuration
  std::unique_ptr<SGHardware::SenseGloveSetup> senseglove_setup_;
  size_t num_gloves_ = 0;
  size_t num_joints_ = 0;
  size_t position_joints_ = 0;
  size_t effort_joints_ = 0;
  size_t vibration_joints_ = 0;

  // Parameters
  std::string selected_robot_;
  int glove_index_ = 0;
  bool is_right_ = true;
  double publish_rate_ = 100.0;

  // States
  std::vector<std::vector<double>> joint_position_;
  std::vector<std::vector<double>> joint_velocity_;
  std::vector<std::vector<double>> joint_effort_;

  // Commands
  std::vector<std::vector<double>> joint_position_command_;
  std::vector<std::vector<double>> joint_vibration_command_;
  std::vector<std::vector<double>> joint_effort_command_;
  std::vector<std::vector<double>> joint_last_position_command_;
  std::vector<std::vector<double>> joint_last_vibration_command_;
  std::vector<std::vector<double>> joint_last_effort_command_;

  // Message publisher
  rclcpp::Publisher<senseglove_msgs::msg::SenseGloveState>::SharedPtr state_publisher_;
  senseglove_msgs::msg::SenseGloveState latest_msg_;

  void initialize_joint_data();
  void initialize_joint_commands(size_t glove_index, size_t joint_index, SGHardware::Joint& joint);
  void process_joint_commands(size_t glove_index, size_t joint_index, size_t & idx_force, size_t & idx_vib, SGHardware::Joint& joint);
  void update_senseglove_state();

  std::string get_topic_name() const;
};

}  // namespace senseglove_hardware_interface