#pragma once

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"

#include <senseglove_msgs/msg/nova2_waveform_command.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <senseglove_hardware/senseglove_robot.hpp>
#include <senseglove_hardware_builder/hardware_builder.hpp>

namespace senseglove_hardware_interface
{

enum class ActuatorType
{
  None,
  ForceFeedback,
  Vibration
};

// Single Glove state and command interfaces
struct GloveData
{
  size_t num_joints = 0;
  size_t effort_joints = 0;
  size_t vibration_joints = 0;

  // State data
  std::vector<double> joint_position;
  std::vector<double> joint_velocity;
  std::vector<double> joint_effort;

  // Hand pose data
  std::vector<std::vector<double>> hand_xyz;
  std::vector<std::vector<double>> tip_xyz;
  std::vector<double> imu_quat;

  // Command data
  std::vector<double> joint_effort_command;
  std::vector<double> effort_output;
  std::vector<double> vibration_output;

  // Mapping from joint index to command index
  std::vector<size_t> joint_to_command_index;
  std::vector<ActuatorType> joint_actuator_type;

  void initialize(size_t numJoints, size_t effortJoints, size_t vibrationJoints);
  void buildIndexMap(SGHardware::SenseGloveRobot& robot);
};

// ROS2 Control HW for a single SenseGlove
class SenseGloveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SenseGloveHardwareInterface)

  SenseGloveHardwareInterface();
  ~SenseGloveHardwareInterface() override = default;
  explicit SenseGloveHardwareInterface(std::unique_ptr<SGHardware::SenseGloveRobot> robot);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  std::unique_ptr<SGHardware::SenseGloveRobot> robot_;
  GloveData glove_data_;
  double publish_rate_ = 60.0;

  // Nova 2 custom waveform subscriber
  rclcpp::Subscription<senseglove_msgs::msg::Nova2WaveformCommand>::SharedPtr waveform_sub_;

  void initialize_glove_data();
  void waveformCallback(const senseglove_msgs::msg::Nova2WaveformCommand::SharedPtr msg);
  rclcpp::Logger logger_ = rclcpp::get_logger("senseglove.hardware_interface");
};

}  // namespace senseglove_hardware_interface

namespace color
{
constexpr const char* RESET = "\033[0m";
constexpr const char* INFO = "\033[2;36m";
}  // namespace color