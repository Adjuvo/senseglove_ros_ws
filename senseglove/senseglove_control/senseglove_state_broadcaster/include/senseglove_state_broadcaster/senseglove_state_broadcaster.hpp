#pragma once

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/loaned_state_interface.hpp>
#include <controller_interface/controller_interface.hpp>

#include <senseglove_msgs/msg/sense_glove_state.hpp>

namespace senseglove_state_broadcaster
{

class SenseGloveStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

private:
  rclcpp::Publisher<senseglove_msgs::msg::SenseGloveState>::SharedPtr pub_;

  std::string frame_id_;
  std::string topic_name_;
  std::vector<std::string> joint_names_;
};

}  // namespace senseglove_state_broadcaster