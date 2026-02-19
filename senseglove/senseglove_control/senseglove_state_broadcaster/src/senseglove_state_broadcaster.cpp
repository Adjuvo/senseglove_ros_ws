// Copyright (c) 2020 - 2026 SenseGlove
#include "senseglove_state_broadcaster/senseglove_state_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

namespace senseglove_state_broadcaster
{

// Setup
controller_interface::CallbackReturn SenseGloveStateBroadcaster::on_init()
{
  auto node = get_node();
  topic_name_ = node->declare_parameter<std::string>("topic_name", "senseglove_states");
  frame_id_ = node->declare_parameter<std::string>("frame_id", "world");

  node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  node->get_parameter("joints", joint_names_);

  pub_ = node->create_publisher<senseglove_msgs::msg::SenseGloveState>(
    topic_name_, rclcpp::QoS(10).best_effort());

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration SenseGloveStateBroadcaster::command_interface_configuration() const
{
  return {interface_configuration_type::NONE, {}};
}

InterfaceConfiguration SenseGloveStateBroadcaster::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;

  // Joints: position + velocity for each joint
  for (const auto& jn : joint_names_)
  {
    cfg.names.push_back(jn + "/position");
    cfg.names.push_back(jn + "/velocity");
  }

  // Hand Joint Position
  for (int i = 0; i < 20; ++i)
  {
    const std::string base = "hand_joint_" + std::to_string(i) + "/position.";
    cfg.names.push_back(base + "x");
    cfg.names.push_back(base + "y");
    cfg.names.push_back(base + "z");
  }

  // Finger-tip Position
  for (int i = 0; i < 5; ++i)
  {
    const std::string base = "finger_tip_" + std::to_string(i) + "/position.";
    cfg.names.push_back(base + "x");
    cfg.names.push_back(base + "y");
    cfg.names.push_back(base + "z");
  }

  // IMU quaternion
  cfg.names.push_back("imu/orientation.x");
  cfg.names.push_back("imu/orientation.y");
  cfg.names.push_back("imu/orientation.z");
  cfg.names.push_back("imu/orientation.w");

  return cfg;
}

// Read double from a state interface
static inline double read_value(hardware_interface::LoanedStateInterface& sg_interface,
                                rclcpp::Logger logger,
                                rclcpp::Clock::SharedPtr clock,
                                const char* logger_name)
{
  auto opt = sg_interface.get_optional();
  if (opt.has_value())
    return *opt;

  RCLCPP_WARN_THROTTLE(
    logger, *clock, 2'000, "State interface '%s' has no value; using 0.0", logger_name);
  return 0.0;
}

// Update
controller_interface::return_type SenseGloveStateBroadcaster::update(const rclcpp::Time& time,
                                                                     const rclcpp::Duration&)
{
  if (!pub_)
    return controller_interface::return_type::OK;

  const size_t joint_size = joint_names_.size();
  const size_t expected = joint_size * 2 +  // pos+vel per joint
                          20 * 3 +          // handposition xyz
                          5 * 3 +           // fingertips xyz
                          4;                // imu quaternion

  if (state_interfaces_.size() < expected)
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                         *get_node()->get_clock(),
                         2'000,
                         "State interfaces provided (%zu) < expected (%zu). Skipping publish.",
                         state_interfaces_.size(),
                         expected);
    return controller_interface::return_type::OK;
  }

  senseglove_msgs::msg::SenseGloveState msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.joint_names = joint_names_;
  msg.position.resize(joint_size);
  msg.absolute_velocity.resize(joint_size);

  auto logger = get_node()->get_logger();
  auto clock = get_node()->get_clock();

  size_t index = 0;

  // Joints
  for (size_t i = 0; i < joint_size; ++i)
  {
    msg.position[i] = read_value(
      state_interfaces_[index++], logger, clock, (joint_names_[i] + "/position").c_str());
    msg.absolute_velocity[i] = read_value(
      state_interfaces_[index++], logger, clock, (joint_names_[i] + "/velocity").c_str());
  }

  // Hand joints
  msg.hand_position.resize(20);
  for (int i = 0; i < 20; ++i)
  {
    auto& hp = msg.hand_position[i];
    hp.x = read_value(state_interfaces_[index++],
                      logger,
                      clock,
                      ("hand_joint_" + std::to_string(i) + "/position.x").c_str());
    hp.y = read_value(state_interfaces_[index++],
                      logger,
                      clock,
                      ("hand_joint_" + std::to_string(i) + "/position.y").c_str());
    hp.z = read_value(state_interfaces_[index++],
                      logger,
                      clock,
                      ("hand_joint_" + std::to_string(i) + "/position.z").c_str());
  }

  // Fingertips
  msg.finger_tip_position.resize(5);
  for (int i = 0; i < 5; ++i)
  {
    auto& p = msg.finger_tip_position[i];
    p.x = read_value(state_interfaces_[index++],
                     logger,
                     clock,
                     ("finger_tip_" + std::to_string(i) + "/position.x").c_str());
    p.y = read_value(state_interfaces_[index++],
                     logger,
                     clock,
                     ("finger_tip_" + std::to_string(i) + "/position.y").c_str());
    p.z = read_value(state_interfaces_[index++],
                     logger,
                     clock,
                     ("finger_tip_" + std::to_string(i) + "/position.z").c_str());
  }

  // IMU quaternion
  msg.imu_orientation.x =
    read_value(state_interfaces_[index++], logger, clock, "imu/orientation.x");
  msg.imu_orientation.y =
    read_value(state_interfaces_[index++], logger, clock, "imu/orientation.y");
  msg.imu_orientation.z =
    read_value(state_interfaces_[index++], logger, clock, "imu/orientation.z");
  msg.imu_orientation.w =
    read_value(state_interfaces_[index++], logger, clock, "imu/orientation.w");

  pub_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace senseglove_state_broadcaster

PLUGINLIB_EXPORT_CLASS(senseglove_state_broadcaster::SenseGloveStateBroadcaster,
                       controller_interface::ControllerInterface)