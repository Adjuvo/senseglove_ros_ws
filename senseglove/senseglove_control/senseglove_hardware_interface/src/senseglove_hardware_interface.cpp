// Copyright (c) 2020 - 2025 SenseGlove
#include "senseglove_hardware_interface/senseglove_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <stdexcept>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

namespace senseglove_hardware_interface
{

SenseGloveHardwareInterface::SenseGloveHardwareInterface() = default;

CallbackReturn SenseGloveHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    // Parse required parameters
    selected_robot_ = info_.hardware_parameters.at("selected_robot");
    AllowedRobot robot_enum(selected_robot_);

    glove_index_ = std::stoi(info_.hardware_parameters.at("glove_index"));

    std::string is_right_str = info_.hardware_parameters.at("is_right");
    std::transform(is_right_str.begin(), is_right_str.end(), is_right_str.begin(), ::tolower);
    is_right_ = (is_right_str == "true") ? true : false;

    publish_rate_ = std::stod(info_.hardware_parameters.at("publish_rate"));

    urdf::Model urdf_model;
    if (!urdf_model.initString(info_.original_xml)) {
        RCLCPP_ERROR(get_logger(), "Failed to parse URDF from original_xml");
        return CallbackReturn::ERROR;
    }

    HardwareBuilder builder(robot_enum, glove_index_, is_right_);
    builder.setUrdfModel(std::move(urdf_model)); 
    senseglove_setup_ = builder.createSenseGloveSetup();

    if (!senseglove_setup_ || senseglove_setup_->size() == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("SenseGloveHW"), "No gloves detected in setup.");
      return CallbackReturn::ERROR;
    }

    if (senseglove_setup_->size() != 1) {
      RCLCPP_ERROR(get_logger(), "Expected 1 glove, got %zu", senseglove_setup_->size());
      return CallbackReturn::ERROR;
    }

    num_gloves_        = senseglove_setup_->size();
    num_joints_        = senseglove_setup_->getSenseGloveRobot(0).getJointSize();
    position_joints_   = senseglove_setup_->getSenseGloveRobot(0).getPositionJointSize();
    effort_joints_     = senseglove_setup_->getSenseGloveRobot(0).getEffortJointSize();
    vibration_joints_  = senseglove_setup_->getSenseGloveRobot(0).getVibrationJointSize();

    initialize_joint_data();
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("SenseGloveHW"), "Initialization failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

void SenseGloveHardwareInterface::initialize_joint_data()
{
  joint_position_.resize(num_gloves_, std::vector<double>(num_joints_, 0.0));
  joint_velocity_.resize(num_gloves_, std::vector<double>(num_joints_, 0.0));
  joint_effort_.resize(num_gloves_, std::vector<double>(num_joints_, 0.0));

  joint_position_command_.resize(num_gloves_, std::vector<double>(position_joints_, 0.0));
  joint_vibration_command_.resize(num_gloves_, std::vector<double>(vibration_joints_, 0.0));
  joint_effort_command_.resize(num_gloves_, std::vector<double>(effort_joints_, 0.0));

  joint_last_position_command_.resize(num_gloves_, std::vector<double>(position_joints_, 0.0));
  joint_last_vibration_command_.resize(num_gloves_, std::vector<double>(vibration_joints_, 0.0));
  joint_last_effort_command_.resize(num_gloves_, std::vector<double>(effort_joints_, 0.0));

  hand_xyz_.resize(num_joints_, std::vector<double>(3, 0.0));
  tip_xyz_.resize(5, std::vector<double>(3, 0.0));
  imu_quat_.resize(4, 0.0);
}

std::vector<StateInterface> SenseGloveHardwareInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (size_t i = 0; i < num_gloves_; ++i) {
    auto & robot = senseglove_setup_->getSenseGloveRobot(i);

    for (size_t j = 0; j < num_joints_; ++j) {
      auto & joint = robot.getJoint(j);
      const auto name = joint.getName();

      state_interfaces.emplace_back(StateInterface(name, hardware_interface::HW_IF_POSITION, &joint_position_[i][j]));
      state_interfaces.emplace_back(StateInterface(name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i][j]));
      state_interfaces.emplace_back(StateInterface(name, hardware_interface::HW_IF_EFFORT, &joint_effort_[i][j]));
    }
  }

  // Positions of all hand joints relative to the Sense Glove origin
  for (size_t h = 0; h < hand_xyz_.size(); ++h) {
    state_interfaces.emplace_back(StateInterface("hand_joint_" + std::to_string(h), "position.x", &hand_xyz_[h][0]));
    state_interfaces.emplace_back(StateInterface("hand_joint_" + std::to_string(h), "position.y", &hand_xyz_[h][1]));
    state_interfaces.emplace_back(StateInterface("hand_joint_" + std::to_string(h), "position.z", &hand_xyz_[h][2]));
  }

   // Finger Tip positions in 3D (world) space.
  for (size_t f = 0; f < tip_xyz_.size(); ++f) {
    state_interfaces.emplace_back(StateInterface("finger_tip_" + std::to_string(f), "position.x", &tip_xyz_[f][0]));
    state_interfaces.emplace_back(StateInterface("finger_tip_" + std::to_string(f), "position.y", &tip_xyz_[f][1]));
    state_interfaces.emplace_back(StateInterface("finger_tip_" + std::to_string(f), "position.z", &tip_xyz_[f][2]));
  }

  // IMU quaternion
  state_interfaces.emplace_back(StateInterface("imu", "orientation.x", &imu_quat_[0]));
  state_interfaces.emplace_back(StateInterface("imu", "orientation.y", &imu_quat_[1]));
  state_interfaces.emplace_back(StateInterface("imu", "orientation.z", &imu_quat_[2]));
  state_interfaces.emplace_back(StateInterface("imu", "orientation.w", &imu_quat_[3]));

  return state_interfaces;
}

std::vector<CommandInterface> SenseGloveHardwareInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_gloves_; ++i) {
    auto & robot = senseglove_setup_->getSenseGloveRobot(i);
    for (size_t j = 0; j < num_joints_; ++j) {
      auto & joint = robot.getJoint(j);
      const std::string name = joint.getName();

      if (joint.getActuationMode() == SGHardware::ActuationMode::position) 
      {
        command_interfaces.emplace_back(CommandInterface(name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i][j]));
      }
      else if (joint.getActuationMode() == SGHardware::ActuationMode::torque ||
               joint.getActuationMode() == SGHardware::ActuationMode::effort)
      {
        command_interfaces.emplace_back(CommandInterface(name, hardware_interface::HW_IF_EFFORT, &joint_effort_command_[i][j]));
      }
    }
  }
  return command_interfaces;
}

// Read Data
return_type SenseGloveHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const auto dt = std::chrono::duration<double>(period.seconds());

  for (size_t i = 0; i < num_gloves_; ++i) {
    auto & robot = senseglove_setup_->getSenseGloveRobot(i);

    if (!robot.updateGloveData(dt)) {
      continue;
    }

    // Joints
    for (size_t j = 0; j < num_joints_; ++j) {
      auto & joint = robot.getJoint(j);
      joint_position_[i][j] = joint.getPosition();
      joint_velocity_[i][j] = joint.getVelocity();
      joint_effort_[i][j]   = joint.getTorque();
    }

    // Per-joint hand positions
    for (size_t k = 0; k < num_joints_; ++k) {
      const auto hp = robot.getHandPosition(static_cast<int>(k));
      hand_xyz_[k][0] = hp.GetX();
      hand_xyz_[k][1] = hp.GetY();
      hand_xyz_[k][2] = hp.GetZ();
    }

    // Fingertip positions
    for (size_t f = 0; f < num_joints_; ++f) {
      const auto tip = robot.getFingerTip(static_cast<int>(f));
      tip_xyz_[f][0] = tip.GetX();
      tip_xyz_[f][1] = tip.GetY();
      tip_xyz_[f][2] = tip.GetZ();
    }

    // IMU quaternion
    SGCore::Kinematics::Quat q;
    if (robot.getImuRotation(q)) {
      imu_quat_[0] = q.GetX();
      imu_quat_[1] = q.GetY();
      imu_quat_[2] = q.GetZ();
      imu_quat_[3] = q.GetW();
    }
  }
  return return_type::OK;
}

// Write Data
return_type SenseGloveHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_gloves_; ++i) {
    auto & robot = senseglove_setup_->getSenseGloveRobot(i);
    size_t idx_force = 0; // brake/squeeze
    size_t idx_vib   = 0;

    for (size_t j = 0; j < num_joints_; ++j) {
      auto & joint = robot.getJoint(j);
      if (joint.canActuate()) {
        process_joint_commands(i, j, idx_force, idx_vib, joint);
      }
    }

    robot.queueEffort(joint_last_effort_command_[i]);
    robot.queueVibrations(joint_last_vibration_command_[i]);
    robot.sendHaptics();
  }
  return return_type::OK;
}

// Process Joint Commands -> Splice joint_effort_command vector into vectors for FFB and vibration commands
void SenseGloveHardwareInterface::process_joint_commands(size_t glove_index, size_t joint_index, size_t & idx_force, size_t & idx_vib, SGHardware::Joint & joint)
{
  if (joint.getActuationMode() == SGHardware::ActuationMode::position) {
    switch (joint.getActuationType().getValue()) {
      case SGHardware::ActuationType::brake:
      case SGHardware::ActuationType::squeeze:
        joint_last_position_command_[glove_index][idx_force++] = joint_position_command_[glove_index][joint_index];
        break;
      case SGHardware::ActuationType::vibration:
        joint_last_vibration_command_[glove_index][idx_vib++] = joint_vibration_command_[glove_index][joint_index];
        break;
    }
  } 
  else if (joint.getActuationMode() == SGHardware::ActuationMode::torque || 
           joint.getActuationMode() == SGHardware::ActuationMode::effort) {
    switch (joint.getActuationType().getValue()) {
      case SGHardware::ActuationType::brake:
      case SGHardware::ActuationType::squeeze:
        joint_last_effort_command_[glove_index][idx_force++] = joint_effort_command_[glove_index][joint_index];
        break;
      case SGHardware::ActuationType::vibration:
        joint_last_vibration_command_[glove_index][idx_vib++] = joint_vibration_command_[glove_index][joint_index];
        break;
    }
  }
}

} // namespace senseglove_hardware_interface

PLUGINLIB_EXPORT_CLASS(senseglove_hardware_interface::SenseGloveHardwareInterface, hardware_interface::SystemInterface)