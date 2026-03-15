// Copyright (c) 2020 - 2026 SenseGlove

#include "senseglove_hardware_interface/senseglove_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <stdexcept>

using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

namespace senseglove_hardware_interface
{

void GloveData::initialize(size_t numJoints, size_t effortJoints, size_t vibrationJoints)
{
  num_joints = numJoints;
  effort_joints = effortJoints;
  vibration_joints = vibrationJoints;

  joint_position.resize(num_joints, 0.0);
  joint_velocity.resize(num_joints, 0.0);
  joint_effort.resize(num_joints, 0.0);

  size_t actuatable = effort_joints + vibration_joints;
  joint_effort_command.resize(actuatable, 0.0);
  effort_output.resize(effort_joints, 0.0);
  vibration_output.resize(vibration_joints, 0.0);

  hand_xyz.resize(20, std::vector<double>(3, 0.0));
  tip_xyz.resize(5, std::vector<double>(3, 0.0));
  imu_quat.resize(4, 0.0);
  imu_quat[3] = 1.0;

  joint_to_command_index.resize(num_joints, SIZE_MAX);
  joint_actuator_type.resize(num_joints, ActuatorType::None);
}

void GloveData::buildIndexMap(SGHardware::SenseGloveRobot& robot)
{
  size_t cmd_idx = 0;
  for (size_t j = 0; j < num_joints; ++j)
  {
    auto& joint = robot.getJoint(j);
    if (joint.canActuate())
    {
      joint_to_command_index[j] = cmd_idx++;
      auto type = joint.getActuationType();
      if (type == SGHardware::ActuationType::brake || type == SGHardware::ActuationType::squeeze)
        joint_actuator_type[j] = ActuatorType::ForceFeedback;
      else if (type == SGHardware::ActuationType::vibration)
        joint_actuator_type[j] = ActuatorType::Vibration;
    }
  }
}

SenseGloveHardwareInterface::SenseGloveHardwareInterface() = default;

SenseGloveHardwareInterface::SenseGloveHardwareInterface(
  std::unique_ptr<SGHardware::SenseGloveRobot> robot)
  : robot_(std::move(robot))
{
}

CallbackReturn SenseGloveHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  auto& logger = logger_;

  try
  {
    auto it = info_.hardware_parameters.find("publish_rate");
    if (it != info_.hardware_parameters.end())
      publish_rate_ = std::stod(it->second);

    if (!robot_)
    {
      // Parse URDF
      auto urdf_model = std::make_shared<urdf::Model>();
      if (!urdf_model->initString(info_.original_xml))
      {
        RCLCPP_ERROR(logger, "Failed to parse URDF");
        return CallbackReturn::ERROR;
      }

      // Get parameters
      std::string selected_robot = info_.hardware_parameters.at("selected_robot");
      std::string glove_serial = info_.hardware_parameters.at("glove_serial");
      std::string is_right_str = info_.hardware_parameters.at("is_right");

      std::transform(is_right_str.begin(), is_right_str.end(), is_right_str.begin(), ::tolower);
      bool is_right = (is_right_str == "true");

      logger_ = rclcpp::get_logger("senseglove.glove0" + glove_serial + "." +
                                   (is_right ? "rh" : "lh") + ".hardware_interface");
      auto& logger = logger_;
      RCLCPP_INFO(logger,
                  "%sInitializing SenseGlove: %s (serial: %s, %s)%s",
                  color::INFO,
                  selected_robot.c_str(),
                  glove_serial.c_str(),
                  is_right ? "right" : "left",
                  color::RESET);

      // Build robot
      senseglove_hardware_builder::AllowedRobot robot_enum(selected_robot);
      senseglove_hardware_builder::HardwareBuilder builder(robot_enum, glove_serial, is_right);
      builder.setUrdfModel(urdf_model);

      robot_ = builder.createRobot();
    }

    if (!robot_)
    {
      RCLCPP_ERROR(logger, "Failed to create SenseGlove robot");
      return CallbackReturn::ERROR;
    }

    // Initialize glove data
    initialize_glove_data();

    // Subscriber: Nova 2 custom waveform commands
    if (robot_->getGloveType() == SGHardware::SenseGloveRobot::GloveType::Nova2)
    {
      const std::string waveform_topic =
        std::string(get_node()->get_namespace()) + "/vibration_waveform";
      waveform_sub_ = get_node()->create_subscription<senseglove_msgs::msg::Nova2WaveformCommand>(
        waveform_topic,
        rclcpp::QoS(10),
        [this](const senseglove_msgs::msg::Nova2WaveformCommand::SharedPtr msg) {
          waveformCallback(msg);
        });
      RCLCPP_INFO(logger, "Subscribed to Nova 2 waveform commands on: %s", waveform_topic.c_str());
    }

    RCLCPP_DEBUG(logger,
                 "SenseGlove initialized: %s with %zu joints [ffb:%zu, vib:%zu]",
                 robot_->getRobotName().c_str(),
                 glove_data_.num_joints,
                 glove_data_.effort_joints,
                 glove_data_.vibration_joints);

    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Initialization failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

void SenseGloveHardwareInterface::initialize_glove_data()
{
  glove_data_.initialize(
    robot_->getJointSize(), robot_->getEffortJointSize(), robot_->getVibrationJointSize());
  glove_data_.buildIndexMap(*robot_);
}

std::vector<StateInterface> SenseGloveHardwareInterface::export_state_interfaces()
{
  auto& logger = logger_;
  std::vector<StateInterface> interfaces;

  // Joint state interfaces
  for (size_t j = 0; j < glove_data_.num_joints; ++j)
  {
    const std::string& name = robot_->getJoint(j).getName();
    interfaces.emplace_back(
      name, hardware_interface::HW_IF_POSITION, &glove_data_.joint_position[j]);
    interfaces.emplace_back(
      name, hardware_interface::HW_IF_VELOCITY, &glove_data_.joint_velocity[j]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &glove_data_.joint_effort[j]);
  }

  // Position interfaces of all hand joints relative to the Sense Glove origin
  for (size_t h = 0; h < glove_data_.hand_xyz.size(); ++h)
  {
    std::string n = "hand_joint_" + std::to_string(h);
    interfaces.emplace_back(n, "position.x", &glove_data_.hand_xyz[h][0]);
    interfaces.emplace_back(n, "position.y", &glove_data_.hand_xyz[h][1]);
    interfaces.emplace_back(n, "position.z", &glove_data_.hand_xyz[h][2]);
  }

  // Fingertip position interfaces (5 fingers x 3 coordinates)
  for (size_t f = 0; f < glove_data_.tip_xyz.size(); ++f)
  {
    std::string n = "finger_tip_" + std::to_string(f);
    interfaces.emplace_back(n, "position.x", &glove_data_.tip_xyz[f][0]);
    interfaces.emplace_back(n, "position.y", &glove_data_.tip_xyz[f][1]);
    interfaces.emplace_back(n, "position.z", &glove_data_.tip_xyz[f][2]);
  }

  // IMU orientation interface
  interfaces.emplace_back("imu", "orientation.x", &glove_data_.imu_quat[0]);
  interfaces.emplace_back("imu", "orientation.y", &glove_data_.imu_quat[1]);
  interfaces.emplace_back("imu", "orientation.z", &glove_data_.imu_quat[2]);
  interfaces.emplace_back("imu", "orientation.w", &glove_data_.imu_quat[3]);

  RCLCPP_DEBUG(logger, "Exported %zu state interfaces", interfaces.size());
  return interfaces;
}

std::vector<CommandInterface> SenseGloveHardwareInterface::export_command_interfaces()
{
  auto& logger = logger_;
  std::vector<CommandInterface> interfaces;

  size_t cmd_idx = 0;
  for (size_t j = 0; j < glove_data_.num_joints; ++j)
  {
    auto& joint = robot_->getJoint(j);
    if (joint.canActuate() && cmd_idx < glove_data_.joint_effort_command.size())
    {
      interfaces.emplace_back(joint.getName(),
                              hardware_interface::HW_IF_EFFORT,
                              &glove_data_.joint_effort_command[cmd_idx++]);
    }
  }

  RCLCPP_DEBUG(logger, "Exported %zu command interfaces", interfaces.size());
  return interfaces;
}

return_type SenseGloveHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
  const auto dt = std::chrono::duration<double>(period.seconds());

  if (!robot_->updateGloveData(dt))
    return return_type::OK;

  // Update joint states
  for (size_t j = 0; j < glove_data_.num_joints; ++j)
  {
    auto& joint = robot_->getJoint(j);
    glove_data_.joint_position[j] = joint.getPosition();
    glove_data_.joint_velocity[j] = joint.getVelocity();
    glove_data_.joint_effort[j] = joint.getTorque();
  }

  // Update hand positions
  for (size_t h = 0; h < glove_data_.hand_xyz.size(); ++h)
  {
    const auto pos = robot_->getHandPosition(static_cast<int>(h));
    glove_data_.hand_xyz[h][0] = pos.GetX();
    glove_data_.hand_xyz[h][1] = pos.GetY();
    glove_data_.hand_xyz[h][2] = pos.GetZ();
  }

  // Update fingertip positions
  for (size_t f = 0; f < glove_data_.tip_xyz.size(); ++f)
  {
    const auto tip = robot_->getFingerTip(static_cast<int>(f));
    glove_data_.tip_xyz[f][0] = tip.GetX();
    glove_data_.tip_xyz[f][1] = tip.GetY();
    glove_data_.tip_xyz[f][2] = tip.GetZ();
  }

  // Update IMU
  SGCore::Kinematics::Quat q;
  if (robot_->getImuRotation(q))
  {
    glove_data_.imu_quat[0] = q.GetX();
    glove_data_.imu_quat[1] = q.GetY();
    glove_data_.imu_quat[2] = q.GetZ();
    glove_data_.imu_quat[3] = q.GetW();
  }

  return return_type::OK;
}

void SenseGloveHardwareInterface::waveformCallback(
  const senseglove_msgs::msg::Nova2WaveformCommand::SharedPtr msg)
{
  using Msg = senseglove_msgs::msg::Nova2WaveformCommand;
  using Motor = SGCore::Nova::ENova2VibroMotor;

  // Map message motor_location to API enum
  Motor motor;
  switch (msg->motor_location)
  {
    case Msg::MOTOR_THUMB_TIP:
      motor = Motor::ThumbFingertip;
      break;
    case Msg::MOTOR_INDEX_TIP:
      motor = Motor::IndexFingertip;
      break;
    case Msg::MOTOR_PALM_INDEX:
      motor = Motor::PalmIndexSide;
      break;
    case Msg::MOTOR_PALM_PINKY:
      motor = Motor::PalmPinkySide;
      break;
    default:
      RCLCPP_WARN(
        logger_, "Unknown motor_location %u: ignoring waveform command", msg->motor_location);
      return;
  }

  // Build CustomWaveform
  SGCore::CustomWaveform waveform(msg->amplitude, msg->sustain_time, msg->frequency_start);
  waveform.SetFrequencyEnd(msg->frequency_end);
  waveform.SetAttackTime(msg->attack_time);
  waveform.SetSustainTime(msg->sustain_time);
  waveform.SetDecayTime(msg->decay_time);
  waveform.SetRepeatAmount(msg->repeat_amount);
  waveform.SetInfinite(msg->infinite);

  if (msg->frequency_switch_time > 0.0f)
  {
    waveform.SetFrequencySwitchTime(msg->frequency_switch_time);
    waveform.SetFrequencySwitchFactor(msg->frequency_switch_factor);
  }

  switch (msg->wave_type)
  {
    case Msg::WAVE_SINE:
      waveform.SetWaveType(SGCore::EWaveformType::Sine);
      break;
    case Msg::WAVE_SQUARE:
      waveform.SetWaveType(SGCore::EWaveformType::Square);
      break;
    case Msg::WAVE_SAW_UP:
      waveform.SetWaveType(SGCore::EWaveformType::SawUp);
      break;
    case Msg::WAVE_SAW_DOWN:
      waveform.SetWaveType(SGCore::EWaveformType::SawDown);
      break;
    case Msg::WAVE_TRIANGLE:
      waveform.SetWaveType(SGCore::EWaveformType::Triangle);
      break;
    case Msg::WAVE_NOISE:
      waveform.SetWaveType(SGCore::EWaveformType::Noise);
      break;
    default:
      waveform.SetWaveType(SGCore::EWaveformType::Sine);
      break;
  }

  // Queue custom waveform for the robot
  robot_->queueCustomWaveform(waveform, motor);
}

return_type SenseGloveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::fill(glove_data_.effort_output.begin(), glove_data_.effort_output.end(), 0.0);
  std::fill(glove_data_.vibration_output.begin(), glove_data_.vibration_output.end(), 0.0);

  size_t ffb_idx = 0, vib_idx = 0;

  for (size_t j = 0; j < glove_data_.num_joints; ++j)
  {
    size_t cmd_idx = glove_data_.joint_to_command_index[j];
    if (cmd_idx == SIZE_MAX)
      continue;

    double val = glove_data_.joint_effort_command[cmd_idx];

    if (glove_data_.joint_actuator_type[j] == ActuatorType::ForceFeedback &&
        ffb_idx < glove_data_.effort_joints)
      glove_data_.effort_output[ffb_idx++] = val;
    else if (glove_data_.joint_actuator_type[j] == ActuatorType::Vibration &&
             vib_idx < glove_data_.vibration_joints)
      glove_data_.vibration_output[vib_idx++] = val;
  }

  // FFB for all gloves
  // Vibration levels for Nova 1 / DK1.
  // For Nova 2, vibration_output is empty; use custom waveforms instead
  robot_->queueEffort(glove_data_.effort_output);
  robot_->queueVibrations(glove_data_.vibration_output);
  robot_->sendHaptics();

  robot_->processCustomWaveform();

  return return_type::OK;
}

}  // namespace senseglove_hardware_interface

PLUGINLIB_EXPORT_CLASS(senseglove_hardware_interface::SenseGloveHardwareInterface,
                       hardware_interface::SystemInterface)
