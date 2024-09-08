// Copyright (c) 2020 - 2024 SenseGlove
#include "senseglove_hardware_interface/senseglove_hardware_interface.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <string>

#include <urdf/model.h>

using hardware_interface::JointHandle;
using hardware_interface::JointStateHandle;
using hardware_interface::PositionJointInterface;

SenseGloveHardwareInterface::SenseGloveHardwareInterface(std::unique_ptr<SGHardware::SenseGloveSetup> setup)
  : sensegloveSetup(std::move(setup)), num_gloves_(this->sensegloveSetup ? this->sensegloveSetup->size() : 0)
{
  if (!this->sensegloveSetup)
  {
    throw std::runtime_error("SenseGloveSetup is null");
  }
}

// Initialization
bool SenseGloveHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle& /* robot_hw_nh */)
{
  ROS_INFO_STREAM("Senseglove HW Interface: Initializing realtime publisher for the SenseGlove states");

  std::string topicName = "/" + this->sensegloveSetup->getSenseGloveRobot(0).getRobotName() +
                          handedness[this->sensegloveSetup->getSenseGloveRobot(0).getRight()] + "/senseglove_states/";

  this->senseglove_state_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<senseglove_shared_resources::SenseGloveState>>(
          nh, topicName, 1);

  ROS_INFO_STREAM("Senseglove HW Interface: Constructed topic: " << topicName);  

  this->uploadJointNames(nh);

  num_joints_ = this->sensegloveSetup->getSenseGloveRobot(0).getJointSize();
  effort_joints_ = this->sensegloveSetup->getSenseGloveRobot(0).getEffortJointSize();
  vibration_joints_ = this->sensegloveSetup->getSenseGloveRobot(0).getVibrationJointSize();

  this->reserveMemory();

  // Start ethercat cycle in the hardware
  this->sensegloveSetup->startCommunication(true);

  this->initializeInterfaces();

  ROS_INFO_STREAM("Senseglove HW Interface: Successfully actuated all joints");

  this->registerInterface(&this->joint_state_interface_);
  this->registerInterface(&this->position_joint_interface_);
  this->registerInterface(&this->effort_joint_interface_);
    
  return true;
}

// Initialize Interfaces
void SenseGloveHardwareInterface::initializeInterfaces()
{
  for (size_t i = 0; i < num_gloves_; ++i)
  {
    // Initialize interfaces for each joint
    for (size_t k = 0; k < num_joints_; ++k)
    {
      SGHardware::Joint& joint = this->sensegloveSetup->getSenseGloveRobot(i).getJoint(k);
      ROS_DEBUG_STREAM("Joint State Interface: Obtained necessary joint");

      // Create joint state interface
      JointStateHandle joint_state_handle(joint.getName(), &jointPosition[i][k], &jointVelocity[i][k], &jointEffort[i][k]);
      ROS_DEBUG_STREAM("Joint State Interface: Handle created");

      joint_state_interface_.registerHandle(joint_state_handle);
      ROS_DEBUG_STREAM("Joint State Interface: Joint State Handle Registered");

      if (joint.getActuationMode() == SGHardware::ActuationMode::position)
      {
        JointHandle joint_position_handle(joint_state_handle, &jointPositionCommand[i][k]);
        position_joint_interface_.registerHandle(joint_position_handle);
        ROS_DEBUG_STREAM("Joint State Interface: Position Joint Interface Handle Registered");
      }
      else if (joint.getActuationMode() == SGHardware::ActuationMode::torque)
      {
        JointHandle joint_effort_handle_(joint_state_handle, &jointEffortCommand[i][k]);
        effort_joint_interface_.registerHandle(joint_effort_handle_);
        ROS_DEBUG_STREAM("Joint State Interface: Effort Joint Interface Handle Registered");
      }

      // Create velocity joint interface
      JointHandle joint_velocity_handle(joint_state_handle, &jointVelocityCommand[i][k]);
      velocity_joint_interface_.registerHandle(joint_velocity_handle);
      ROS_DEBUG_STREAM("Joint State Interface: Velocity Joint Interface Handle Registered");

      // Prepare Joints for Actuation
      if (joint.canActuate())
      {
        this->initializeJointCommands(i, k, joint);
      }
    }
  }
}

// Initialize Joint Commands
void SenseGloveHardwareInterface::initializeJointCommands(size_t glove_index, size_t joint_index, SGHardware::Joint& joint)
{
  if (sensegloveSetup->getSenseGloveRobot(glove_index).updateGloveData(ros::Duration(0.0)))
  {
    jointPosition[glove_index][joint_index] = joint.getPosition();
    jointVelocity[glove_index][joint_index] = joint.getVelocity();
    jointEffort[glove_index][joint_index] = 0.0;
  }

  if (joint.getActuationMode() == SGHardware::ActuationMode::position)
  {
    jointPositionCommand[glove_index][joint_index] = 0.0;
  }
  else if (joint.getActuationMode() == SGHardware::ActuationMode::torque)
  {
    jointEffortCommand[glove_index][joint_index] = 0.0;
  }
}

// Read Data
void SenseGloveHardwareInterface::read(const ros::Time& /* time */, const ros::Duration& elapsed_time)
{
  for (size_t i = 0; i < num_gloves_; ++i)
  {
    if (sensegloveSetup->getSenseGloveRobot(i).updateGloveData(elapsed_time))
    {
      auto& robot = sensegloveSetup->getSenseGloveRobot(i);
      for (size_t j = 0; j < num_joints_; ++j)
      {
      {
        auto& joint = robot.getJoint(j);
        jointPosition[i][j] = joint.getPosition();
        jointVelocity[i][j] = joint.getVelocity();
        jointEffort[i][j] = joint.getTorque();
      }
      }
      this->updateSenseGloveState();
    }
  }
}

// Write Data
void SenseGloveHardwareInterface::write(const ros::Time& /* time */, const ros::Duration& /* elapsed_time */)
{
  // Accumulate data and do not send yet
  for (size_t i = 0; i < num_gloves_; ++i)
  {
    size_t k = 0;
    SGHardware::SenseGloveRobot& robot = sensegloveSetup->getSenseGloveRobot(i);
    for (size_t j = 0; j < num_joints_; ++j)
    {
      SGHardware::Joint& joint = robot.getJoint(j);
      if (joint.canActuate())
      {
        this->processJointCommands(i, j, k, joint);
      }
    }
    robot.queueEffort(jointLastPositionCommand[i]);
    robot.queueVibrations(jointLastVibrationCommand[i]);
    robot.sendHaptics();
  }
}

// Process Joint Commands -> Splice joint_effort_command vector into vectors for FFB and vibration commands
void SenseGloveHardwareInterface::processJointCommands(size_t glove_index, size_t joint_index, size_t& command_index, SGHardware::Joint& joint)
{
  if (joint.getActuationMode() == SGHardware::ActuationMode::position)
  {
    switch(joint.getActuationType().getValue())
    {
      case SGHardware::ActuationType::brake:
        jointLastPositionCommand[glove_index][command_index] = jointPositionCommand[glove_index][joint_index];
        ++command_index;
        break;

      case SGHardware::ActuationType::vibration:
        jointLastVibrationCommand[glove_index][command_index] = jointPositionCommand[glove_index][joint_index];
        break; 
      
      case SGHardware::ActuationType::squeeze:
        jointLastPositionCommand[glove_index][command_index] = jointPositionCommand[glove_index][joint_index];
        ++command_index;
        break;
    }
  }
  else if (joint.getActuationMode() == SGHardware::ActuationMode::torque)
  {
    switch(joint.getActuationType().getValue())
    {
      case SGHardware::ActuationType::brake:
        jointLastEffortCommand[glove_index][command_index] = jointEffortCommand[glove_index][joint_index];
        ++command_index;
        break;

      case SGHardware::ActuationType::vibration:
        jointLastVibrationCommand[glove_index][command_index] = jointPositionCommand[glove_index][joint_index];
        break; 
      
      case SGHardware::ActuationType::squeeze:
        jointLastEffortCommand[glove_index][command_index] = jointEffortCommand[glove_index][joint_index];
        ++command_index;
        break;
    }
  }
}

// Upload Joint Names
void SenseGloveHardwareInterface::uploadJointNames(ros::NodeHandle& nh) const
{
  std::vector<std::string> joint_names;
  for (const auto& joint : *this->sensegloveSetup)
  {
    joint_names.push_back(joint.getRobotName());
  }
  std::sort(joint_names.begin(), joint_names.end());
  nh.setParam(this->sensegloveSetup->getSenseGloveRobot(0).getRobotName() + handedness[this->sensegloveSetup->getSenseGloveRobot(0).getRight()] + "/joint_names", joint_names);
}

void SenseGloveHardwareInterface::reserveMemory()
{
  jointPosition.resize(num_gloves_);
  jointPositionCommand.resize(num_gloves_);
  jointLastPositionCommand.resize(num_gloves_);
  jointVelocity.resize(num_gloves_);
  jointVelocityCommand.resize(num_gloves_);
  jointEffort.resize(num_gloves_);
  jointEffortCommand.resize(num_gloves_);
  jointLastEffortCommand.resize(num_gloves_);
  jointLastVibrationCommand.resize(num_gloves_);

  
  for (unsigned int i = 0; i < num_gloves_; ++i)
  {
    jointPosition[i].resize(num_joints_, 0.0);
    jointPositionCommand[i].resize(num_joints_, 0.0);
    jointVelocity[i].resize(num_joints_, 0.0);
    jointVelocityCommand[i].resize(num_joints_, 0.0);
    jointEffort[i].resize(num_joints_, 0.0);
    jointEffortCommand[i].resize(num_joints_, 0.0);
    
    jointLastPositionCommand[i].resize(effort_joints_, 0.0);
    jointLastVibrationCommand[i].resize(vibration_joints_, 0.0);
    jointLastEffortCommand[i].resize(effort_joints_, 0.0);
  }

  senseglove_state_pub_->msg_.joint_names.resize(num_gloves_ * num_joints_);
  senseglove_state_pub_->msg_.position.resize(num_gloves_ * num_joints_);
  senseglove_state_pub_->msg_.absolute_velocity.resize(num_gloves_ * num_joints_);
  senseglove_state_pub_->msg_.hand_position.resize(num_gloves_ * num_joints_);
  senseglove_state_pub_->msg_.finger_tip_positions.resize(5);
}

void SenseGloveHardwareInterface::updateSenseGloveState()
{
  if (!senseglove_state_pub_->trylock())
  {
    return;
  }

  senseglove_state_pub_->msg_.header.stamp = ros::Time::now();
  for (size_t i = 0; i < num_gloves_; ++i)
  {
    SGHardware::SenseGloveRobot& robot = sensegloveSetup->getSenseGloveRobot(i);
    for (size_t k = 0; k < num_joints_; ++k)
    {
      SGHardware::Joint& joint = robot.getJoint(k);
      senseglove_state_pub_->msg_.header.stamp = ros::Time::now();
      senseglove_state_pub_->msg_.joint_names[k] = joint.getName();
      senseglove_state_pub_->msg_.position[k] = joint.getPosition();
      senseglove_state_pub_->msg_.absolute_velocity[k] = joint.getVelocity();
      
      senseglove_state_pub_->msg_.hand_position[k].x = robot.getHandPosition(k).GetX();
      senseglove_state_pub_->msg_.hand_position[k].y = robot.getHandPosition(k).GetY();
      senseglove_state_pub_->msg_.hand_position[k].z = robot.getHandPosition(k).GetZ();

    }
    for (int j = 0; j < 5; ++j)
    {
      senseglove_state_pub_->msg_.finger_tip_positions[j].x = robot.getFingerTip(j).GetX();
      senseglove_state_pub_->msg_.finger_tip_positions[j].y = robot.getFingerTip(j).GetY();
      senseglove_state_pub_->msg_.finger_tip_positions[j].z = robot.getFingerTip(j).GetZ();
    }
  }

  senseglove_state_pub_->unlockAndPublish();
}