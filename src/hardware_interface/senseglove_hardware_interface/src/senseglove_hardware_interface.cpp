// Copyright 2020 SenseGlove
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
  : sensegloveSetup(std::move(setup)), num_gloves_(this->sensegloveSetup->size())
{
}

bool SenseGloveHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle& /* robot_hw_nh */)
{
  ROS_INFO_STREAM("Senseglove HW Interface: Initializing realtime publisher for the SenseGlove states");

  std::string topicName = "/" + this->sensegloveSetup->getSenseGloveRobot(0).getRobotName() +
                          handedness[this->sensegloveSetup->getSenseGloveRobot(0).getRight()] + "/senseglove_states/";

  this->senseglove_state_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<senseglove_shared_resources::SenseGloveState>>(
          nh,
          "/" + this->sensegloveSetup->getSenseGloveRobot(0).getRobotName() +
              handedness[this->sensegloveSetup->getSenseGloveRobot(0).getRight()] + "/senseglove_states/",
          1);

  ROS_INFO_STREAM("Senseglove HW Interface: Constructed topic: " << topicName);  

  senseglove_haptics_sub_ = nh.subscribe("/" + this->sensegloveSetup->getSenseGloveRobot(0).getRobotName() +
                                         handedness[this->sensegloveSetup->getSenseGloveRobot(0).getRight()] + "/senseglove_haptics/",
                                         1, &SenseGloveHardwareInterface::hapticSubscriber, this); 
    
  this->uploadJointNames(nh);

  num_joints_ = this->sensegloveSetup->getSenseGloveRobot(0).getJointSize();

  this->reserveMemory();

  // Start ethercat cycle in the hardware
  this->sensegloveSetup->startCommunication(true);

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
        // Set the first target as the current position
        if (sensegloveSetup->getSenseGloveRobot(i).updateGloveData(ros::Duration(0.0)))
        {
          jointPosition[i][k] = joint.getPosition();
          jointVelocity[i][k] = joint.getVelocity();
          jointEffort[i][k] = 0.0;
        }

        if (joint.getActuationMode() == SGHardware::ActuationMode::position)
        {
          jointEffortCommand[i][k] = 0.0;
        }
        else if (joint.getActuationMode() == SGHardware::ActuationMode::torque)
        {
          jointEffortCommand[i][k] = 0.0;
        }
      }
    }
  }

  ROS_INFO_STREAM("Senseglove HW Interface: Successfully actuated all joints");

  this->registerInterface(&this->joint_state_interface_);
  this->registerInterface(&this->position_joint_interface_);
  this->registerInterface(&this->effort_joint_interface_);

  return true;
}

void SenseGloveHardwareInterface::read(const ros::Time& /* time */, const ros::Duration& elapsed_time)
{
  // read senseglove robot data
  for (size_t i = 0; i < num_gloves_; ++i)
  {
    if (sensegloveSetup->getSenseGloveRobot(i).updateGloveData(elapsed_time))
    {
      for (size_t k = 0; k < num_joints_; ++k)
      {
        SGHardware::Joint& joint = sensegloveSetup->getSenseGloveRobot(i).getJoint(k);

        if (!sensegloveSetup->getSenseGloveRobot(i).getRight() && k % 4 == 3)
        {
          jointPosition[i][k] = -joint.getPosition();  // finger_brake
                                                       // std::cout << joint.getName() << std::endl;
        }
        else
        {
          jointPosition[i][k] = joint.getPosition();
          jointVelocity[i][k] = joint.getVelocity();
          jointEffort[i][k] = joint.getTorque();
        }
      }
      this->updateSenseGloveState();
    }
  }
}

void SenseGloveHardwareInterface::write(const ros::Time& /* time */, const ros::Duration& /*&elapsed_time*/)
{
  for (size_t i = 0; i < num_gloves_; ++i)
  {
    // Splice joint_effort_command vector into vectors for FFB and vibration commands
    int j = 0;
    int h = 0;
    SGHardware::SenseGloveRobot& robot = sensegloveSetup->getSenseGloveRobot(i);
    for (size_t k = 0; k < num_joints_; k++)
    {
      SGHardware::Joint& joint = robot.getJoint(k);
      if (joint.canActuate())
      {
        if (joint.getActuationMode() == SGHardware::ActuationMode::position)
        {
          if (j % 2 == 1)
          {
            jointLastPositionCommand[i][h] = jointPositionCommand[i][k];
            h++;
          }
          else { jointLastVibrationCommand[i][h] = jointPositionCommand[i][k];}

          if (j == 9)  // Actuator List [10]
          {
            robot.actuateEffort(jointLastPositionCommand[i]);
            robot.actuateVibrations(jointLastVibrationCommand[i]);
          }
        }          
        else if (joint.getActuationMode() == SGHardware::ActuationMode::torque)
        {
          if (j % 2 == 1)
          {
            jointLastEffortCommand[i][j] = jointEffortCommand[i][k];
            h++;
          }
          else { jointLastVibrationCommand[i][j] = jointEffortCommand[i][k];}
          
          if (j == 9)  // Actuators 0 - 9
          {
            robot.actuateEffort(jointLastEffortCommand[i]);
            robot.actuateVibrations(jointLastVibrationCommand[i]);
          }
        } 
        j++;
      }
    }
    j = 0;
    h = 0;
  }
}

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
  senseglove_force_command_.resize(num_gloves_);
  senseglove_vibration_command_.resize(num_gloves_);
  
  for (unsigned int i = 0; i < num_gloves_; ++i)
  {
    jointPosition[i].resize(num_joints_, 0.0);
    jointPositionCommand[i].resize(num_joints_, 0.0);
    jointVelocity[i].resize(num_joints_, 0.0);
    jointVelocityCommand[i].resize(num_joints_, 0.0);
    jointEffort[i].resize(num_joints_, 0.0);
    jointEffortCommand[i].resize(num_joints_, 0.0);
    
    jointLastPositionCommand[i].resize(5, 0.0);
    jointLastVibrationCommand[i].resize(5, 0.0);
    jointLastEffortCommand[i].resize(5, 0.0);
    senseglove_force_command_[i].resize(5, 0.0);
    senseglove_vibration_command_[i].resize(5, 0.0);

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

void SenseGloveHardwareInterface::hapticSubscriber(const std_msgs::Float64MultiArray::ConstPtr &msg)
{ 
  for (size_t i = 0; i < num_gloves_; ++i) 
  {
    SGHardware::SenseGloveRobot& robot = sensegloveSetup->getSenseGloveRobot(i);

    for (size_t j = 0; j < 10; j++)
    {
      if (j < 5)
      {
        senseglove_force_command_[i].push_back(msg->data[j]);
      }
      else
      {
        senseglove_vibration_command_[i].push_back(msg->data[j]);
      }

      if (j == 9)  // actuators 0 - 9
      {
        robot.actuateEffort(senseglove_force_command_[i]);
        robot.actuateVibrations(senseglove_vibration_command_[i]);
        senseglove_force_command_[i].clear();
        senseglove_vibration_command_[i].clear();
      }
    }
  }

}
