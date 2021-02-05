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

SenseGloveHardwareInterface::SenseGloveHardwareInterface(std::unique_ptr<senseglove::SenseGloveSetup> setup)
        : senseglove_setup_(std::move(setup)), num_gloves_(this->senseglove_setup_->size()){
}


bool SenseGloveHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle & /* robot_hw_nh */) {
  // Initialize realtime publisher for the SenseGlove states
  this->senseglove_state_pub_ =
      std::make_unique < realtime_tools::RealtimePublisher < senseglove_shared_resources::SenseGloveState >> (
          nh, "/" + this->senseglove_setup_->getSenseGloveRobot(0).getName() + "/senseglove_states/", 4);

  this->uploadJointNames(nh);

    num_joints_ = 20; // make dependent on size of senseglove robot
    this->reserveMemory();

    // Start ethercat cycle in the hardware
    this->senseglove_setup_->startCommunication(true);

    //std::string senseglove_name = this->senseglove_setup_->getSenseGloveRobot(0).getName();



    for(size_t i = 0; i < num_gloves_; ++i){
        // Initialize interfaces for each joint
        for (size_t k = 0; k < num_joints_; ++k) {
            senseglove::Joint &joint = this->senseglove_setup_->getSenseGloveRobot(i).getJoint(k);
            ROS_DEBUG_STREAM("Obtained necessary joint");

            // Create joint state interface
            JointStateHandle joint_state_handle(joint.getName(), &joint_position_[i][k], &joint_velocity_[i][k],
                                                &joint_effort_[i][k]);
            ROS_DEBUG_STREAM("joint state interface handle created");
            joint_state_interface_.registerHandle(joint_state_handle);
            ROS_DEBUG_STREAM("joint state interface handle registered");

            if (joint.getActuationMode() == senseglove::ActuationMode::position) {
                // Create position joint interface
                JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[i][k]);
                position_joint_interface_.registerHandle(joint_position_handle);

            } else if (joint.getActuationMode() == senseglove::ActuationMode::torque) {
                // Create effort joint interface
                JointHandle joint_effort_handle_(joint_state_handle, &joint_effort_command_[i][k]);
                effort_joint_interface_.registerHandle(joint_effort_handle_);
            }
            ROS_DEBUG_STREAM("Handles registered");

            // Create velocity joint interface
            JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_command_[i][k]);
            velocity_joint_interface_.registerHandle(joint_velocity_handle);

            // Prepare joints for actuation
            if (joint.canActuate())
            {
                // Set the first target as the current position
                if(senseglove_setup_->getSenseGloveRobot(i).updateGloveData(ros::Duration(0.0)))
                {
                  joint_position_[i][k] = joint.getPosition();
                  joint_velocity_[i][k] = joint.getVelocity();
                  joint_effort_[i][k] = 0.0;
                }

                if (joint.getActuationMode() == senseglove::ActuationMode::position) {
                  joint_effort_command_[i][k] = 0.0;
                } else if (joint.getActuationMode() == senseglove::ActuationMode::torque){
                  joint_effort_command_[i][k] = 0.0;
                }
            }
        }
    }
    ROS_INFO("Successfully actuated all joints");

    this->registerInterface(&this->joint_state_interface_);
    this->registerInterface(&this->position_joint_interface_);
    this->registerInterface(&this->effort_joint_interface_);

    return true;
}

void SenseGloveHardwareInterface::validate()
{
    // TO DO
}

void SenseGloveHardwareInterface::read(const ros::Time & /* time */, const ros::Duration &elapsed_time)
{
    // read senseglove robot data
    for (size_t i = 0; i < num_gloves_; ++i) {
        if(senseglove_setup_->getSenseGloveRobot(i).updateGloveData(elapsed_time))
        {
          for (size_t k = 0; k < num_joints_; ++k) {
            senseglove::Joint &joint = senseglove_setup_->getSenseGloveRobot(i).getJoint(k);

            // Update position with he most accurate velocity
//            joint.readAngle();
            joint_position_[i][k] = joint.getPosition();
            joint_velocity_[i][k] = joint.getVelocity();
            joint_effort_[i][k] = joint.getTorque();
          }
          this->updateSenseGloveState();
        }
    }
}

void SenseGloveHardwareInterface::write(const ros::Time & /* time */, const ros::Duration &/*&elapsed_time*/) {
    for (size_t i = 0; i < num_gloves_; ++i) {
//        for (size_t k = 0; k < num_joints_; k++)
//        {
//            senseglove::Joint &joint = this->senseglove_setup_->getSenseGloveRobot(i).getJoint(k);
//            if (joint.getActuationMode() == senseglove::ActuationMode::position)
//            {
//
//              joint_position_command_[i][k] = joint_last_position_command_[i][k];
//              has_actuated_ |= (joint_position_command_[i][k] != 0.0);
//            }
//            else if (joint.getActuationMode() == senseglove::ActuationMode::torque)
//            {
//              joint_effort_command_[i][k] = joint_last_effort_command_[i][k];
//              has_actuated_ |= (joint_effort_command_[i][k] != 0.0);
//            }
//        }
//
//        if (not has_actuated_) {
//            bool found_non_zero = false;
//            for (size_t k = 0; k < num_joints_; k++)
//            {
//                if (joint_effort_command_[i][k] != 0)
//                {
//                    ROS_ERROR("Non-zero effort on first actuation for joint %s",
//                              senseglove_setup_->getSenseGloveRobot(i).getJoint(k).getName().c_str());
//                    found_non_zero = true;
//                }
//            }
//            if (found_non_zero) {
//                throw std::runtime_error("Safety limits acted before actual controller started actuating");
//            }
//        }

        // Splice joint_effort_command vector into vectors for ffb and buzz commands
        int j = 0;
        int h = 0;
        senseglove::SenseGloveRobot &robot = senseglove_setup_->getSenseGloveRobot(i);
        for (size_t k= 0; k < num_joints_; k++)
        {
            senseglove::Joint &joint = robot.getJoint(k);
            if(joint.canActuate())
            {
                if (joint.getActuationMode() == senseglove::ActuationMode::position)
                {
                    if (j % 2 == 1)
                    {
                        joint_last_position_command_[i][h] = joint_position_command_[i][k];
                        h++;
                    }
                    else
                    {
                        joint_last_buzz_command_[i][h] = joint_position_command_[i][k];
                    }

                    if (j == 9)
                    {
                        robot.actuateEffort(joint_last_position_command_[i]);
                        robot.actuateBuzz(joint_last_buzz_command_[i]);
                    }
                }
                else if (joint.getActuationMode() == senseglove::ActuationMode::torque)
                {
                    if (j % 2 == 1)
                    {
                        joint_last_effort_command_[i][j] = joint_effort_command_[i][k];
                        h++;
                    }
                    else
                    {
                        joint_last_buzz_command_[i][j] = joint_effort_command_[i][k];
                    }

                    if (j == 9)
                    {
                        robot.actuateEffort(joint_last_effort_command_[i]);
                        robot.actuateBuzz(joint_last_buzz_command_[i]);
                    }
                }
                j++;
            }
        }
        j = 0;
        h = 0;
    }
}

void SenseGloveHardwareInterface::uploadJointNames(ros::NodeHandle &nh) const {
    std::vector <std::string> joint_names;
    for (const auto &joint : *this->senseglove_setup_) {
        joint_names.push_back(joint.getName());
    }
    std::sort(joint_names.begin(), joint_names.end());
    nh.setParam("/senseglove/joint_names", joint_names);
}

void SenseGloveHardwareInterface::reserveMemory() {
    joint_position_.resize(num_gloves_);
    joint_position_command_.resize(num_gloves_);
    joint_last_position_command_.resize(num_gloves_);
    joint_velocity_.resize(num_gloves_);
    joint_velocity_command_.resize(num_gloves_);
    joint_effort_.resize(num_gloves_);
    joint_effort_command_.resize(num_gloves_);
    joint_last_effort_command_.resize(num_gloves_);
    joint_last_buzz_command_.resize(num_gloves_);
    for (unsigned int i = 0; i < num_gloves_; ++i)
    {
      joint_position_[i].resize(num_joints_, 0.0);
      joint_position_command_[i].resize(num_joints_, 0.0);
      joint_last_position_command_[i].resize(5, 0.0);
      joint_velocity_[i].resize(num_joints_, 0.0);
      joint_velocity_command_[i].resize(num_joints_, 0.0);
      joint_effort_[i].resize(num_joints_, 0.0);
      joint_effort_command_[i].resize(num_joints_, 0.0);
      joint_last_effort_command_[i].resize(5, 0.0);
      joint_last_buzz_command_[i].resize(5, 0.0);
    }

    senseglove_state_pub_->msg_.joint_names.resize(num_gloves_*num_joints_);
    senseglove_state_pub_->msg_.position.resize(num_gloves_*num_joints_);
    senseglove_state_pub_->msg_.absolute_velocity.resize(num_gloves_*num_joints_);
    senseglove_state_pub_->msg_.hand_position.resize(num_gloves_*num_joints_);
}

void SenseGloveHardwareInterface::updateSenseGloveState() {
    if (!senseglove_state_pub_->trylock()) {
        return;
    }

    senseglove_state_pub_->msg_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < num_gloves_; ++i)
    {
      senseglove::SenseGloveRobot &robot = senseglove_setup_->getSenseGloveRobot(i);
      for (size_t k = 0; k < num_joints_; ++k)
      {
        senseglove::Joint& joint = robot.getJoint(k);
        //        senseglove::SenseGloveState senseglove_state = joint.getSenseGloveState();
        senseglove_state_pub_->msg_.header.stamp = ros::Time::now();
        senseglove_state_pub_->msg_.joint_names[i] = joint.getName();
        senseglove_state_pub_->msg_.position[i] = joint.getPosition();
        senseglove_state_pub_->msg_.absolute_velocity[i] = joint.getVelocity();
        senseglove_state_pub_->msg_.hand_position[k].x = robot.getHandPos(k).x;
        senseglove_state_pub_->msg_.hand_position[k].y = robot.getHandPos(k).y;
        senseglove_state_pub_->msg_.hand_position[k].z = robot.getHandPos(k).z;
      }
    }

    senseglove_state_pub_->unlockAndPublish();
}
