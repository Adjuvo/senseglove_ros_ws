// Copyright 2019 Project March.
#include "senseglove_hardware/joint.h"

#include <ros/ros.h>

#include <bitset>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace senseglove
{
    Joint::Joint(std::string name, int net_number) : name_(std::move(name)), net_number_(net_number)
    {
    }

    Joint::Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<IMotionCube> imc)
            : name_(std::move(name)), net_number_(net_number), allow_actuation_(allow_actuation), imc_(std::move(imc))
    {
    }

    Joint::Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<IMotionCube> imc,
                 std::unique_ptr<TemperatureGES> temperature_ges)
            : name_(std::move(name))
            , net_number_(net_number)
            , allow_actuation_(allow_actuation)
            , imc_(std::move(imc))
            , temperature_ges_(std::move(temperature_ges))
    {
    }

    bool Joint::initialize(int cycle_time)
    {
        return false;
    }

    void Joint::prepareActuation()
    {
        if (!this->canActuate())
        {
            ROS_ERROR("Failed to prepare joint %s for actuation", this->name_.c_str());
        }
        ROS_INFO("[%s] Preparing for actuation", this->name_.c_str());
        this->position_ = this->readAngle();
        this->velocity_ = 0;
        ROS_INFO("[%s] Successfully prepared for actuation", this->name_.c_str());
    }

    void Joint::actuateRad(double target_position)
    {
        if (!this->canActuate())
        {
            ROS_ERROR("Joint %s is not allowed to actuate", this->name_.c_str());
        }
        this->imc_->actuateRad(target_position);
    }

    void Joint::readAngle(/*const ros::Duration& elapsed_time*/)
    {
        // get angle from finger array at correct index
    }

    double Joint::getPosition() const
    {
        return this->position_;
    }

    double Joint::getVelocity() const
    {
        return this->velocity_;
    }

    void Joint::actuateTorque(int16_t target_torque)
    {
        if (!this->canActuate())
        {
            ROS_ERROR("Joint %s is not allowed to actuate", this->name_.c_str());
        }
        forcefeedbackcommand();
    }

    double Joint::getTorque()
    {
        return 0.0;
    }

    void Joint::setAllowActuation(bool allow_actuation)
    {
        this->allow_actuation_ = allow_actuation;
    }

    int Joint::getNetNumber() const
    {
        return this->net_number_;
    }

    std::string Joint::getName() const
    {
        return this->name_;
    }

    bool Joint::canActuate() const
    {
        return this->allow_actuation_ && this->hasIMotionCube();
    }

    ActuationMode Joint::getActuationMode() const
    {
        return this->imc_->getActuationMode();
    }
}  // namespace senseglove
