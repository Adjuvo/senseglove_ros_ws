// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"
#include "senseglove_hardware/senseglove_setup.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace senseglove
{
    SenseGloveSetup::SenseGloveSetup(std::vector<senseglove::SenseGloveRobot> sensegloves, urdf::Model urdf_left, urdf::Model urdf_right, int cycle_time)
            : sensegloves_(sensegloves), urdf_left_(std::move(urdf_left)), urdf_right_(std::move(urdf_right)),
            sensecom_(/*max slave/robot index*/0, cycle_time)
    {
    }

    void SenseGloveSetup::startCommunication(bool /*reset*/)
    {

    }

    void SenseGloveSetup::stopCommunication()
    {

    }

    bool SenseGloveSetup::isCommunicationOperational()
    {
        return false;
    }

    const SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(::std::string gloveName) const
    {
        if (!sensecom_.isOperational())
        {
            ROS_WARN("Trying to access joints while communication is not operational. This "
                     "may lead to incorrect sensor data.");
        }
        for (auto& glove : sensegloves_)
        {
            if (glove.getName() == gloveName)
            {
                return glove;
            }
        }

        throw std::out_of_range("Could not find glove with name " + gloveName);
    }

    const SenseGloveRobot& SenseGloveSetup::getSenseGloveRobot(int index) const
    {
        if (!this->sensecom_.isOperational())
        {
            ROS_WARN("Trying to access joints while ethercat is not operational. This "
                     "may lead to incorrect sensor data.");
        }
        return sensegloves_.at(index);
    }

    size_t SenseGloveSetup::size() const
    {
        return sensegloves_.size();
    }

    SenseGloveSetup::iterator SenseGloveSetup::begin()
    {
        if (!sensecom_.isOperational())
        {
            ROS_WARN("Trying to access sensegloves while communication is not operational. This "
                     "may lead to incorrect sensor data.");
        }
        return this->sensegloves_.begin();
    }

    SenseGloveSetup::iterator SenseGloveSetup::end()
    {
        return this->sensegloves_.end();
    }

    SenseGloveSetup::~SenseGloveSetup()
    {
        stopCommunication();
    }

    const urdf::Model& SenseGloveSetup::getRobotUrdf(std::string glove_robot_name) const
    {
        return getSenseGloveRobot(glove_robot_name).getUrdf();
    }

}  // namespace senseglove
