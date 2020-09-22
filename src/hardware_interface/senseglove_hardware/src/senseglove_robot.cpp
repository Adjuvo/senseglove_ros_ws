// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
//#include "senseglove_hardware/senseglove_robot.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace senseglove
{
    SenseGloveRobot::SenseGloveRobot(::std::vector<Joint> jointList, urdf::Model urdf)
            : jointList(std::move(jointList))
            , urdf_(std::move(urdf))
    {
    }

    void SenseGloveRobot::startCommunication(bool reset_imc)
    {

    }

    void SenseGloveRobot::stopCommunication()
    {

    }

    bool SenseGloveRobot::isCommunicationOperational()
    {
        return false;
    }

    Joint& SenseGloveRobot::getJoint(::std::string jointName)
    {
        if (!Sensecom.isOperational())
        {
            ROS_WARN("Trying to access joints while communication is not operational. This "
                     "may lead to incorrect sensor data.");
        }
        for (auto& joint : jointList)
        {
            if (joint.getName() == jointName)
            {
                return joint;
            }
        }

        throw std::out_of_range("Could not find joint with name " + jointName);
    }

    Joint& SenseGloveRobot::getJoint(size_t index)
    {
        if (!Sensecom.isOperational())
        {
            ROS_WARN("Trying to access joints while ethercat is not operational. This "
                     "may lead to incorrect sensor data.");
        }
        return this->jointList.at(index);
    }

    size_t SenseGloveRobot::size() const
    {
        return this->jointList.size();
    }

    SenseGloveRobot::iterator SenseGloveRobot::begin()
    {
        if (!Sensecom.isOperational())
        {
            ROS_WARN("Trying to access joints while ethercat is not operational. This "
                     "may lead to incorrect sensor data.");
        }
        return this->jointList.begin();
    }

    SenseGloveRobot::iterator SenseGloveRobot::end()
    {
        return this->jointList.end();
    }

    SenseGloveRobot::~SenseGloveRobot()
    {
        stopCommunication();
    }

    const urdf::Model& SenseGloveRobot::getUrdf() const
    {
        return this->urdf_;
    }

}  // namespace senseglove
