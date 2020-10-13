// Copyright 2020 SenseGlove
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace senseglove
{
    SenseGloveRobot::SenseGloveRobot(::std::vector<Joint> jointList, urdf::Model urdf)
            : jointList_(std::move(jointList))
            , urdf_(std::move(urdf))
    {
    }

    std::string SenseGloveRobot::getName() const
    {
        return this->name_;
    }

    Joint& SenseGloveRobot::getJoint(::std::string jointName)
    {
        for (auto& joint : jointList_)
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
        return this->jointList_.at(index);
    }

    size_t SenseGloveRobot::size() const
    {
        return this->jointList_.size();
    }

    SenseGloveRobot::iterator SenseGloveRobot::begin()
    {
        return this->jointList_.begin();
    }

    SenseGloveRobot::iterator SenseGloveRobot::end()
    {
        return this->jointList_.end();
    }

    SenseGloveRobot::~SenseGloveRobot()
    {
    }

    const urdf::Model& SenseGloveRobot::getUrdf() const
    {
        return this->urdf_;
    }

}  // namespace senseglove
