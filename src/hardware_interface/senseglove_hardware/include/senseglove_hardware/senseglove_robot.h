// Copyright 2020 SenseGlove.
#ifndef ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
#define ROS_WORKSPACE_SENSEGLOVE_ROBOT_H

#include "senseglove_hardware/communication/sense_com.h"
#include "senseglove_hardware/joint.h"
#include "SenseGlove.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

namespace senseglove
{
    class SenseGloveRobot
    {
    private:
        ::std::vector<Joint> jointList_;
        urdf::Model urdf_;
        SGCore::SG::SenseGlove senseglove_;
        const std::string name_;

    public:
        using iterator = std::vector<Joint>::iterator;

        SenseGloveRobot(::std::vector<Joint> jointList, urdf::Model urdf);

        ~SenseGloveRobot();

        /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
        SenseGloveRobot(SenseGloveRobot&) = delete;
        SenseGloveRobot& operator=(SenseGloveRobot&) = delete;

        /* Delete move assignment since string cannot be move assigned */
        SenseGloveRobot(SenseGloveRobot&&) = default;
        SenseGloveRobot& operator=(SenseGloveRobot&&) = delete;

        std::string getName() const;

        Joint& getJoint(::std::string jointName);

        Joint& getJoint(size_t index);

        size_t size() const;

        iterator begin();
        iterator end();

        const urdf::Model& getUrdf() const;

        /** @brief Override comparison operator */
        friend bool operator==(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
        {
            if (lhs.jointList_.size() != rhs.jointList_.size())
            {
                return false;
            }
            for (unsigned int i = 0; i < lhs.jointList_.size(); i++)
            {
                const senseglove::Joint& lhsJoint = lhs.jointList_.at(i);
                const senseglove::Joint& rhsJoint = rhs.jointList_.at(i);
                if (lhsJoint != rhsJoint)
                {
                    return false;
                }
            }
            return true;
        }

        friend bool operator!=(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
        {
            return !(lhs == rhs);
        }

        /** @brief Override stream operator for clean printing */
        friend ::std::ostream& operator<<(std::ostream& os, const SenseGloveRobot& senseGloveRobot)
        {
            for (unsigned int i = 0; i < senseGloveRobot.jointList_.size(); i++)
            {
                os << senseGloveRobot.jointList_.at(i) << "\n";
            }
            return os;
        }
    };
}  // namespace senseglove

#endif //ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
