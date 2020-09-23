// Copyright 2020 SenseGlove.
#ifndef ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
#define ROS_WORKSPACE_SENSEGLOVE_ROBOT_H

#include "senseglove_hardware/communication/sense_com.h"
#include "senseglove_hardware/joint.h"

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
        ::std::vector<Joint> jointList;
        urdf::Model urdf_;
        Sensecom sensecom_;

    public:
        using iterator = std::vector<Joint>::iterator;

        SenseGloveRobot(::std::vector<Joint> jointList, urdf::Model urdf, int cycle_time);

        ~SenseGloveRobot();

        /* Delete move constructor/assignment since atomic bool cannot be moved */
//        SenseGloveRobot(SenseGloveRobot&&) = delete;
//        SenseGloveRobot& operator=(SenseGloveRobot&&) = delete;

        void startCommunication(bool /*reset*/);

        void stopCommunication();

        bool isCommunicationOperational();

        Joint& getJoint(::std::string jointName);

        Joint& getJoint(size_t index);

        size_t size() const;

        iterator begin();
        iterator end();

        const urdf::Model& getUrdf() const;

        /** @brief Override comparison operator */
        friend bool operator==(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
        {
            if (lhs.jointList.size() != rhs.jointList.size())
            {
                return false;
            }
            for (unsigned int i = 0; i < lhs.jointList.size(); i++)
            {
                const senseglove::Joint& lhsJoint = lhs.jointList.at(i);
                const senseglove::Joint& rhsJoint = rhs.jointList.at(i);
                if (lhsJoint != rhsJoint)
                {
                    return false;
                }
            }
            return true;
        }

        /** @brief Override stream operator for clean printing */
        friend ::std::ostream& operator<<(std::ostream& os, const SenseGloveRobot& senseGloveRobot)
        {
            for (unsigned int i = 0; i < senseGloveRobot.jointList.size(); i++)
            {
                os << senseGloveRobot.jointList.at(i) << "\n";
            }
            return os;
        }
    };
}  // namespace senseglove

#endif //ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
