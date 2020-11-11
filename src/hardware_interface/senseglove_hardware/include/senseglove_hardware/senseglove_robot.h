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
        SGCore::SG::SenseGlove senseglove_;
        SGCore::SG::SG_GloveInfo model_;
        SGCore::SG::SG_SensorData sensor_data_;
        SGCore::SG::SG_GlovePose glove_pose_;
        ::std::vector<Joint> joint_list_;
        urdf::Model urdf_;
        const std::string name_;
        const SGCore::DeviceType device_type_;
        const int robot_index_;

    public:
        using iterator = std::vector<Joint>::iterator;

        SenseGloveRobot(SGCore::SG::SenseGlove glove, ::std::vector<Joint> jointList, urdf::Model urdf, int robotIndex);

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

        void updateGloveData(const ros::Duration period);

        /** @brief Override comparison operator */
        friend bool operator==(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
        {
            if (lhs.joint_list_.size() != rhs.joint_list_.size())
            {
                return false;
            }
            for (unsigned int i = 0; i < lhs.joint_list_.size(); i++)
            {
                const senseglove::Joint& lhsJoint = lhs.joint_list_.at(i);
                const senseglove::Joint& rhsJoint = rhs.joint_list_.at(i);
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
            for (unsigned int i = 0; i < senseGloveRobot.joint_list_.size(); i++)
            {
                os << senseGloveRobot.joint_list_.at(i) << "\n";
            }
            return os;
        }
    };
}  // namespace senseglove

#endif //ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
