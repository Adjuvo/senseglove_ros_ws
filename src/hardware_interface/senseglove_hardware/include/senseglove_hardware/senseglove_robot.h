/**
 * @file
 *
 * @author  Rogier
 * @author  Akshay Radhamohan Menon <akshay@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2024 senseglove
 *
 * @section DESCRIPTION
 *
 * A class to represent different actuation modes, with methods for 
 * conversion, comparison, and numerical & string representation of actuation modes.
 */
#ifndef ROS_WORKSPACE_senseglove_ROBOT_H
#define ROS_WORKSPACE_senseglove_ROBOT_H

#include "senseglove_hardware/joint.h"
#include "SenseGlove.hpp"
#include "SenseGloveInfo.hpp"
#include "SenseGloveSensorData.hpp"
#include "SenseGlovePose.hpp"
#include "SGConnect.hpp"

#include "BasicHandModel.hpp"
#include "HandPose.hpp"
#include "DeviceList.hpp"
#include "Vect3D.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>


//#include <urdf/model.h>

namespace senseglove
{
  class SenseGloveRobot
  {
  private:
    SGCore::SG::SenseGlove senseglove_;
    SGCore::SG::SenseGloveInfo model_;
    SGCore::SG::SenseGloveSensorData sensor_data_;
    SGCore::SG::SenseGlovePose glove_pose_;
    SGCore::Kinematics::BasicHandModel hand_model_;
    std::vector<SGCore::Kinematics::Vect3D> tip_positions_;
    SGCore::HandPose hand_pose_;
    ::std::vector<Joint> joint_list_;
    urdf::Model urdf_;
    const std::string name_;
    const SGCore::EDeviceType device_type_;
    const int robot_index_;
    bool updated_;

  public:
    using iterator = std::vector<Joint>::iterator;

    SenseGloveRobot(SGCore::SG::SenseGlove glove, ::std::vector<Joint> jointList, urdf::Model urdf, int robotIndex,
                    bool is_right);

    ~SenseGloveRobot();

    /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
    SenseGloveRobot(SenseGloveRobot&) = delete;
    SenseGloveRobot& operator=(SenseGloveRobot&) = delete;

    /* Delete move assignment since string cannot be move assigned */
    SenseGloveRobot(SenseGloveRobot&&) = default;
    SenseGloveRobot& operator=(SenseGloveRobot&&) = delete;

    std::string getName() const;
    int getIndex() const;
    bool getRight();

    Joint& getJoint(::std::string jointName);

    Joint& getJoint(size_t index);

    SGCore::Kinematics::Vect3D getHandPos(int i);
    SGCore::Kinematics::Vect3D getFingerTip(int i);

    // ros control works exclusively with doubles, but the sendHaptics function works with integers
    void actuateEffort(std::vector<double> effort_command);
    void actuateEffort(double e_0, double e_1, double e_2, double e_3, double e_4);
    void actuateBuzz(std::vector<double> buzz_command);
    void actuateBuzz(double b_0, double b_1, double b_2, double b_3, double b_4);
    void stopActuating();

    size_t size() const;

    iterator begin();
    iterator end();

    const urdf::Model& getUrdf() const;

    bool updateGloveData(const ros::Duration period);

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

#endif  // ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
