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
#ifndef ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
#define ROS_WORKSPACE_SENSEGLOVE_ROBOT_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "joint.h"
#include "BasicHandModel.hpp"
#include "HandPose.hpp"
#include "DeviceList.hpp"
#include "Vect3D.hpp"

#include "SenseGlove.hpp"
#include "SenseGloveSensorData.hpp"
#include "SenseGlovePose.hpp"

#include "NovaGlove.hpp"
#include "NovaGloveSensorData.hpp"

#include "Nova2Glove.hpp"
#include "Nova2GloveSensorData.hpp"

using namespace SGCore;
using namespace SGCore::SG;
using namespace SGCore::Nova;

namespace SGHardware
{
  class SenseGloveRobot
  {
  private:
    // DK1 Specific
    // SenseGlove senseglove;    
    SenseGloveSensorData sensegloveSensorData;
    SenseGlovePose senseglovePose;

    //Nova Specific
    NovaGlove novaglove;    
    NovaGloveSensorData novaSensorData;  

    // Nova2 Specific
    Nova2Glove nova2glove;    
    Nova2GloveSensorData nova2SensorData;

    // Shared
    std::shared_ptr<HapticGlove> hapticglove;
    HandPose handPose;
    Kinematics::BasicHandModel handModel;
    Kinematics::Vect3D jointPosition;
    Kinematics::Vect3D tipPositions;  
    std::vector<std::vector<Kinematics::Vect3D>> handPoseAngles;

    std::shared_ptr<SenseGlove> senseglovePtr = std::dynamic_pointer_cast<SenseGlove>(hapticglove);
    std::shared_ptr<NovaGlove> novaglovePtr = std::dynamic_pointer_cast<NovaGlove>(hapticglove);
    std::shared_ptr<Nova2Glove> nova2glovePtr = std::dynamic_pointer_cast<Nova2Glove>(hapticglove);

    ::std::vector<Joint> jointList;
    urdf::Model urdfModel;
    const std::string SenseGloveRobotName;
    const EDeviceType deviceType;
    const int robotIndex;
    bool isUpdated;

  public:
  
    using iterator = std::vector<Joint>::iterator;

    SenseGloveRobot(std::shared_ptr<HapticGlove> hapticglove, ::std::vector<Joint> jointList, urdf::Model urdfModel, int robotIndex, bool isRight);
    ~SenseGloveRobot();

    /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
    SenseGloveRobot(SenseGloveRobot&) = delete;
    SenseGloveRobot& operator=(SenseGloveRobot&) = delete;

    /* Delete move assignment since string cannot be move assigned */
    SenseGloveRobot(SenseGloveRobot&&) = default;
    SenseGloveRobot& operator=(SenseGloveRobot&&) = delete;

    std::string getRobotName() const;
    EDeviceType getRobotType() const;
    int getRobotIndex() const;
    bool getRight();

    Joint& getJoint(::std::string jointName);
    Joint& getJoint(size_t index);

    size_t getJointSize();

    Kinematics::Vect3D getHandPosition(int i);
    Kinematics::Vect3D getFingerTip(int i);

    // ros control works exclusively with doubles, but the sendHaptics function works with integers
    void actuateEffort(std::vector<double> effortCommand);
    void actuateEffort(double e_0, double e_1, double e_2, double e_3, double e_4);
    void actuateVibrations(std::vector<double> vibrationCommand);
    void actuateVibrations(double v_0, double v_1, double v_2, double v_3, double v_4);
    void stopActuating();

    size_t size() const;
    
    iterator begin();
    iterator end();

    const urdf::Model& getUrdf() const;

    bool updateGloveData(const ros::Duration period);

    /** @brief Override comparison operator */
    friend bool operator==(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
    {
      if (lhs.jointList.size() != rhs.jointList.size())
      {
        return false;
      }
      for (unsigned int i = 0; i < lhs.jointList.size(); i++)
      {
        const SGHardware::Joint& lhsJoint = lhs.jointList.at(i);
        const SGHardware::Joint& rhsJoint = rhs.jointList.at(i);
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
      for (unsigned int i = 0; i < senseGloveRobot.jointList.size(); i++)
      {
        os << senseGloveRobot.jointList.at(i) << "\n";
      }
      return os;
    }
  };
}  // namespace SGHardware

#endif  // ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
