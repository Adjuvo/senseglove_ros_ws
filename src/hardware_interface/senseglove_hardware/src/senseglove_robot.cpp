// Copyright (c) 2020 - 2024 SenseGlove
#include <senseglove_hardware/joint.h>
#include <senseglove_hardware/senseglove_robot.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <numeric>

#include <limits>

#include <ros/ros.h>

namespace SGHardware
{
  SenseGloveRobot::SenseGloveRobot(std::shared_ptr<HapticGlove> glove, ::std::vector<Joint> jointList, urdf::Model urdfModel, int robotIndex, bool isRight)
    : hapticglove(glove)
    , handModel(Kinematics::BasicHandModel::Default(isRight))
    , jointList(std::move(jointList))
    , urdfModel(std::move(urdfModel))
    , SenseGloveRobotName("/senseglove/" + std::to_string(int((robotIndex) / 2)))
    , deviceType(this->hapticglove->GetDeviceType())
    , robotIndex(robotIndex)
    , isUpdated(false)
  {
  }

  std::string SenseGloveRobot::getRobotName() const
  {
    return this->SenseGloveRobotName;
  }

  EDeviceType SenseGloveRobot::getRobotType() const
  {
    return this->deviceType;
  }

  int SenseGloveRobot::getRobotIndex() const
  {
    return this->robotIndex;
  }

  bool SenseGloveRobot::getRight()
  {
    return this->hapticglove->IsRight();
  }

  Joint& SenseGloveRobot::getJoint(::std::string jointName)
  {
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
    return this->jointList.at(index);
  }

  size_t SenseGloveRobot::getJointSize()
  {
    return this->jointList.size();
  }

  // Function to flatten vector of vectors of Vector3D
  Kinematics::Vect3D SenseGloveRobot::getHandPosition(int i)
  {
    // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
    // SG uses vector of vectors and ROS uses one long array 
    if (senseglovePtr != nullptr)
    {
      jointPosition = handPose.GetJointPositions()[std::floor(i / 4)][i % 4];
    }
    else if (novaglovePtr != nullptr || nova2glovePtr != nullptr)
    {
      if (i > 19) { jointPosition = {0.0, 0.0, 0.0}; }
      else { jointPosition = handPose.GetJointPositions()[std::floor(i / 4)][i % 4]; } 
    }
    return jointPosition;
  }

  Kinematics::Vect3D SenseGloveRobot::getFingerTip(int i)
  {
    // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
    // SG uses vector of vectors and ROS uses one long array
    if (senseglovePtr != nullptr)
    {
      tipPositions = senseglovePose.CalculateFingertips(senseglovePtr->GetFingerThimbleOffsets())[i];
    }
    else if (novaglovePtr != nullptr || nova2glovePtr != nullptr)
    {
      if (i > 19) { tipPositions = {0.0, 0.0, 0.0}; }
      else { tipPositions = handPose.GetJointPositions()[i][3]; }
    }
    return tipPositions;
  }

  void SenseGloveRobot::actuateEffort(std::vector<double> effortCommand)
  {
    if (DeviceList::SenseComRunning())  // check if the Sense Comm is running. If not, warn the end user.
    {
      std::vector<float> effortLevels(effortCommand.begin(), effortCommand.end());
      if ((std::accumulate(effortLevels.begin(), effortLevels.end(), decltype(effortLevels)::value_type(0.0f))) < 10.0)  // less than noticable ffb
      {
        this->hapticglove->StopHaptics();
      }
      else
      {
        this->hapticglove->QueueForceFeedbackLevels(effortLevels);
        this->hapticglove->SendHaptics();
      }

      if (nova2glovePtr != nullptr)
      {
        nova2glovePtr->QueueSqueezeLevel(effortLevels[effortLevels.size() - 1]);
        nova2glovePtr->SendHaptics();
      }
    }
  }

  void SenseGloveRobot::actuateVibrations(std::vector<double> vibrationCommand)
  {
    std::vector<float> vibrationLevels(vibrationCommand.begin(), vibrationCommand.end());

    if ((std::accumulate(vibrationLevels.begin(), vibrationLevels.end(), decltype(vibrationLevels)::value_type(0.0f))) < 10.0)  // less than noticable buzz
    {
      this->hapticglove->StopVibrations();
    }
    else
    {
      this->hapticglove->QueueVibroLevels(vibrationLevels);
      this->hapticglove->SendHaptics();
    }

    if (novaglovePtr != nullptr)
    {
      novaglovePtr->QueueWristLevel(vibrationLevels[vibrationLevels.size() - 1]);
      novaglovePtr->SendHaptics();
    }

    if (nova2glovePtr != nullptr)
    {
      nova2glovePtr->QueueVibroLevel(EHapticLocation::PalmIndexSide, vibrationLevels[vibrationLevels.size() - 2]);
      nova2glovePtr->QueueVibroLevel(EHapticLocation::PalmPinkySide, vibrationLevels[vibrationLevels.size() - 1]);
      nova2glovePtr->SendHaptics();
    }
  }

  void SenseGloveRobot::stopActuating()
  {
    this->hapticglove->StopHaptics();
  }

  size_t SenseGloveRobot::size() const
  {
    return this->jointList.size();
  }

  SenseGloveRobot::iterator SenseGloveRobot::begin()
  {
    return this->jointList.begin();
  }

  SenseGloveRobot::iterator SenseGloveRobot::end()
  {
    return this->jointList.end();
  }

  SenseGloveRobot::~SenseGloveRobot()
  {
  }

  bool SenseGloveRobot::updateGloveData(const ros::Duration period)
  {
    bool gloveUpdate = false;
    bool handUpdate = false;

    if (senseglovePtr != nullptr)
    {
      if (senseglovePtr->GetSensorData(sensegloveSensorData))  // If GetSensorData is true, we have sucesfully received data
      {
        for (auto& joint : jointList)
        {
          joint.position = sensegloveSensorData.GetSensorAngles()[joint.jointIndex / 4][joint.jointIndex % 4];
          double intermediateVelocity = (sensegloveSensorData.GetSensorAngles()[joint.jointIndex / 4][joint.jointIndex % 4] - joint.velocity);

          if (intermediateVelocity != 0.0 and period.toSec() != 0.0) { joint.velocity = intermediateVelocity / 1.0; }
          else { joint.velocity = 0.0; }
        }
      }

      if (!senseglovePtr->GetGlovePose(senseglovePose)) { ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data"); }
      else { gloveUpdate = true; }

      if (!senseglovePtr->GetHandPose(this->handModel, this->handPose)) { ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data"); }
      else { handUpdate = true; }
    }
    else if (novaglovePtr != nullptr)
    {
      handPoseAngles = handPose.GetHandAngles();
      if (!handPoseAngles.empty())
      {
        for (auto& joint : jointList)
        {
          if (joint.jointIndex > 19) 
          { 
            joint.position = 0.0; 
          }
          else
          {
            if (joint.jointIndex % 4 == 0) { joint.position = handPoseAngles[std::floor(joint.jointIndex / 4)][0].GetZ(); }
            else { joint.position = handPoseAngles[std::floor(joint.jointIndex / 4)][joint.jointIndex % 4 - 1].GetY(); }
          }
        }
      }

      if (!novaglovePtr->GetSensorData(novaSensorData)) { ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data"); }
      else { gloveUpdate = true; }

      if (!novaglovePtr->GetHandPose(this->handModel, this->handPose)) { ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data"); }
      else { handUpdate = true; }
    }

    else if (nova2glovePtr != nullptr)
    {
      handPoseAngles = handPose.GetHandAngles();
      if (!handPoseAngles.empty())
      {
        for (auto& joint : jointList)
        {
          if (joint.jointIndex > 19) 
          { 
            joint.position = 0.0; 
          }
          else
          {
            if (joint.jointIndex % 4 == 0) { joint.position = handPoseAngles[std::floor(joint.jointIndex / 4)][0].GetZ(); }
            else { joint.position = handPoseAngles[std::floor(joint.jointIndex / 4)][joint.jointIndex % 4 - 1].GetY(); }
          }
        }
      }

      if (!nova2glovePtr->GetSensorData(nova2SensorData)) { ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data"); }
      else { gloveUpdate = true; }

      if (!nova2glovePtr->GetHandPose(this->handModel, this->handPose)) { ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data"); }
      else { handUpdate = true; }
    }
    isUpdated |= (gloveUpdate and handUpdate);
    return isUpdated;
  }


  const urdf::Model& SenseGloveRobot::getUrdf() const
  {
    return this->urdfModel;
  }

}  // namespace SGHardware
