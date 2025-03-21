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
    , SenseGloveRobotName("senseglove/" + std::to_string(int((robotIndex) / 2)))
    , deviceType(this->hapticglove->GetDeviceType())
    , robotIndex(robotIndex)
    , isUpdated(false)
  {
    for (size_t i = 0; i < this->jointList.size(); ++i)
    {
      jointMap[this->jointList[i].getName()] = i;
    }
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

  Joint& SenseGloveRobot::getJoint(const ::std::string jointName)
  {
    auto it = jointMap.find(jointName);
    if (it != jointMap.end()) 
    {
      return jointList[it->second]; // O(1) lookup
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

  size_t SenseGloveRobot::getEffortJointSize()
  {
    size_t effortJointSize = 0;
      
    for (auto& joint : jointList)
    {
      if (joint.getActuationType() == ActuationType::brake || 
          joint.getActuationType() == ActuationType::squeeze)
      {
        effortJointSize++;
      }
    }
    return effortJointSize;
  }

  size_t SenseGloveRobot::getVibrationJointSize()
  {
    size_t vibrationJointSize = 0;
      
    for (auto& joint : jointList)
    {
      if (joint.getActuationType() == ActuationType::vibration)
      {
        vibrationJointSize++;
      }
    }
    return vibrationJointSize;
  }

  // IMU
  bool SenseGloveRobot::getImuRotation(Kinematics::Quat& outIMU)
  {
    return hapticglove->GetImuRotation(outIMU);
  }

  bool SenseGloveRobot::getNormalizedInput(std::vector<float>& out_normalizedValues)
  {
    out_normalizedValues.clear(); // Ensure it's always clean before use

    if (novaglovePtr)
    {
      if (novaglovePtr->GetNormalizedInput(out_normalizedValues))
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else if (nova2glovePtr)
    {
      if (nova2glovePtr->GetNormalizedInput(out_normalizedValues))
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    return false;
  }
  
  // Function to flatten vector of vectors of Vector3D
  Kinematics::Vect3D SenseGloveRobot::getHandPosition(int i)
  {
    // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
    // SG uses vector of vectors and ROS uses one long array 

    static const int TOTAL_FINGER_JOINT_INDEX = 19;

    if (i > TOTAL_FINGER_JOINT_INDEX)
    {
      jointPosition = {0.0, 0.0, 0.0};
    }    
    else if (senseglovePtr || novaglovePtr || nova2glovePtr)
    {
      jointPosition = handPose.GetJointPositions()[std::floor(i / 4)][i % 4];
    }
    return jointPosition;
  }

  Kinematics::Vect3D SenseGloveRobot::getFingerTip(int i)
  {
    // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
    // SG uses vector of vectors and ROS uses one long array
    static const int TOTAL_FINGER_JOINT_INDEX = 19;

    if (i > TOTAL_FINGER_JOINT_INDEX)
    {
      tipPositions = {0.0, 0.0, 0.0};
    }
    else if (senseglovePtr)
    {
      tipPositions = senseglovePose.CalculateFingertips(senseglovePtr->GetFingerThimbleOffsets())[i];
    }
    else if (novaglovePtr || nova2glovePtr)
    {
      tipPositions = handPose.GetJointPositions()[i][3];
    }
    return tipPositions;
  }

  void SenseGloveRobot::queueEffort(const std::vector<double>& effortCommand)
  {
    static const float MIN_TOTAL_FFB_THRESHOLD = 10.0;
    static const float STRAP_SAFETY_THRESHOLD = 10.0;

    effortLevels.assign(effortCommand.begin(), effortCommand.end());
    float totalEffort = std::accumulate(effortLevels.begin(), effortLevels.end(), 0.0);
    
    if(totalEffort > MIN_TOTAL_FFB_THRESHOLD)
    {
      if (senseglovePtr)
      {
        ffbQueued = senseglovePtr->QueueForceFeedbackLevels(effortLevels);
      }
      else if (novaglovePtr)
      {
        ffbQueued = novaglovePtr->QueueForceFeedbackLevels(effortLevels);
      }
      else if (nova2glovePtr)
      {
        ffbQueued = nova2glovePtr->QueueForceFeedbackLevels(effortLevels);
        squeezeLevel = std::min((effortLevels.back() * STRAP_SAFETY_THRESHOLD) / 100.0f, STRAP_SAFETY_THRESHOLD);
        squeezeQueued = nova2glovePtr->QueueSqueezeLevel(squeezeLevel);
      }
      effortQueued = ffbQueued || squeezeQueued;
    }
    else
    {
      effortQueued = false;
    }
  }

  void SenseGloveRobot::queueVibrations(const std::vector<double>& vibrationCommand)
  {
    static const float MIN_TOTAL_VIBRATION_THRESHOLD = 10.0;

    vibrationLevels.assign(vibrationCommand.begin(), vibrationCommand.end());
    float totalVibration = std::accumulate(vibrationLevels.begin(), vibrationLevels.end(), 0.0);

    if(totalVibration > MIN_TOTAL_VIBRATION_THRESHOLD)
    {
      if (senseglovePtr)
      {
        vibroQueued = senseglovePtr->QueueVibroLevels(vibrationLevels);
      }
      else if (novaglovePtr)
      {
        vibroQueued = novaglovePtr->QueueVibroLevels(vibrationLevels);
        thumperQueued = novaglovePtr->QueueWristLevel(vibrationLevels.back()); //Thumper
      }
      else if (nova2glovePtr)
      { // Issue with packet overload
        // TO-DO: Implement custom waveform service 

        // The below functions send a infinite loop custom waveform -> packet overloading
        // vibroQueued = nova2glovePtr->QueueVibroLevels(vibrationLevels);       
        // vibroQueued = nova2glovePtr->QueueVibroLevel(EHapticLocation::PalmIndexSide, vibrationLevels.back()-1);
        // vibroQueued = nova2glovePtr->QueueVibroLevel(EHapticLocation::PalmPinkySide, vibrationLevels.back()); 
        vibroQueued = false;

      }
      vibrationQueued = vibroQueued || thumperQueued;
    }
    else
    {
      vibrationQueued = false;
    }
  }

  void SenseGloveRobot::sendHaptics()
  {
    if (effortQueued)
    {
      this->hapticglove->SendHaptics();
      effortActive = true;
    }
    else if(effortActive)
    {
      this->hapticglove->StopHaptics();
      effortActive =  false;
    }

    if (vibrationQueued)
    {
      this->hapticglove->SendHaptics();
      vibrationActive = true;
    }
    else if(vibrationActive)
    {
      this->hapticglove->StopVibrations();
      vibrationActive = false;
    }
  }

  void SenseGloveRobot::stopHaptics()
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

  bool SenseGloveRobot::updateGloveData(const std::chrono::duration<double>& period)
  {

    static const int TOTAL_FINGER_JOINT_INDEX = 19;

    bool gloveUpdate = false;
    bool handUpdate = false;

    auto updateJointPositions = [&](const auto& poseAngles) {
      for (auto& joint : jointList)
      {
        int jointGroup = joint.jointIndex / 4; // Determine which finger
        int jointSubIndex = joint.jointIndex % 4; // Determine which joint within the finger

        if (joint.jointIndex > TOTAL_FINGER_JOINT_INDEX) 
        {
          joint.position = 0.0;
        }
        else
        {
          joint.position = (jointSubIndex == 0) 
              ? -poseAngles[jointGroup][0].GetZ() 
              : poseAngles[jointGroup][jointSubIndex - 1].GetY();
        }
      }
    };

    if (senseglovePtr)
    {
      if (senseglovePtr->GetSensorData(sensegloveSensorData))
      {
        for (auto& joint : jointList)
        {
          int jointGroup = joint.jointIndex / 4;
          int jointSubIndex = joint.jointIndex % 4;

          joint.position = sensegloveSensorData.GetSensorAngles()[jointGroup][jointSubIndex];
          double intermediateVelocity = sensegloveSensorData.GetSensorAngles()[jointGroup][jointSubIndex] - joint.velocity;
          joint.velocity = (intermediateVelocity != 0.0 && period.count() != 0.0) ? (intermediateVelocity / period.count()) : 0.0;
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
        updateJointPositions(handPoseAngles);
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
        updateJointPositions(handPoseAngles);
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
