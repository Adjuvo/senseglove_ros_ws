// Copyright (c) 2020 - 2025 SenseGlove

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <senseglove_hardware/senseglove_robot.hpp>

namespace SGHardware
{
  SenseGloveRobot::SenseGloveRobot(
    std::shared_ptr<HapticGlove> glove, 
    std::vector<Joint> jointList, 
    const urdf::Model urdfModel, 
    int robotIndex, 
    bool isRight)
    : hapticglove(std::move(glove))
    , handModel(SGCore::Kinematics::BasicHandModel::Default(isRight))
    , jointList(std::move(jointList))
    , urdfModel(std::move(urdfModel))
    , SenseGloveRobotName("senseglove/glove" + std::to_string(int((robotIndex) / 2)))
    , deviceType(this->hapticglove ? this->hapticglove->GetDeviceType() : SGCore::EDeviceType::Unknown)
    , robotIndex(robotIndex)
    , isRight(isRight)
    , isUpdated(false)
  {
    // Initialize specific pointers
    senseglovePtr = std::dynamic_pointer_cast<SGCore::SG::SenseGlove>(hapticglove);
    novaglovePtr  = std::dynamic_pointer_cast<SGCore::Nova::NovaGlove>(hapticglove);
    nova2glovePtr = std::dynamic_pointer_cast<SGCore::Nova::Nova2Glove>(hapticglove);

    for (size_t i = 0; i < this->jointList.size(); ++i)
    {
      jointMap[this->jointList[i].getName()] = i;
    }
  }

  std::string SenseGloveRobot::getRobotName() const
  {
    return this->SenseGloveRobotName;
  }

  SGCore::EDeviceType SenseGloveRobot::getRobotType() const
  {
    return this->deviceType;
  }

  int SenseGloveRobot::getRobotIndex() const
  {
    return this->robotIndex;
  }

  bool SenseGloveRobot::getRight() const
  {
    return hapticglove ? this->hapticglove->IsRight() : false;
  }

  Joint& SenseGloveRobot::getJoint(const ::std::string jointName)
  {
    auto it = jointMap.find(jointName);
    if (it != jointMap.end()) 
    {
      return jointList[it->second];
    }
    throw std::out_of_range("Could not find joint with name " + jointName);
  }

  Joint& SenseGloveRobot::getJoint(size_t index)
  {
    return this->jointList.at(index);
  }

  size_t SenseGloveRobot::getJointSize() const
  {
    return this->jointList.size();
  }

  size_t SenseGloveRobot::getEffortJointSize() const
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

  size_t SenseGloveRobot::getVibrationJointSize() const
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

  bool SenseGloveRobot::getImuRotation(SGCore::Kinematics::Quat& outIMU) const
  {
    return hapticglove->GetImuRotation(outIMU);
  }

  bool SenseGloveRobot::getNormalizedInput(std::vector<float>& out_normalizedValues) const
  {
    out_normalizedValues.clear();
    if (novaglovePtr)
    {
      return novaglovePtr->GetNormalizedInput(out_normalizedValues);
    }
    else if (nova2glovePtr)
    {
      return nova2glovePtr->GetNormalizedInput(out_normalizedValues);
    }
    return false;
  }
  
  // Function to flatten vector of vectors of Vector3D
  SGCore::Kinematics::Vect3D SenseGloveRobot::getHandPosition(int i) const
  {
    // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
    // SG uses vector of vectors and ROS uses one long array 

    static const int TOTAL_FINGER_JOINT_INDEX = 19;
    SGCore::Kinematics::Vect3D jointPosition{0.0, 0.0, 0.0};

    if (i <= TOTAL_FINGER_JOINT_INDEX && (senseglovePtr || novaglovePtr || nova2glovePtr))
    {
      const auto & poseVec = handPose.GetJointPositions();
      size_t jointGroup = static_cast<size_t>(i) / 4;
      size_t jointSubIndex = static_cast<size_t>(i) % 4;
      if (jointGroup < poseVec.size() && jointSubIndex < poseVec[jointGroup].size())
      {
        jointPosition = poseVec[jointGroup][jointSubIndex];
      }
    }
    return jointPosition;
  }

  SGCore::Kinematics::Vect3D SenseGloveRobot::getFingerTip(int i) const
  {
    static const int TOTAL_FINGER_JOINT_INDEX = 19;
    SGCore::Kinematics::Vect3D tipPosition{0.0, 0.0, 0.0};

    if (i > TOTAL_FINGER_JOINT_INDEX)
    {
      return tipPosition;
    }
    if (senseglovePtr)
    {
      auto offsets = senseglovePtr->GetFingerThimbleOffsets();
      auto tf = senseglovePose.CalculateFingertips(offsets);
      if (static_cast<size_t>(i) < tf.size())
      {
        tipPosition = tf[i];
      }
    }
    else if (novaglovePtr || nova2glovePtr)
    {
      const auto & poseVec = handPose.GetJointPositions();
      if (static_cast<size_t>(i) < poseVec.size() && poseVec[i].size() > 3)
      {
        tipPosition = poseVec[i][3];
      }
    }
    return tipPosition;
  }

  void SenseGloveRobot::queueEffort(const std::vector<double>& effortCommand)
  {
    static const float MIN_TOTAL_FFB_THRESHOLD = 10.0f;
    static const float STRAP_SAFETY_THRESHOLD = 10.0f;

    effortLevels.assign(effortCommand.begin(), effortCommand.end());
    float totalEffort = std::accumulate(effortLevels.begin(), effortLevels.end(), 0.0f);
    ffbQueued = false;
    squeezeQueued = false;
    
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
    vibroQueued = false;
    thumperQueued = false;

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
        // TODO: Implement custom waveform service 

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
      hapticglove->SendHaptics();
      effortActive = true;
    }
    else if(effortActive)
    {
      hapticglove->StopHaptics();
      effortActive =  false;
    }

    if (vibrationQueued)
    {
      hapticglove->SendHaptics();
      vibrationActive = true;
    }
    else if(vibrationActive)
    {
      hapticglove->StopVibrations();
      vibrationActive = false;
    }
  }

  void SenseGloveRobot::stopHaptics()
  {
    hapticglove->StopHaptics();
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
      
      if (!senseglovePtr->GetGlovePose(senseglovePose)) { RCUTILS_LOG_DEBUG_NAMED("senseglove.robot", "Unsuccessfully updated glove pose data"); }
      else { gloveUpdate = true; }

      if (!senseglovePtr->GetHandPose(this->handModel, this->handPose)) { RCUTILS_LOG_DEBUG_NAMED("senseglove.robot", "Unsuccessfully updated hand pose data"); }
      else { handUpdate = true; }
    }
    else if (novaglovePtr != nullptr)
    {
      handPoseAngles = handPose.GetHandAngles();
      if (!handPoseAngles.empty())
      {
        updateJointPositions(handPoseAngles);
      }

      if (!novaglovePtr->GetSensorData(novaSensorData)) { RCUTILS_LOG_DEBUG_NAMED("senseglove.robot", "Unsuccessfully updated glove pose data"); }
      else { gloveUpdate = true; }

      if (!novaglovePtr->GetHandPose(this->handModel, this->handPose)) { RCUTILS_LOG_DEBUG_NAMED("senseglove.robot", "Unsuccessfully updated hand pose data"); }
      else { handUpdate = true; }
    }

    else if (nova2glovePtr != nullptr)
    {
      handPoseAngles = handPose.GetHandAngles();
      if (!handPoseAngles.empty())
      {
        updateJointPositions(handPoseAngles);
      }

      if (!nova2glovePtr->GetSensorData(nova2SensorData)) { RCUTILS_LOG_DEBUG_NAMED("senseglove.robot", "Unsuccessfully updated glove pose data"); }
      else { gloveUpdate = true; }

      if (!nova2glovePtr->GetHandPose(this->handModel, this->handPose)) { RCUTILS_LOG_DEBUG_NAMED("senseglove.robot", "Unsuccessfully updated hand pose data"); }
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
