// Copyright 2020 SenseGlove
#include <senseglove_hardware/joint.h>
#include <senseglove_hardware/senseglove_robot.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <limits>

#include <ros/ros.h>

namespace SGHardware
{
  SenseGloveRobot::SenseGloveRobot(std::shared_ptr<HapticGlove> glove, ::std::vector<Joint> jointList, urdf::Model urdfModel, int robotIndex, bool isRight)
    : hapticglove(glove)
    , handModel(Kinematics::BasicHandModel::Default(isRight))
    , jointList(std::move(jointList))
    , urdfModel(std::move(urdfModel))
    , SenseGloveRobotName(hapticglove->GetDeviceId() + "/" + std::to_string(int((robotIndex) / 2)))
    , deviceType(this->hapticglove->GetDeviceType())
    , robotIndex(robotIndex)
    , isUpdated(false)
  {
  }

  std::string SenseGloveRobot::getRobotName() const  
  {
    std::string modifiedRobotName = this->SenseGloveRobotName;
    std::replace_if(modifiedRobotName.begin(), modifiedRobotName.end(), 
                    [](unsigned char c) { return !std::isalnum(c) && c != '/'; }, '_');

    return modifiedRobotName;
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

  size_t SenseGloveRobot::getHandPoseSize()
  {
    switch (this->deviceType)
      {
        case EDeviceType::SenseGlove:
          return 20;
        case EDeviceType::Nova:
          return 15;
        case EDeviceType::Nova2:
          return 15;
        default:
          return 20;
      }
  }

  int SenseGloveRobot::getEffortJointSize()
  {
    switch (this->deviceType)
      {
        case EDeviceType::SenseGlove:
          return 9;
        case EDeviceType::Nova:
          return 5;
        case EDeviceType::Nova2:
          return 5;
        default:
          return 9;
      }
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
    else if (novaglovePtr != nullptr) 
    {
      jointPosition = handPose.GetJointPositions()[std::floor(i / 4)][i % 4];     
    }
    else if (nova2glovePtr != nullptr)
    {
      jointPosition = handPose.GetJointPositions()[std::floor(i / 4)][i % 4];    
    }    

    return jointPosition;

  }

  Kinematics::Vect3D SenseGloveRobot::getFingerTip(int i)
  {
    // Make sure to convert between the coordinate frame of the Senseglove and the one used in ROS
    // SG uses vector of vectors and ROS uses one long array
    if (senseglovePtr != nullptr) 
    {
      return senseglovePose.CalculateFingertips(senseglovePtr->GetFingerThimbleOffsets())[i];
    }
    else
    {
      return {0.0, 0.0, 0.0};
    }

  }

  void SenseGloveRobot::actuateEffort(std::vector<double> effortCommand)
  {
    if (DeviceList::SenseComRunning())  // check if the Sense Comm is running. If not, warn the end user.
    {
      std::vector<float> levels01(effortCommand.begin(), effortCommand.end());
      if (effortCommand[0] + effortCommand[1] + effortCommand[2] + effortCommand[3] + effortCommand[4] < 10.0)  // less than noticable ffb
      {
        this->hapticglove->StopHaptics();
      }
      else
      {
        this->hapticglove->QueueForceFeedbackLevels(levels01);
        this->hapticglove->SendHaptics();
      }
    }
  }

  void SenseGloveRobot::actuateEffort(double e_0, double e_1, double e_2, double e_3, double e_4)
  {
    std::vector<double> efforts = { e_0, e_1, e_2, e_3, e_4 };
    this->actuateEffort(efforts);
  }

  void SenseGloveRobot::actuateVibrations(std::vector<double> vibrationCommand)
  {
    std::vector<float> amplitudes(vibrationCommand.begin(), vibrationCommand.end());

    if (vibrationCommand[0] + vibrationCommand[1] + vibrationCommand[2] + vibrationCommand[3] + vibrationCommand[4] < 10.0)  // less than noticable buzz
    {

      this->hapticglove->StopVibrations();
    }
    else
    {
      this->hapticglove->QueueVibroLevels(amplitudes);
      this->hapticglove->SendHaptics();
    }
  }
  void SenseGloveRobot::actuateVibrations(double b_0, double b_1, double b_2, double b_3, double b_4)
  {
    std::vector<double> vibrationCommand = { b_0, b_1, b_2, b_3, b_4 };
    this->actuateVibrations(vibrationCommand);
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

    if(senseglovePtr != nullptr)
    {
      if (senseglovePtr->GetSensorData(sensegloveSensorData))  // If GetSensorData is true, we have sucesfully received data
      {
        for (auto& joint : jointList)
        {
          joint.position = sensegloveSensorData.GetSensorAngles()[joint.jointIndex / 4][joint.jointIndex % 4];
          double intermediateVelocity = (sensegloveSensorData.GetSensorAngles()[joint.jointIndex / 4][joint.jointIndex % 4] - joint.velocity);

          if (intermediateVelocity != 0.0 and period.toSec() != 0.0) {joint.velocity = intermediateVelocity / 1.0;}
          else { joint.velocity = 0.0; }
        }
      }

      if (!senseglovePtr->GetGlovePose(senseglovePose)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data");}
      else {gloveUpdate = true;}

      if (!senseglovePtr->GetHandPose(this->handModel, this->handPose)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data");}
      else {handUpdate = true;}
    }

    else if(novaglovePtr != nullptr)
    {
      if (novaglovePtr->GetSensorData(novaSensorData))  // If GetSensorData is true, we have sucesfully received data
      {
        std::vector<float> normalizedValues{novaglovePtr->ToNormalizedValues(novaSensorData)};

        for (size_t i = 0; i < 5; ++i)
        {
          handAngles.push_back(novaglovePtr->CalculateHandAngles(normalizedValues, interpolator)[i][0].GetZ());
          handAngles.push_back(novaglovePtr->CalculateHandAngles(normalizedValues, interpolator)[i][0].GetY());
        }

        for (auto& joint : jointList)
        { 
          joint.position = handAngles[joint.jointIndex];    
          double intermediateVelocity = joint.position - joint.velocity;

          if (intermediateVelocity != 0.0 and period.toSec() != 0.0) {joint.velocity = intermediateVelocity / 1.0;}
          else { joint.velocity = 0.0; }
        }
      }

      if (!novaglovePtr->GetSensorData(novaSensorData)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data");}
      else {gloveUpdate = true;}

      if (!novaglovePtr->GetHandPose(this->handModel, this->handPose)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data");}
      else {handUpdate = true;}
    }

    else if(nova2glovePtr != nullptr)
    {
      if (nova2glovePtr->GetSensorData(nova2SensorData))  // If GetSensorData is true, we have sucesfully received data
      {
        std::vector<float> normalizedValues{nova2glovePtr->ToNormalizedValues(nova2SensorData)};

        for (size_t i = 0; i < 5; ++i)
        {
          handAngles.push_back(nova2glovePtr->CalculateHandAngles(normalizedValues, interpolator, false)[i][0].GetZ());
          handAngles.push_back(nova2glovePtr->CalculateHandAngles(normalizedValues, interpolator, false)[i][0].GetY());
        }

        for (auto& joint : jointList)
        { 
          joint.position = handAngles[joint.jointIndex];    
          double intermediateVelocity = joint.position - joint.velocity;

          if (intermediateVelocity != 0.0 and period.toSec() != 0.0) {joint.velocity = intermediateVelocity / 1.0;}
          else { joint.velocity = 0.0; }
        }
      }

      if (!novaglovePtr->GetSensorData(novaSensorData)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data");}
      else {gloveUpdate = true;}

      if (!novaglovePtr->GetHandPose(this->handModel, this->handPose)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data");}
      else {handUpdate = true;}
    }
    isUpdated |= (gloveUpdate and handUpdate);
    return isUpdated;
  }
  

  const urdf::Model& SenseGloveRobot::getUrdf() const
  {
    return this->urdfModel;
  }

}  // namespace SGHardware
