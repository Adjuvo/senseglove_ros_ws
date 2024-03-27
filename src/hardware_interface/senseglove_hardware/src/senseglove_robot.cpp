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
    else if (novaglovePtr != nullptr) 
    {
      if (i % 3 == 2) {jointPosition = handPose.GetJointPositions()[std::floor(i / 3)][3];} 
      if (i % 3 == 1) {jointPosition = handPose.GetJointPositions()[std::floor(i / 3)][1];}  
      else {jointPosition = handPose.GetJointPositions()[std::floor(i / 3)][0];}
    }
    else if (nova2glovePtr != nullptr)
    {
      if (i % 2 == 0) {jointPosition = handPose.GetJointPositions()[std::floor(i / 2)][3];}   
      else {jointPosition = handPose.GetJointPositions()[std::floor(i / 2)][1];}  
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
    else if (novaglovePtr != nullptr) 
    {
      tipPositions = handPose.GetJointPositions()[i][3]; 
    }
    else if (nova2glovePtr != nullptr)
    {
      tipPositions = handPose.GetJointPositions()[i][3];    
    }   

    return tipPositions;
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
    else if (novaglovePtr != nullptr)
    {
      handPoseAngles = handPose.GetHandAngles();
      if (!handPoseAngles.empty())
      {
        for (auto& joint : jointList)
        {
          if (joint.jointIndex % 3 == 0) {joint.position = handPoseAngles[std::floor(joint.jointIndex / 3)][0].GetZ();}
          else {joint.position = handPoseAngles[std::floor(joint.jointIndex / 3)][joint.jointIndex % 3].GetY();}
        }
      }
   
      if (!novaglovePtr->GetSensorData(novaSensorData)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated glove pose data");}
      else {gloveUpdate = true;}

      if (!novaglovePtr->GetHandPose(this->handModel, this->handPose)) {ROS_DEBUG_THROTTLE(2, "Unsuccessfully updated hand pose data");}
      else {handUpdate = true;}
    }

    else if (nova2glovePtr != nullptr)
    {
      handPoseAngles = handPose.GetHandAngles();
      if (!handPoseAngles.empty())
      {
        for (auto& joint : jointList)
        {
          if (joint.jointIndex % 3 == 0) {joint.position = handPoseAngles[std::floor(joint.jointIndex / 3)][0].GetZ();}
          else {joint.position = handPoseAngles[std::floor(joint.jointIndex / 3)][joint.jointIndex % 3].GetY();}
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
