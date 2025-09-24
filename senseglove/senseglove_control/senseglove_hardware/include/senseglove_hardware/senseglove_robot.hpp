#ifndef ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
#define ROS_WORKSPACE_SENSEGLOVE_ROBOT_H

#include <vector>
#include <numeric>
#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <ostream>
#include <cmath>

#include <senseglove_hardware/joint.hpp>

// SenseGlove API headers
#include <SenseGlove.hpp>
#include <SenseGloveSensorData.hpp>
#include <SenseGlovePose.hpp>

#include <NovaGlove.hpp>
#include <NovaGloveSensorData.hpp>

#include <Nova2Glove.hpp>
#include <Nova2GloveSensorData.hpp>

#include <BasicHandModel.hpp>
#include <HandPose.hpp>
#include <DeviceList.hpp>
#include <Vect3D.hpp>
#include <Quat.hpp>

namespace SGHardware
{
  class SenseGloveRobot
  {   
  public:  
    using iterator = std::vector<Joint>::iterator;
    
    // Constructor: pass a shared HapticGlove pointer, joint list, URDF model, index, and handedness
    SenseGloveRobot(std::shared_ptr<HapticGlove> hapticglove, 
                    std::vector<Joint> jointList, 
                    urdf::Model urdfModel, 
                    int robotIndex, 
                    bool isRight);

    ~SenseGloveRobot();

    // Delete copy constructor/assignment since the unique_ptr cannot be copied
    SenseGloveRobot(SenseGloveRobot&) = delete;
    SenseGloveRobot& operator=(SenseGloveRobot&) = delete;

    // Delete move assignment since string cannot be move assigned
    SenseGloveRobot(SenseGloveRobot&&) = default;
    SenseGloveRobot& operator=(SenseGloveRobot&&) = delete;
    
    // Accessors
    std::string getRobotName() const;
    SGCore::EDeviceType getRobotType() const;
    int getRobotIndex() const;
    bool getRight() const;

    Joint& getJoint(const std::string jointName);
    Joint& getJoint(size_t index);

    size_t getJointSize() const;
    size_t getPositionJointSize() const;
    size_t getEffortJointSize() const;
    size_t getVibrationJointSize() const;

    SGCore::Kinematics::Vect3D getHandPosition(int i) const;
    SGCore::Kinematics::Vect3D getFingerTip(int i) const;

    bool getImuRotation(SGCore::Kinematics::Quat & outIMU) const;

    std::vector<float> effortLevels;
    std::vector<float> vibrationLevels;
    float squeezeLevel = 0.0f;

    // ros control works exclusively with doubles, but the sendHaptics function works with integers
    void queueEffort(const std::vector<double>& effortCommand);    
    void queueVibrations(const std::vector<double>& vibrationCommand);

    void sendHaptics();
    void stopHaptics();

    bool effortActive = false;
    bool vibrationActive = false;

    bool effortQueued = false;
    bool vibrationQueued = false;
    bool ffbQueued = false;
    bool squeezeQueued = false;
    bool vibroQueued = false;
    bool thumperQueued = false;

    size_t size() const;    
    iterator begin();
    iterator end();

    const urdf::Model& getUrdf() const;

    bool updateGloveData(const std::chrono::duration<double>& period);

    // Override comparison operator
    friend bool operator==(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
    {
      if (lhs.jointList.size() != rhs.jointList.size()) {
        return false;
      }
      for (size_t i = 0; i < lhs.jointList.size(); ++i) {
        if (lhs.jointList[i] != rhs.jointList[i]) {
          return false;
        }
      }
      return true;
    }
    friend bool operator!=(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
    {
      return !(lhs == rhs);
    }
    friend std::ostream& operator<<(std::ostream& os, const SenseGloveRobot& robot)
    {
      for (const auto& j : robot.jointList) {
        os << j << "\n";
      }
      return os;
    }
  
    private:
    // DK1 Specific  
    SGCore::SG::SenseGloveSensorData sensegloveSensorData;
    SGCore::SG::SenseGlovePose senseglovePose;

    //Nova Specific
    SGCore::Nova::NovaGlove novaglove;    
    SGCore::Nova::NovaGloveSensorData novaSensorData;  

    // Nova2 Specific
    SGCore::Nova::Nova2Glove nova2glove;    
    SGCore::Nova::Nova2GloveSensorData nova2SensorData;

    // Shared
    std::shared_ptr<SGCore::HapticGlove> hapticglove;
    std::shared_ptr<SGCore::SG::SenseGlove> senseglovePtr;
    std::shared_ptr<SGCore::Nova::NovaGlove> novaglovePtr;
    std::shared_ptr<SGCore::Nova::Nova2Glove> nova2glovePtr;
    
    SGCore::HandPose handPose;
    SGCore::Kinematics::BasicHandModel handModel;
    SGCore::Kinematics::Vect3D jointPosition;
    SGCore::Kinematics::Vect3D tipPositions;  
    std::vector<std::vector<SGCore::Kinematics::Vect3D>> handPoseAngles;

    std::vector<Joint> jointList;
    std::unordered_map<std::string, size_t> jointMap;
    urdf::Model urdfModel;
    const std::string SenseGloveRobotName;
    const SGCore::EDeviceType deviceType;
    const int robotIndex;
    const bool isRight;
    bool isUpdated = false;

  };
}  // namespace SGHardware

#endif  // ROS_WORKSPACE_SENSEGLOVE_ROBOT_H
