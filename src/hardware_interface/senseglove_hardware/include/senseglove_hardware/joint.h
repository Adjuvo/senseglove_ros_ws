// Copyright 2020 Senseglove
#ifndef ROS_WORKSPACE_JOINT_H
#define ROS_WORKSPACE_JOINT_H

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <senseglove_hardware/actuation_mode.h>
#include <urdf/model.h>

#include <Fingers.h>
#include <SenseGlove.h>

namespace senseglove
{
class Joint
{
  friend class SenseGloveRobot;

public:
  /**
   * Initializes a Joint without a finger
   * Actuation will be disabled.
   */
  Joint(std::string name, int net_number);

  /**
   * Initializes a Joint without a finger
   */
  Joint(std::string name, int net_number, bool allow_actuation, ActuationMode mode);

  /**
   * Initializes a Joint with a Finger
   */
  Joint(std::string name, int net_number, bool allow_actuation, ActuationMode mode,
        std::unique_ptr<SGCore::Finger> finger);

  virtual ~Joint() noexcept = default;

  /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;

  /* Delete move assignment since string cannot be move assigned */
  Joint(Joint&&) = default;
  Joint& operator=(Joint&&) = delete;

  bool initialize();
  void prepareActuation();

  double readAngle();

  double getPosition() const;
  double getVelocity() const;
  double getTorque();
  //        SenseGloveState getSenseGloveState();

  std::string getName() const;
  int getTemperatureGESSlaveIndex() const;
  int getIMotionCubeSlaveIndex() const;
  int getNetNumber() const;

  ActuationMode getActuationMode() const;

  bool canActuate() const;
  bool receivedDataUpdate();
  void setAllowActuation(bool allow_actuation);

  /** @brief Override comparison operator */
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.name_ == rhs.name_ && lhs.finger_ == rhs.finger_ &&  // Mits Finger een comparison operator heeft
           lhs.allow_actuation_ == rhs.allow_actuation_ &&
           lhs.getActuationMode().getValue() == rhs.getActuationMode().getValue();
  }

  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    os << "name: " << joint.name_ << ", "
       << "ActuationMode: " << joint.getActuationMode().toString() << ", "
       << "allowActuation: " << joint.allow_actuation_;  // << ", "
    // << "finger: " << finger_; // Mits finger een ostream operator heeft.
    return os;
  }

private:
  const std::string name_;
  const int joint_index_;
  bool allow_actuation_ = false;
  ActuationMode actuation_mode_;

  double position_ = 0.0;
  double velocity_ = 0.0;
  std::unique_ptr<SGCore::Finger> finger_ = nullptr;
};

}  // namespace senseglove

#endif  // ROS_WORKSPACE_JOINT_H
