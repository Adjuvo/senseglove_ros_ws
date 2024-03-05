/**
 * @file joint.h
 *
 * @brief A class to represent a joint in the Senseglove.
 *
 * @section LICENSE
 * Copyright (c) 2020 - 2024 SenseGlove
 *
 * @section AUTHOR
 * - Rogier
 * - Akshay Radhamohan Menon <akshay@senseglove.com>
 */


#ifndef ROS_WORKSPACE_JOINT_H
#define ROS_WORKSPACE_JOINT_H

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <urdf/model.h>
#include <senseglove_hardware/actuation_mode.h>

#include "Fingers.hpp"
#include "SenseGlove.hpp"


namespace senseglove
{
  class Joint
  {
    friend class SenseGloveRobot;

  public:
    // <summary> Initializes a Joint without a finger. Actuation is disabled by default. </summary>
    Joint(std::string name, int joint_index);

    // <summary> Initializes a Joint without a finger, with actuation options </summary>
    Joint(std::string name, int joint_index, bool allow_actuation, ActuationMode mode);

    // <summary> Initializes a Joint with a specific SG finger, with actuation options </summary>
    Joint(std::string name, int joint_index, bool allow_actuation, ActuationMode mode, std::unique_ptr<SGCore::EFinger> finger);

    /// <summary> Destructor defined default, ensuring proper destruction of derived classes </summary>
    virtual ~Joint() noexcept = default;

    // <summary>  Delete copy constructor & assignment operator since the unique_ptr cannot be copied </summary>
    Joint(const Joint&) = delete;
    Joint& operator=(const Joint&) = delete;

    // <summary> Delete move assignment since string cannot be move assigned </summary>
    Joint(Joint&&) = default;
    Joint& operator=(Joint&&) = delete;

    // --------------------------------------------------------------------------------------
    // Joint Methods

    // <summary> Initializes the joint </summary>
    bool initialize();

    // <summary> Gets the name of the joint </summary>
    std::string getName() const;

    // <summary> Gets the index of the joint </summary>
    int getIndex() const;
    
    // <summary> Reads the angle of the joint </summary>
    double readAngle();

    // <summary> Gets the position of the joint </summary>
    double getPosition() const;

    // <summary> Gets the velocity of the joint </summary>
    double getVelocity() const;

    // <summary> Gets the torque of the joint </summary>
    double getTorque();

    // <summary> Gets the actuation mode of the joint </summary>
    ActuationMode getActuationMode() const;

    // <summary> Prepares the joint for actuation </summary>
    void prepareActuation();

    // <summary> Checks if the joint can be actuated </summary>
    bool canActuate() const;

    // <summary> Setter to allow actuation </summary>
    void setAllowActuation(bool allow_actuation);

    // --------------------------------------------------------------------------------------
    // Friend Functions
    
    /// <summary> Checks if the names, finger objects, actuation permission flags, and actuation modes of two joints are equal </summary>
    friend bool operator==(const Joint& lhs, const Joint& rhs)
    {
      return lhs.name_ == rhs.name_ && lhs.finger_ == rhs.finger_ &&  // Provided Finger has a comparison operator
            lhs.allow_actuation_ == rhs.allow_actuation_ &&
            lhs.getActuationMode().getValue() == rhs.getActuationMode().getValue();
    }

    // <summary> Comparison operator </summary>
    friend bool operator!=(const Joint& lhs, const Joint& rhs)
    {
      return !(lhs == rhs);
    }

    // <summary> Override stream operator for clean printing </summary>
    friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
    {
      os << "name: " << joint.name_ << ", "
        << "ActuationMode: " << joint.getActuationMode().toString() << ", "
        << "allowActuation: " << joint.allow_actuation_;
      return os;
    }

  private:
    // <summary> Name of the joint </summary>
    const std::string name_;

    // <summary> Index of the joint </summary>
    const int joint_index_;

    // <summary> Flag indicating if actuation is allowed for the joint </summary>
    bool allow_actuation_ = false;

    // <summary> Mode of actuation of the joint </summary>
    ActuationMode actuation_mode_;

    // <summary> Position of the joint </summary>
    double position_ = 0.0;

    // <summary> Velocity of the joint </summary>
    double velocity_ = 0.0;

    // <summary> Unique pointer to a SG finger object associated with the joint </summary>
    std::unique_ptr<SGCore::EFinger> finger_ = nullptr;
  };

}  // namespace senseglove

#endif  // ROS_WORKSPACE_JOINT_H
