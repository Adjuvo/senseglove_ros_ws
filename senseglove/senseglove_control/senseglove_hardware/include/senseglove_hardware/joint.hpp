#ifndef ROS_WORKSPACE_JOINT_H
#define ROS_WORKSPACE_JOINT_H

#include <memory>
#include <string>
#include <utility>

#include <urdf/model.h>
#include <senseglove_hardware/actuation_mode.hpp>
#include <senseglove_hardware/actuation_type.hpp>

// SenseGlove API headers
#include <SenseGlove.hpp>
#include <Fingers.hpp>

using namespace SGCore;

namespace SGHardware
{
  class Joint
  {
    friend class SenseGloveRobot;

  public:
    // Initializes a Joint without a finger. Actuation is disabled by default.
    Joint(std::string jointName, 
          int jointIndex);

    // Initializes a Joint without a finger, with actuation options
    Joint(std::string jointName, 
          int jointIndex, 
          ActuationType actuationType, 
          ActuationMode actuationMode, 
          bool allowActuation);

    // Initializes a Joint with a specific SG finger, with actuation options
    Joint(std::string jointName, 
          int jointIndex, 
          ActuationType actuationType, 
          ActuationMode actuationMode, 
          bool allowActuation, 
          std::unique_ptr<EFinger> finger);

    virtual ~Joint() noexcept = default;

    //  Delete copy constructor & assignment operator since the unique_ptr cannot be copied
    Joint(const Joint&) = delete;
    Joint& operator=(const Joint&) = delete;

    // Delete move assignment since string cannot be move assigned
    Joint(Joint&&) = default;
    Joint& operator=(Joint&&) = delete;

    // --------------------------------------------------------------------------------------
    // Joint Methods

    // Initializes the joint
    bool initialize();

    // Gets the name of the joint
    std::string getName() const;

    // Gets the index of the joint
    int getIndex() const;
    
    // Reads the angle of the joint
    double readAngle();

    // Gets the position of the joint
    double getPosition() const;

    // Gets the velocity of the joint
    double getVelocity() const;

    // Gets the torque of the joint
    double getTorque() const;

    // Gets the actuation type of the joint
    ActuationType getActuationType() const;

    // Gets the actuation mode of the joint
    ActuationMode getActuationMode() const;

    // Prepares the joint for actuation
    void prepareActuation();

    // Checks if the joint can be actuated
    bool canActuate() const;

    // Setter to allow actuation
    void setAllowActuation(bool allow_actuation);

    // --------------------------------------------------------------------------------------
    // Friend Functions
    
    /// Checks if the names, finger objects, actuation permission flags, and actuation modes of two joints are equal
    friend bool operator==(const Joint& lhs, const Joint& rhs)
    {
      return lhs.jointName == rhs.jointName
          && lhs.finger == rhs.finger 
          && lhs.allowActuation == rhs.allowActuation 
          && lhs.getActuationMode().getValue() == rhs.getActuationMode().getValue();
    }

    // Comparison operator
    friend bool operator!=(const Joint& lhs, const Joint& rhs)
    {
      return !(lhs == rhs);
    }

    // Override stream operator for clean printing
    friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
    {
      os << "name: " << joint.jointName << ", "
        << "ActuationMode: " << joint.getActuationMode().toString() << ", "
        << "allowActuation: " << (joint.allowActuation ? "true" : "false");
      return os;
    }

  private:
    // Name of the joint
    const std::string jointName;

    // Index of the joint
    const int jointIndex;

    // Type of actuation of the joint
    ActuationType actuationType;

    // Mode of actuation of the joint
    ActuationMode actuationMode;

    // Flag indicating if actuation is allowed for the joint
    bool allowActuation = false;

    // Position of the joint
    double position = 0.0;

    // Velocity of the joint
    double velocity = 0.0;

    // Unique pointer to a SG finger object associated with the joint
    std::unique_ptr<SGCore::EFinger> finger = nullptr;
  };

}  // namespace SGHardware

#endif  // ROS_WORKSPACE_JOINT_H
