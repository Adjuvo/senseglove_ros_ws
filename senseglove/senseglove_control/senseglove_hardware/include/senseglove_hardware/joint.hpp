#ifndef SENSEGLOVE_HARDWARE_JOINT_HPP
#define SENSEGLOVE_HARDWARE_JOINT_HPP

#include <urdf/model.h>

#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include <Fingers.hpp>
#include <SenseGlove.hpp>
#include <senseglove_hardware/actuation_mode.hpp>
#include <senseglove_hardware/actuation_type.hpp>

namespace SGHardware
{
class Joint
{
  friend class SenseGloveRobot;

public:
  // Initializes a Joint without a finger. Actuation is disabled by default
  Joint(std::string jointName, int jointIndex);

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
        std::unique_ptr<SGCore::EFinger> finger);

  virtual ~Joint() noexcept = default;

  //  Delete copy constructor & assignment operator since the unique_ptr cannot be copied
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;

  // Delete move assignment since string cannot be move assigned
  Joint(Joint&&) noexcept = default;
  Joint& operator=(Joint&&) noexcept = default;

  // Accessors
  // Returns the joint name
  const std::string& getName() const noexcept
  {
    return jointName_;
  }

  // Returns the joint index
  int getIndex() const noexcept
  {
    return jointIndex_;
  }

  // Returns current joint position in radians
  double getPosition() const noexcept
  {
    return position_;
  }

  // Returns current joint velocity in rad/s
  double getVelocity() const noexcept
  {
    return velocity_;
  }

  // Returns computed joint torque
  double getTorque() const noexcept
  {
    return 0.0;
  }

  // Returns the actuation type (brake, squeeze, vibration, none)
  ActuationType getActuationType() const noexcept
  {
    return actuationType_;
  }

  // Returns the actuation mode (effort, position)
  ActuationMode getActuationMode() const noexcept
  {
    return actuationMode_;
  }

  // Returns true if this joint can send haptic commands
  bool canActuate() const noexcept
  {
    return allowActuation_;
  }

  // Enable or disable actuation for this joint
  void setAllowActuation(bool allow) noexcept
  {
    allowActuation_ = allow;
  }

  // Prepare joint for actuation
  void prepareActuation();

  // Friend Functions
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.jointName_ == rhs.jointName_ && lhs.jointIndex_ == rhs.jointIndex_ &&
           lhs.allowActuation_ == rhs.allowActuation_ &&
           lhs.actuationMode_.getValue() == rhs.actuationMode_.getValue() &&
           lhs.actuationType_.getValue() == rhs.actuationType_.getValue();
  }

  // Comparison operator
  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }

  // Override stream operator for clean printing
  friend std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    os << "Joint{name=" << joint.jointName_ << ", index=" << joint.jointIndex_
       << ", type=" << joint.actuationType_.toString()
       << ", mode=" << joint.actuationMode_.toString()
       << ", actuatable=" << (joint.allowActuation_ ? "yes" : "no") << "}";
    return os;
  }

private:
  const std::string jointName_;
  const int jointIndex_;

  ActuationType actuationType_{ActuationType::brake};
  ActuationMode actuationMode_{ActuationMode::effort};
  bool allowActuation_ = false;

  double position_ = 0.0;
  double prevPosition_ = 0.0;
  double velocity_ = 0.0;

  std::unique_ptr<SGCore::EFinger> finger_ = nullptr;
};

}  // namespace SGHardware

#endif  // SENSEGLOVE_HARDWARE_JOINT_HPP