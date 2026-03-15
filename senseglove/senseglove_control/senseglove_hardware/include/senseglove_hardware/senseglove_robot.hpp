#ifndef SENSEGLOVE_HARDWARE_SENSEGLOVE_ROBOT_HPP
#define SENSEGLOVE_HARDWARE_SENSEGLOVE_ROBOT_HPP

#include <realtime_tools/realtime_buffer.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <senseglove_hardware/joint.hpp>

// SenseGlove API headers
#include <BasicHandModel.hpp>
#include <CustomWaveform.hpp>
#include <DeviceList.hpp>
#include <HandPose.hpp>
#include <Nova2Glove.hpp>
#include <Nova2GloveSensorData.hpp>
#include <NovaGlove.hpp>
#include <NovaGloveSensorData.hpp>
#include <Quat.hpp>
#include <SenseGlove.hpp>
#include <SenseGlovePose.hpp>
#include <SenseGloveSensorData.hpp>
#include <Vect3D.hpp>

namespace SGHardware
{

// Custom waveform command for one Nova 2 LRA motor
struct WaveformCmd
{
  SGCore::CustomWaveform waveform{0.5f, 0.2f, 180.0f};
  SGCore::Nova::ENova2VibroMotor motor{SGCore::Nova::ENova2VibroMotor::ThumbFingertip};
};

class SenseGloveRobot
{
public:
  using iterator = std::vector<Joint>::iterator;
  using const_iterator = std::vector<Joint>::const_iterator;

  enum class GloveType
  {
    SenseGlove,
    Nova,
    Nova2,
    Unknown
  };

  static constexpr int TOTAL_FINGER_JOINTS = 19;
  static constexpr int NUM_FINGERS = 5;
  static constexpr int JOINTS_PER_FINGER = 4;
  static constexpr int DISTAL_JOINT_INDEX = 3;

  static constexpr float MIN_TOTAL_FFB_THRESHOLD = 10.0f;
  static constexpr float MIN_TOTAL_VIBRATION_THRESHOLD = 10.0f;
  static constexpr float STRAP_SAFETY_THRESHOLD = 10.0f;

  // Construct a SenseGlove robot
  SenseGloveRobot(std::shared_ptr<SGCore::HapticGlove> hapticglove,
                  std::vector<Joint> jointList,
                  std::shared_ptr<urdf::Model> urdfModel,
                  const std::string& serial,
                  bool isRight);

  ~SenseGloveRobot();

  // Non-copyable, movable
  SenseGloveRobot(const SenseGloveRobot&) = delete;
  SenseGloveRobot& operator=(const SenseGloveRobot&) = delete;

  SenseGloveRobot(SenseGloveRobot&&) noexcept = default;
  SenseGloveRobot& operator=(SenseGloveRobot&&) = delete;

  // Identification
  const std::string& getRobotName() const noexcept
  {
    return robotName_;
  }
  const std::string& getSerial() const noexcept
  {
    return serial_;
  }
  const std::string& getHandedness() const noexcept
  {
    return handedness_;
  }
  SGCore::EDeviceType getDeviceType() const noexcept
  {
    return deviceType_;
  }
  GloveType getGloveType() const noexcept
  {
    return gloveType_;
  }
  bool isRight() const noexcept
  {
    return isRight_;
  }

  // Joint Access
  Joint& getJoint(const std::string& jointName);
  const Joint& getJoint(const std::string& jointName) const;

  Joint& getJoint(size_t index);
  const Joint& getJoint(size_t index) const;

  size_t getJointSize() const noexcept
  {
    return jointList_.size();
  }
  size_t getPositionJointSize() const noexcept
  {
    return positionJointCount_;
  }
  size_t getEffortJointSize() const noexcept
  {
    return effortJointCount_;
  }
  size_t getVibrationJointSize() const noexcept
  {
    return vibrationJointCount_;
  }

  // Iterator Access
  size_t size() const noexcept
  {
    return jointList_.size();
  }
  iterator begin() noexcept
  {
    return jointList_.begin();
  }
  iterator end() noexcept
  {
    return jointList_.end();
  }
  const_iterator begin() const noexcept
  {
    return jointList_.begin();
  }
  const_iterator end() const noexcept
  {
    return jointList_.end();
  }

  // Pose Data
  SGCore::Kinematics::Vect3D getHandPosition(int jointIndex) const;
  SGCore::Kinematics::Vect3D getFingerTip(int fingerIndex) const;
  bool getImuRotation(SGCore::Kinematics::Quat& outIMU) const;
  const std::shared_ptr<urdf::Model>& getUrdf() const noexcept
  {
    return urdfModel_;
  }

  // Haptics
  void queueEffort(const std::vector<double>& effortCommand);
  void queueVibrations(const std::vector<double>& vibrationCommand);
  void sendHaptics();
  void stopHaptics();

  // Nova 2 custom waveform
  // queueCustomWaveform() is called from the non-RT subscriber callback
  // processCustomWaveform() is called from write() in the RT control loop
  void queueCustomWaveform(SGCore::CustomWaveform& waveform, SGCore::Nova::ENova2VibroMotor motor);
  void processCustomWaveform();

  bool updateGloveData(const std::chrono::duration<double>& period);

  // Operator
  friend bool operator==(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
  {
    return lhs.serial_ == rhs.serial_ && lhs.isRight_ == rhs.isRight_;
  }

  friend bool operator!=(const SenseGloveRobot& lhs, const SenseGloveRobot& rhs)
  {
    return !(lhs == rhs);
  }

  friend std::ostream& operator<<(std::ostream& os, const SenseGloveRobot& robot)
  {
    os << "SenseGloveRobot{name=" << robot.robotName_ << ", serial=" << robot.serial_
       << ", hand=" << robot.handedness_ << ", joints=" << robot.jointList_.size() << "}";
    return os;
  }

private:
  GloveType gloveType_ = GloveType::Unknown;
  std::shared_ptr<SGCore::HapticGlove> hapticglove_;

  std::shared_ptr<SGCore::SG::SenseGlove> senseglovePtr_;
  std::shared_ptr<SGCore::Nova::NovaGlove> novaglovePtr_;
  std::shared_ptr<SGCore::Nova::Nova2Glove> nova2glovePtr_;

  SGCore::SG::SenseGloveSensorData sensegloveSensorData_;
  SGCore::SG::SenseGlovePose senseglovePose_;
  SGCore::Nova::NovaGloveSensorData novaSensorData_;
  SGCore::Nova::Nova2GloveSensorData nova2SensorData_;

  SGCore::HandPose handPose_;
  SGCore::Kinematics::BasicHandModel handModel_;
  std::vector<std::vector<SGCore::Kinematics::Vect3D>> handPoseAngles_;

  std::vector<Joint> jointList_;
  std::unordered_map<std::string, size_t> jointMap_;
  std::shared_ptr<urdf::Model> urdfModel_;

  size_t positionJointCount_ = 0;
  size_t effortJointCount_ = 0;
  size_t vibrationJointCount_ = 0;

  const std::string serial_;
  const std::string robotName_;
  const std::string handedness_;
  const SGCore::EDeviceType deviceType_;
  const bool isRight_;

  std::vector<float> effortLevels_;
  std::vector<float> vibrationLevels_;
  float squeezeLevel_ = 0.0f;

  bool effortQueued_ = false;
  bool vibrationQueued_ = false;
  bool effortActive_ = false;
  bool vibrationActive_ = false;

  // Nova 2 waveform RT buffer
  realtime_tools::RealtimeBuffer<WaveformCmd> waveform_buffer_;
  std::atomic<bool> waveform_pending_{false};

  void computeJointCounts();
  void updateJointPositions(const std::vector<std::vector<SGCore::Kinematics::Vect3D>>& poseAngles);
  void updateJointVelocities(double dt);
};

}  // namespace SGHardware

#endif  // SENSEGLOVE_HARDWARE_SENSEGLOVE_ROBOT_HPP