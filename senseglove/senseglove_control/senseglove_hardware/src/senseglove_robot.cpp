// Copyright (c) 2020 - 2026 SenseGlove

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>

#include <senseglove_hardware/senseglove_robot.hpp>

namespace SGHardware
{

SenseGloveRobot::SenseGloveRobot(std::shared_ptr<SGCore::HapticGlove> glove,
                                 std::vector<Joint> jointList,
                                 std::shared_ptr<urdf::Model> urdfModel,
                                 const std::string& serial,
                                 bool isRight)
  : hapticglove_(std::move(glove))
  , handModel_(SGCore::Kinematics::BasicHandModel::Default(isRight))
  , jointList_(std::move(jointList))
  , urdfModel_(std::move(urdfModel))
  , serial_(serial)
  , robotName_(hapticglove_ ? hapticglove_->GetDeviceId() : "Unknown")
  , handedness_(isRight ? "rh" : "lh")
  , deviceType_(hapticglove_ ? hapticglove_->GetDeviceType() : SGCore::EDeviceType::Unknown)
  , isRight_(isRight)
{
  // Initialize glove type
  senseglovePtr_ = std::dynamic_pointer_cast<SGCore::SG::SenseGlove>(hapticglove_);
  novaglovePtr_ = std::dynamic_pointer_cast<SGCore::Nova::NovaGlove>(hapticglove_);
  nova2glovePtr_ = std::dynamic_pointer_cast<SGCore::Nova::Nova2Glove>(hapticglove_);

  if (senseglovePtr_)
    gloveType_ = GloveType::SenseGlove;
  else if (nova2glovePtr_)
    gloveType_ = GloveType::Nova2;
  else if (novaglovePtr_)
    gloveType_ = GloveType::Nova;
  else
    gloveType_ = GloveType::Unknown;

  // Build joint name lookup map
  jointMap_.reserve(jointList_.size());
  for (size_t i = 0; i < jointList_.size(); ++i)
    jointMap_[jointList_[i].getName()] = i;

  computeJointCounts();

  effortLevels_.resize(effortJointCount_, 0.0f);
  vibrationLevels_.resize(vibrationJointCount_, 0.0f);

  RCLCPP_DEBUG(rclcpp::get_logger("senseglove.robot"),
               "Created robot: %s (%s) with %zu joints [pos:%zu, ffb:%zu, vib:%zu]",
               robotName_.c_str(),
               handedness_.c_str(),
               jointList_.size(),
               positionJointCount_,
               effortJointCount_,
               vibrationJointCount_);
}

SenseGloveRobot::~SenseGloveRobot()
{
  if (hapticglove_)
  {
    try
    {
      hapticglove_->StopHaptics();
    }
    catch (...)
    {
    }
  }
}

void SenseGloveRobot::computeJointCounts()
{
  positionJointCount_ = 0;
  effortJointCount_ = 0;
  vibrationJointCount_ = 0;

  for (const auto& joint : jointList_)
  {
    const auto type = joint.getActuationType();
    if (type == ActuationType::brake || type == ActuationType::squeeze)
    {
      effortJointCount_++;
      positionJointCount_++;
    }
    else if (type == ActuationType::vibration)
    {
      vibrationJointCount_++;
    }
    else
    {
      positionJointCount_++;
    }
  }
}

Joint& SenseGloveRobot::getJoint(const std::string& jointName)
{
  auto it = jointMap_.find(jointName);
  if (it == jointMap_.end())
    throw std::out_of_range("Joint not found: " + jointName);
  return jointList_[it->second];
}

const Joint& SenseGloveRobot::getJoint(const std::string& jointName) const
{
  auto it = jointMap_.find(jointName);
  if (it == jointMap_.end())
    throw std::out_of_range("Joint not found: " + jointName);
  return jointList_[it->second];
}

Joint& SenseGloveRobot::getJoint(size_t index)
{
  if (index >= jointList_.size())
    throw std::out_of_range("Joint index out of range: " + std::to_string(index));
  return jointList_[index];
}

const Joint& SenseGloveRobot::getJoint(size_t index) const
{
  if (index >= jointList_.size())
    throw std::out_of_range("Joint index out of range: " + std::to_string(index));
  return jointList_[index];
}

SGCore::Kinematics::Vect3D SenseGloveRobot::getHandPosition(int jointIndex) const
{
  SGCore::Kinematics::Vect3D position{0.0, 0.0, 0.0};
  if (jointIndex < 0 || jointIndex >= TOTAL_FINGER_JOINTS)
    return position;
  if (gloveType_ == GloveType::Unknown)
    return position;

  const auto& poseVec = handPose_.GetJointPositions();
  const size_t fingerIndex = static_cast<size_t>(jointIndex) / JOINTS_PER_FINGER;
  const size_t jointSubIndex = static_cast<size_t>(jointIndex) % JOINTS_PER_FINGER;

  if (fingerIndex < poseVec.size() && jointSubIndex < poseVec[fingerIndex].size())
    position = poseVec[fingerIndex][jointSubIndex];

  return position;
}

SGCore::Kinematics::Vect3D SenseGloveRobot::getFingerTip(int fingerIndex) const
{
  SGCore::Kinematics::Vect3D tipPosition{0.0, 0.0, 0.0};
  if (fingerIndex < 0 || fingerIndex >= NUM_FINGERS)
    return tipPosition;
  if (gloveType_ == GloveType::Unknown)
    return tipPosition;

  if (gloveType_ == GloveType::SenseGlove && senseglovePtr_)
  {
    const auto offsets = senseglovePtr_->GetFingerThimbleOffsets();
    const auto tips = senseglovePose_.CalculateFingertips(offsets);
    if (static_cast<size_t>(fingerIndex) < tips.size())
      return tips[static_cast<size_t>(fingerIndex)];
  }

  const auto& poseVec = handPose_.GetJointPositions();
  if (static_cast<size_t>(fingerIndex) < poseVec.size() &&
      DISTAL_JOINT_INDEX < poseVec[fingerIndex].size())
    tipPosition = poseVec[fingerIndex][DISTAL_JOINT_INDEX];

  return tipPosition;
}

bool SenseGloveRobot::getImuRotation(SGCore::Kinematics::Quat& outIMU) const
{
  if (!hapticglove_)
    return false;
  return hapticglove_->GetImuRotation(outIMU);
}

void SenseGloveRobot::queueEffort(const std::vector<double>& effortCommand)
{
  const size_t copySize = std::min(effortCommand.size(), effortLevels_.size());
  for (size_t i = 0; i < copySize; ++i)
    effortLevels_[i] = static_cast<float>(effortCommand[i]);
  for (size_t i = copySize; i < effortLevels_.size(); ++i)
    effortLevels_[i] = 0.0f;

  float totalEffort = 0.0f;
  for (size_t i = 0; i < copySize; ++i)
    totalEffort += std::abs(effortLevels_[i]);

  effortQueued_ = false;
  if (totalEffort > MIN_TOTAL_FFB_THRESHOLD)
  {
    bool ffbQueued = false;
    bool squeezeQueued = false;

    switch (gloveType_)
    {
      case GloveType::SenseGlove:
        ffbQueued = senseglovePtr_->QueueForceFeedbackLevels(effortLevels_);
        break;
      case GloveType::Nova:
        ffbQueued = novaglovePtr_->QueueForceFeedbackLevels(effortLevels_);
        break;
      case GloveType::Nova2:
        squeezeLevel_ = std::min((effortLevels_.back() * STRAP_SAFETY_THRESHOLD) / 100.0f,
                                 STRAP_SAFETY_THRESHOLD);
        ffbQueued = nova2glovePtr_->QueueForceFeedbackLevels(effortLevels_);
        squeezeQueued = nova2glovePtr_->QueueSqueezeLevel(squeezeLevel_);
        break;
      default:
        break;
    }
    effortQueued_ = ffbQueued || squeezeQueued;
  }
}

void SenseGloveRobot::queueVibrations(const std::vector<double>& vibrationCommand)
{
  const size_t copySize = std::min(vibrationCommand.size(), vibrationLevels_.size());
  for (size_t i = 0; i < copySize; ++i)
    vibrationLevels_[i] = static_cast<float>(vibrationCommand[i]);
  for (size_t i = copySize; i < vibrationLevels_.size(); ++i)
    vibrationLevels_[i] = 0.0f;

  float totalVibration = 0.0f;
  for (size_t i = 0; i < copySize; ++i)
    totalVibration += std::abs(vibrationLevels_[i]);

  vibrationQueued_ = false;
  if (totalVibration > MIN_TOTAL_VIBRATION_THRESHOLD)
  {
    bool vibroQueued = false;
    bool thumperQueued = false;

    switch (gloveType_)
    {
      case GloveType::SenseGlove:
        vibroQueued = senseglovePtr_->QueueVibroLevels(vibrationLevels_);
        break;
      case GloveType::Nova:
        vibroQueued = novaglovePtr_->QueueVibroLevels(vibrationLevels_);
        thumperQueued = novaglovePtr_->QueueWristLevel(vibrationLevels_.back());
        break;
      default:
        break;
    }
    vibrationQueued_ = vibroQueued || thumperQueued;
  }
}

void SenseGloveRobot::sendHaptics()
{
  bool anyQueued = effortQueued_ || vibrationQueued_;
  bool anyActive = effortActive_ || vibrationActive_;

  if (anyQueued)
  {
    hapticglove_->SendHaptics();
    effortActive_ = effortQueued_;
    vibrationActive_ = vibrationQueued_;
  }
  else if (anyActive)
  {
    hapticglove_->StopHaptics();
    hapticglove_->StopVibrations();
    effortActive_ = false;
    vibrationActive_ = false;
  }
}

void SenseGloveRobot::queueCustomWaveform(SGCore::CustomWaveform& waveform,
                                          SGCore::Nova::ENova2VibroMotor motor)
{
  if (gloveType_ != GloveType::Nova2 || !nova2glovePtr_)
  {
    RCLCPP_WARN_ONCE(rclcpp::get_logger("senseglove.robot"),
                     "queueCustomWaveform called on non-Nova2 glove: ignored");
    return;
  }

  waveform_buffer_.writeFromNonRT(WaveformCmd{waveform, motor});
  waveform_pending_.store(true, std::memory_order_release);
}

void SenseGloveRobot::processCustomWaveform()
{
  if (gloveType_ != GloveType::Nova2 || !nova2glovePtr_)
    return;

  if (!waveform_pending_.load(std::memory_order_acquire))
    return;

  WaveformCmd* cmd = waveform_buffer_.readFromRT();
  nova2glovePtr_->SendCustomWaveform_Nova2(cmd->waveform, cmd->motor);
  waveform_pending_.store(false, std::memory_order_release);
}

void SenseGloveRobot::stopHaptics()
{
  if (hapticglove_)
  {
    hapticglove_->StopHaptics();
    hapticglove_->StopVibrations();
  }
  effortActive_ = false;
  vibrationActive_ = false;
  effortQueued_ = false;
  vibrationQueued_ = false;
}

void SenseGloveRobot::updateJointPositions(
  const std::vector<std::vector<SGCore::Kinematics::Vect3D>>& poseAngles)
{
  for (auto& joint : jointList_)
  {
    const int idx = joint.getIndex();
    if (idx < 0 || idx >= TOTAL_FINGER_JOINTS)
    {
      joint.position_ = 0.0;
      continue;
    }

    const int fingerIndex = idx / JOINTS_PER_FINGER;
    const int jointSubIndex = idx % JOINTS_PER_FINGER;

    if (static_cast<size_t>(fingerIndex) >= poseAngles.size())
    {
      joint.position_ = 0.0;
      continue;
    }

    if (jointSubIndex == 0)
      joint.position_ = -poseAngles[fingerIndex][0].GetZ();
    else if (static_cast<size_t>(jointSubIndex - 1) < poseAngles[fingerIndex].size())
      joint.position_ = poseAngles[fingerIndex][jointSubIndex - 1].GetY();
  }
}

void SenseGloveRobot::updateJointVelocities(double dt)
{
  for (auto& joint : jointList_)
  {
    if (dt > 0.0)
      joint.velocity_ = (joint.position_ - joint.prevPosition_) / dt;
    else
      joint.velocity_ = 0.0;
    joint.prevPosition_ = joint.position_;
  }
}

bool SenseGloveRobot::updateGloveData(const std::chrono::duration<double>& period)
{
  auto logger = rclcpp::get_logger("senseglove.robot");
  const double dt = period.count();

  bool gloveUpdate = false;
  bool handUpdate = false;

  switch (gloveType_)
  {
    case GloveType::SenseGlove:
      if (senseglovePtr_->GetSensorData(sensegloveSensorData_))
      {
        gloveUpdate = senseglovePtr_->GetGlovePose(senseglovePose_);
        handUpdate = senseglovePtr_->GetHandPose(handModel_, handPose_);
        if (gloveUpdate && handUpdate)
        {
          const auto& sensorAngles = sensegloveSensorData_.GetSensorAngles();
          for (auto& joint : jointList_)
          {
            const int idx = joint.getIndex();
            if (idx < 0)
              continue;
            const int fingerIndex = idx / JOINTS_PER_FINGER;
            const int jointSubIndex = idx % JOINTS_PER_FINGER;
            if (static_cast<size_t>(fingerIndex) < sensorAngles.size() &&
                static_cast<size_t>(jointSubIndex) < sensorAngles[fingerIndex].size())
              joint.position_ = sensorAngles[fingerIndex][jointSubIndex];
          }
          updateJointVelocities(dt);
        }
      }
      break;

    case GloveType::Nova:
      gloveUpdate = novaglovePtr_->GetSensorData(novaSensorData_);
      handUpdate = novaglovePtr_->GetHandPose(handModel_, handPose_);
      if (gloveUpdate && handUpdate)
      {
        handPoseAngles_ = handPose_.GetHandAngles();
        if (!handPoseAngles_.empty())
        {
          updateJointPositions(handPoseAngles_);
          updateJointVelocities(dt);
        }
      }
      break;

    case GloveType::Nova2:
      gloveUpdate = nova2glovePtr_->GetSensorData(nova2SensorData_);
      handUpdate = nova2glovePtr_->GetHandPose(handModel_, handPose_);
      if (gloveUpdate && handUpdate)
      {
        handPoseAngles_ = handPose_.GetHandAngles();
        if (!handPoseAngles_.empty())
        {
          updateJointPositions(handPoseAngles_);
          updateJointVelocities(dt);
        }
      }
      break;

    case GloveType::Unknown:
    default:
      RCLCPP_WARN_THROTTLE(
        logger, *rclcpp::Clock::make_shared(), 5'000, "Cannot update data: unknown glove type");
      break;
  }

  return gloveUpdate && handUpdate;
}

}  // namespace SGHardware