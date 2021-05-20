// Copyright 2020 SenseGlove.
#ifndef ROS_WORKSPACE_SENSEGLOVE_SETUP_H
#define ROS_WORKSPACE_SENSEGLOVE_SETUP_H

#include "senseglove_hardware/joint.h"
#include "SenseGlove.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

namespace senseglove
{
class SenseGloveSetup
{
private:
  ::std::vector<senseglove::SenseGloveRobot> sensegloves_;

public:
  using iterator = std::vector<senseglove::SenseGloveRobot>::iterator;

  SenseGloveSetup(senseglove::SenseGloveRobot sensegloves);

  SenseGloveSetup(std::vector<senseglove::SenseGloveRobot> sensegloves);

  ~SenseGloveSetup();

  /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
  SenseGloveSetup(SenseGloveSetup&) = delete;
  SenseGloveSetup& operator=(SenseGloveSetup&) = delete;

  /* Delete move assignment since string cannot be move assigned */
  SenseGloveSetup(SenseGloveSetup&&) = delete;
  SenseGloveSetup& operator=(SenseGloveSetup&&) = delete;

  void startCommunication(bool /*reset*/);

  void stopCommunication();

  bool isCommunicationOperational();

  SenseGloveRobot& getSenseGloveRobot(::std::string gloveName);

  SenseGloveRobot& getSenseGloveRobot(int index);

  size_t size() const;

  iterator begin();
  iterator end();

  const urdf::Model& getRobotUrdf(std::string glove_robot_name);

  /** @brief Override comparison operator */
  friend bool operator==(const SenseGloveSetup& lhs, const SenseGloveSetup& rhs)
  {
    if (lhs.sensegloves_.size() != rhs.sensegloves_.size())
    {
      return false;
    }
    for (unsigned int i = 0; i < lhs.sensegloves_.size(); i++)
    {
      const senseglove::SenseGloveRobot& lhsGlove = lhs.sensegloves_.at(i);
      const senseglove::SenseGloveRobot& rhsGlove = rhs.sensegloves_.at(i);
      if (lhsGlove != rhsGlove)
      {
        return false;
      }
    }
    return true;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const SenseGloveSetup& senseGloveSetup)
  {
    for (unsigned int i = 0; i < senseGloveSetup.sensegloves_.size(); i++)
    {
      os << senseGloveSetup.sensegloves_.at(i) << "\n";
    }
    return os;
  }
};
}  // namespace senseglove

#endif  // ROS_WORKSPACE_SENSEGLOVE_SETUP_H
