// Copyright 2020 SenseGlove.
#ifndef ROS_WORKSPACE_SENSEGLOVE_SETUP_H
#define ROS_WORKSPACE_SENSEGLOVE_SETUP_H

#include "senseglove_hardware/joint.h"

namespace SGHardware
{
  class SenseGloveSetup
  {
  private:
    ::std::vector<SGHardware::SenseGloveRobot> SGRobots;

  public:
    using iterator = std::vector<SGHardware::SenseGloveRobot>::iterator;

    SenseGloveSetup(SGHardware::SenseGloveRobot SGRobots);
    SenseGloveSetup(std::vector<SGHardware::SenseGloveRobot> SGRobots);

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
      if (lhs.SGRobots.size() != rhs.SGRobots.size())
      {
        return false;
      }
      for (unsigned int i = 0; i < lhs.SGRobots.size(); i++)
      {
        const SGHardware::SenseGloveRobot& lhsGlove = lhs.SGRobots.at(i);
        const SGHardware::SenseGloveRobot& rhsGlove = rhs.SGRobots.at(i);
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
      for (unsigned int i = 0; i < senseGloveSetup.SGRobots.size(); i++)
      {
        os << senseGloveSetup.SGRobots.at(i) << "\n";
      }
      return os;
    }
  };
}  // namespace SGHardware

#endif  // ROS_WORKSPACE_SENSEGLOVE_SETUP_H
