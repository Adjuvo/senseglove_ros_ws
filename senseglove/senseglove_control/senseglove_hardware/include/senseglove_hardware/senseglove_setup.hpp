#ifndef ROS_WORKSPACE_SENSEGLOVE_SETUP_H
#define ROS_WORKSPACE_SENSEGLOVE_SETUP_H

#include <senseglove_hardware/senseglove_robot.hpp>

#include <stdexcept>

namespace SGHardware
{
  class SenseGloveSetup
  {
  public:
    using RobotPtr = std::unique_ptr<SenseGloveRobot>;
    using iterator = std::vector<RobotPtr>::iterator;
    using const_iterator = std::vector<RobotPtr>::const_iterator;

    // Construct from a single robot (moved in)
    explicit SenseGloveSetup(RobotPtr robot);

    // Construct from multiple robots (moved in)
    explicit SenseGloveSetup(std::vector<RobotPtr> robots);

    ~SenseGloveSetup() = default;

    // Non-copyable (SenseGloveRobot is non-copyable)
    SenseGloveSetup(const SenseGloveSetup&) = delete;
    SenseGloveSetup& operator=(const SenseGloveSetup&) = delete;
    // Allow move
    SenseGloveSetup(SenseGloveSetup&&) = default;
    SenseGloveSetup& operator=(SenseGloveSetup&&) = default;

    // Start/stop communication across all robots; implementations log at node level
    void startCommunication(bool reset);
    void stopCommunication();
    bool isCommunicationOperational() const;

    // Access by name
    const SenseGloveRobot& getSenseGloveRobot(const std::string & gloveName) const;

    // Access by index
    const SenseGloveRobot& getSenseGloveRobot(size_t index) const;

    size_t size() const noexcept { return SGRobots.size(); }
    iterator begin() noexcept { return SGRobots.begin(); }
    iterator end() noexcept { return SGRobots.end(); }
    const_iterator begin() const noexcept { return SGRobots.begin(); }
    const_iterator end() const noexcept { return SGRobots.end(); }

    // Retrieve URDF model of a robot by name
    const urdf::Model& getRobotUrdf(const std::string & gloveName) const;

    // Comparison operators
    friend bool operator==(const SenseGloveSetup& lhs, const SenseGloveSetup& rhs)
    {
      if (lhs.SGRobots.size() != rhs.SGRobots.size())
        return false;
      for (size_t i = 0; i < lhs.SGRobots.size(); ++i) {
        if (*lhs.SGRobots[i] != *rhs.SGRobots[i]) {
          return false;
        }
      }
      return true;
    }
    friend bool operator!=(const SenseGloveSetup& lhs, const SenseGloveSetup& rhs)
    {
      return !(lhs == rhs);
    }

    // Stream operator
    friend std::ostream& operator<<(std::ostream& os, const SenseGloveSetup& setup)
    {
      for (const auto& robot_ptr : setup.SGRobots) {
        os << *robot_ptr << "\n";
      }
      return os;
    }
    
  private:
    std::vector<RobotPtr> SGRobots;
    std::unordered_map<std::string, size_t> nameMap;

    void buildMap();
  };
}  // namespace SGHardware

#endif  // ROS_WORKSPACE_SENSEGLOVE_SETUP_H
