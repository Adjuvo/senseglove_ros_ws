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
    SenseGloveSetup(SenseGloveSetup&) = delete;
    SenseGloveSetup& operator=(SenseGloveSetup&) = delete;
    // Allow move
    SenseGloveSetup(SenseGloveSetup&&) = default;
    SenseGloveSetup& operator=(SenseGloveSetup&&) = default;

    // Start/stop communication across all robots; implementations log at node level
    void startCommunication(bool reset);
    void stopCommunication();
    bool isCommunicationOperational();

    // Access by name
    SenseGloveRobot& getSenseGloveRobot(const std::string & gloveName);

    // Access by index
    SenseGloveRobot& getSenseGloveRobot(size_t index);

    size_t size() const noexcept { return SGRobots.size(); }
    iterator begin() noexcept { return SGRobots.begin(); }
    iterator end() noexcept { return SGRobots.end(); }
    const_iterator begin() const noexcept { return SGRobots.begin(); }
    const_iterator end() const noexcept { return SGRobots.end(); }

    // Retrieve URDF model of a robot by name
    const std::shared_ptr<urdf::Model>& getRobotUrdf(const std::string & gloveName);

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
