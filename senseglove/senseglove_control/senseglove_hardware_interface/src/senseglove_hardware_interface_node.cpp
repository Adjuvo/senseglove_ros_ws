// Copyright (c) 2020 - 2025 SenseGlove

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

#include <senseglove_hardware/senseglove_robot.hpp>
#include <senseglove_hardware_builder/hardware_builder.hpp>
#include <senseglove_hardware_interface/senseglove_hardware_interface.hpp>

std::unique_ptr<SGHardware::SenseGloveSetup> build(AllowedRobot selectedRobot, int gloveIndex, bool isRight, const rclcpp::Logger & logger);
bool toBool(const std::string &str);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("senseglove_hardware_interface");

  if (argc < 4)
  {
    RCLCPP_FATAL_STREAM(node->get_logger(), 
      "Usage: senseglove_hardware_interface_node Robot gloveIndex isRight");
    return EXIT_FAILURE;
  }

  AllowedRobot selectedRobot = AllowedRobot(argv[1]);
  int gloveIndex = std::stoi(argv[2]);
  bool isRight = toBool(argv[3]);

  RCLCPP_INFO_STREAM(node->get_logger(), 
    "Selected robot: " << selectedRobot << ", index: " << gloveIndex << ", isRight: " << isRight);

  auto setup = build(selectedRobot, gloveIndex, isRight, node->get_logger());
  auto senseglove_hw = std::make_shared<SenseGloveHardwareInterface>(std::move(setup));

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);

  // Load publish rate parameter
  double publish_rate = node->declare_parameter("publish_rate", 60.0); // default 50 Hz
  RCLCPP_INFO_STREAM(node->get_logger(), "Using publish rate: " << publish_rate << " Hz");

  controller_manager::ControllerManager cm(senseglove_hw, node, "senseglove_controller_manager");

  auto last_time = std::chrono::steady_clock::now();
  const auto desired_period = std::chrono::duration<double>(1.0 / publish_rate);
  rclcpp::Rate rate(publish_rate);

  while (rclcpp::ok())
  {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = now - last_time;
    last_time = now;

    senseglove_hw->read(now, elapsed_time);
    cm.update(node->now(), rclcpp::Duration::from_seconds(elapsed_time.count()));
    senseglove_hw->write(now, elapsed_time);

    executor->spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

std::unique_ptr<SGHardware::SenseGloveSetup> build(AllowedRobot selectedRobot, int gloveIndex, bool isRight, const rclcpp::Logger & logger)
{
  try
  {
    HardwareBuilder builder(selectedRobot, gloveIndex, isRight);
    return builder.createSenseGloveSetup();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL_STREAM(logger, 
      "Exception while building hardware: " << e.what());
    std::exit(EXIT_FAILURE);
  }
}

bool toBool(const std::string &str)
{
  std::string lower_str = str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
  std::istringstream is(lower_str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}