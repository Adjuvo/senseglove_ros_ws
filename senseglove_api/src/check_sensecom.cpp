#include "rclcpp/rclcpp.hpp"
#include "SenseGlove/Core/DeviceList.hpp"

#include <iostream>
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  for (int i = 0; i < 10; ++i) {
    bool running = SGCore::DeviceList::SenseComRunning();
    std::cout << "[Try " << i << "] SenseCom running? "
              << (running ? "yes" : "no") << std::endl;
    if (running) break;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::shutdown();
  return 0;
}
