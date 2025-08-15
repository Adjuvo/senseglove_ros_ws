#include "rclcpp/rclcpp.hpp"
#include "SenseGlove/Core/DeviceList.hpp"
#include "SenseGlove/Core/SenseGlove.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (!SGCore::DeviceList::SenseComRunning()) {
    std::cerr << "SenseCom is NOT running, cannot check gloves." << std::endl;
    return 1;
  }

  // Get all connected haptic gloves
  auto gloves = SGCore::HapticGlove::GetHapticGloves(true);

  if (gloves.empty()) {
    std::cout << "No SenseGloves detected." << std::endl;
  } else {
    std::cout << "Detected " << gloves.size() << " glove(s):" << std::endl;
    int i = 0;
    for (auto & g : gloves) {
      std::cout << " [" << i++ << "] "
                << "ID: " << g->GetDeviceId()
                << " - isRight: " << (g->IsRight() ? "true" : "false")
                << " - deviceType: " << (int)g->GetDeviceType()
                << std::endl;
    }
  }

  rclcpp::shutdown();
  return 0;
}
