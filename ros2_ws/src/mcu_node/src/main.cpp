#include "mcu_node/gimbal.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mcu_node::McuNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mcu_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

