#include "rclcpp/rclcpp.hpp"
#include "kelo_tulip_ros2/PlatformDriverNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Every parameter is declared explicitly in the node (constructor, readWheelConfig). Do NOT use
  // automatically_declare_parameters_from_overrides here: the values in the YAML file would be
  // declared twice (exception at run time).
  rclcpp::NodeOptions options;
  auto node = std::make_shared<kelo_tulip_ros2::PlatformDriverNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
