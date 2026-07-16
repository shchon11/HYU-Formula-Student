#include "wpnt_publisher_node.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wpnt_publisher::WpntPublisher>());
  rclcpp::shutdown();
  return 0;
}
