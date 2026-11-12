#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "skills_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "skills", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int code = 0;
  try
  {
    so101_skills::SkillsServer server(node);
    spinner.join();  // until Ctrl-C
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  if (spinner.joinable())
    spinner.join();
  return code;
}
