#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "smart_escape/smart_escape_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 1. 启用多线程执行器
  // 这允许节点同时处理 Costmap 回调和 Action 请求，互不阻塞
  rclcpp::executors::MultiThreadedExecutor executor;

  // 2. 创建节点配置
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false); // 尝试使用进程内通信加速

  // 3. 实例化脱困服务器
  auto node = std::make_shared<smart_escape::SmartEscapeServer>(options);

  // 4. 开始运行
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "SmartEscapeServer starting with MultiThreadedExecutor...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}