#include <rclcpp/rclcpp.hpp>

class PackageBuildDemoNode : public rclcpp::Node
{
public:
  PackageBuildDemoNode() : Node("demo_package_build_cpp_node")
  {
    RCLCPP_INFO(get_logger(), "============================================================");
    RCLCPP_INFO(get_logger(), "  验收点 1（C++）：独立建包能力演示");
    RCLCPP_INFO(get_logger(), "============================================================");
    RCLCPP_INFO(get_logger(), "  包名   : ur3_stage1_review_cpp");
    RCLCPP_INFO(get_logger(), "  节点名 : %s", get_name());
    RCLCPP_INFO(get_logger(), "  命名空间: %s", get_namespace());
    RCLCPP_INFO(get_logger(), "  [结论] C++ 包结构正确，colcon 构建通过，节点可正常启动。");
    RCLCPP_INFO(get_logger(), "============================================================");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PackageBuildDemoNode>();
  rclcpp::shutdown();
  return 0;
}
