#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include "openarm_socketcan_ros2/openarm_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<openarm_socketcan_ros2::OpenArmNode>(rclcpp::NodeOptions());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Starting with explicit SingleThreadedExecutor");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
