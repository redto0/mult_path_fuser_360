#include "mult_path_fuser_360/multPathFuser360_node.hpp"

int main(int argc, char** argv) {
    // Setup runtime
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add nodes to executor
    auto node = std::make_shared<mult_path_fuser_360>(options);
    exec.add_node(node);

    // Run
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
