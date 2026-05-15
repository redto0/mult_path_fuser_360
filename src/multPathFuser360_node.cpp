#include "mult_path_fuser_360/multPathFuser360_node.hpp"

// For _1
using namespace std::placeholders;

mult_path_fuser_360::mult_path_fuser_360(const rclcpp::NodeOptions& options) : Node("mult_path_fuser_360", options) {
    // Parameters
    float x = this->declare_parameter<float>("foo", -10.0);

    this->path_gps_sub = this->create_subscription<nav_msgs::msg::Path>("/path_gps", 5);

    this->path_vision_sub = this->create_subscription<nav_msgs::msg::Path>("/path_vision", 5);

    this->combined_path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // tf
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

// Store the latest GPS path
void mult_path_fuser_360::gps_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // store the lastest gps_path
    // note this only works because the GPS path is published at a lower rate than the other path! 
    this->path_gps = *msg;
}

// Fuse paths: vision preferred, append extra GPS points if vision is shorter
void mult_path_fuser_360::vision_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    nav_msgs::msg::Path fused_path = *msg;  // Start with vision path

    // Append any extra GPS points
    if (!path_gps.poses.empty()) {
        size_t vision_size = fused_path.poses.size();
        size_t gps_size = path_gps.poses.size();

        if (gps_size > vision_size) {
            // Append GPS points beyond the vision path
            fused_path.poses.insert(fused_path.poses.end(), path_gps.poses.begin() + vision_size, path_gps.poses.end());
        }
    }

    // Publish fused path
    combined_path_pub->publish(fused_path);
}