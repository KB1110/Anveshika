#include "anveshika_slam/vio.hpp"

namespace anveshika_slam {

VIONode::VIONode() : Node("vio_node") {
    rgbd_sub_ = this->create_subscription<slam_msgs::msg::RGBDImage>(
        "/slam/rgbd/image", 10,
        std::bind(&VIONode::rgbdCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/slam/imu/cam", 100,
        std::bind(&VIONode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "VIO node initialized.");
}

void VIONode::rgbdCallback(const slam_msgs::msg::RGBDImage::ConstSharedPtr& msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received RGBD frame at %u", msg->header.stamp.sec);
    // TODO: Visual feature tracking, stereo triangulation, depth association
}

void VIONode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received IMU reading at %u", msg->header.stamp.sec);
    // TODO: IMU pre-integration for motion prediction
}

}  // namespace anveshika_slam
