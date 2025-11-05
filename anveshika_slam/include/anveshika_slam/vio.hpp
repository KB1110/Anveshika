#ifndef ANVESHIKA_SLAM_VIO_HPP
#define ANVESHIKA_SLAM_VIO_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"

namespace anveshika_slam
{

class VIONode : public rclcpp::Node {
public:
    VIONode();

private:   
    void rgbdCallback(slam_msgs::msg::RGBDImage::ConstSharedPtr msg);
    void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);

    rclcpp::Subscription<slam_msgs::msg::RGBDImage>::SharedPtr rgbd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

} // namespace anveshika_slam

#endif // ANVESHIKA_SLAM_VIO_HPP