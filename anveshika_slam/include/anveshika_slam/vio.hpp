#ifndef ANVESHIKA_SLAM_VIO_HPP
#define ANVESHIKA_SLAM_VIO_HPP

#include <rclcpp/rclcpp.hpp>

namespace anveshika_slam
{

class VIONode : public rclcpp::Node
{
public:
    explicit VIONode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~VIONode();

private:

    // Add your member variables and methods here
};

} // namespace anveshika_slam

#endif // ANVESHIKA_SLAM_VIO_HPP