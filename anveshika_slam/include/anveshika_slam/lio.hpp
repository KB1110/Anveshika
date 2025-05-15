#ifndef ANVESHIKA_SLAM_LIO_HPP
#define ANVESHIKA_SLAM_LIO_HPP

#include <rclcpp/rclcpp.hpp>

namespace anveshika_slam
{

class LIONode : public rclcpp::Node
{
public:
    explicit LIONode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LIONode();

private:

    // Add your member variables and methods here
};

} // namespace anveshika_slam

#endif // ANVESHIKA_SLAM_LIO_HPP