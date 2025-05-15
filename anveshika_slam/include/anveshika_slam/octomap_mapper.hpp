#ifndef ANVESHIKA_SLAM_OCTOMAP_MAPPER_HPP
#define ANVESHIKA_SLAM_OCTOMAP_MAPPER_HPP

#include <rclcpp/rclcpp.hpp>

namespace anveshika_slam
{

class OctomapMapperNode : public rclcpp::Node
{
public:
    explicit OctomapMapperNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~OctomapMapperNode();

private:
    // Add your member variables and methods here
};

} // namespace anveshika_slam

#endif // ANVESHIKA_SLAM_OCTOMAP_MAPPER_HPP