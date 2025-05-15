#ifndef ANVESHIKA_SLAM_POSE_GRAPH_HPP
#define ANVESHIKA_SLAM_POSE_GRAPH_HPP

#include <rclcpp/rclcpp.hpp>

namespace anveshika_slam
{

class PoseGraphNode : public rclcpp::Node
{
public:
    explicit PoseGraphNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PoseGraphNode();

private:

    // Add your member variables and methods here
};

} // namespace anveshika_slam

#endif // ANVESHIKA_SLAM_POSE_GRAPH_HPP