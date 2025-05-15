#include "anveshika_slam/pose_graph.hpp"

namespace anveshika_slam
{

PoseGraphNode::PoseGraphNode(const rclcpp::NodeOptions &options)
: Node("pose_graph_node", options)
{

}

PoseGraphNode::~PoseGraphNode()
{

}

} // namespace anveshika_slam