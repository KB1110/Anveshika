#include "rclcpp/rclcpp.hpp"
#include "anveshika_slam/sensor_interface.hpp"
#include "anveshika_slam/vio.hpp"
#include "anveshika_slam/lio.hpp"
#include "anveshika_slam/pose_graph.hpp"
#include "anveshika_slam/octomap_mapper.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto sensor_node = std::make_shared<anveshika_slam::SensorInterfaceNode>();
    auto vio_node = std::make_shared<anveshika_slam::VIONode>();
    auto lio_node = std::make_shared<anveshika_slam::LIONode>();
    auto pg_node = std::make_shared<anveshika_slam::PoseGraphNode>();
    auto octo_node = std::make_shared<anveshika_slam::OctomapMapperNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensor_node);
    executor.add_node(vio_node);
    executor.add_node(lio_node);
    executor.add_node(pg_node);
    executor.add_node(octo_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
