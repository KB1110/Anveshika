#ifndef ANVESHIKA_SLAM_SENSOR_INTERFACE_HPP
#define ANVESHIKA_SLAM_SENSOR_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace anveshika_slam
{

class SensorInterfaceNode : public rclcpp::Node
{
public:
  SensorInterfaceNode();

private:
  void rgbdCallback(sensor_msgs::msg::Image::ConstSharedPtr rgb_msg, sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
  void cameraImuCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void bodyImuCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;

  using RGBDSyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<RGBDSyncPolicy>> rgbd_sync_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr camera_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr body_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr camera_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr body_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;

  std::string rgb_topic_in_;
  std::string depth_topic_in_;
  std::string camera_imu_topic_in_;
  std::string body_imu_topic_in_;
  std::string lidar_topic_in_;

  std::string rgb_topic_out_;
  std::string depth_topic_out_;
  std::string camera_imu_topic_out_;
  std::string body_imu_topic_out_;
  std::string lidar_topic_out_;

  void declareParameters();
};

}  // namespace anveshika_slam

#endif  // ANVESHIKA_SLAM_SENSOR_INTERFACE_HPP