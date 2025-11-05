#ifndef ANVESHIKA_SLAM_SENSOR_INTERFACE_HPP
#define ANVESHIKA_SLAM_SENSOR_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "anveshika_interfaces/msg/rgbd_image.hpp"

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
  void declareParameters();

  void rgbdCallback(sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                    sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                    sensor_msgs::msg::CameraInfo::ConstSharedPtr rgb_info_msg,
                    sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info_msg);

  void cameraImuCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void bodyImuCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_, depth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> rgb_info_sub_, depth_info_sub_;

  using RGBDSyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<message_filters::Synchronizer<RGBDSyncPolicy>> rgbd_sync_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr camera_imu_sub_, body_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  rclcpp::Publisher<anveshika_interfaces::msg::RGBDImage>::SharedPtr rgbd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr camera_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr body_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;

  std::string rgb_topic_in_;
  std::string depth_topic_in_;
  std::string camera_imu_topic_in_;
  std::string body_imu_topic_in_;
  std::string lidar_topic_in_;

  std::string rgb_info_topic_in_;
  std::string depth_info_topic_in_;

  std::string rgbd_topic_out_;
  std::string camera_imu_topic_out_;
  std::string body_imu_topic_out_;
  std::string lidar_topic_out_;
};

}  // namespace anveshika_slam

#endif  // ANVESHIKA_SLAM_SENSOR_INTERFACE_HPP