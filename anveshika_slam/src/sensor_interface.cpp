#include "anveshika_slam/sensor_interface.hpp"

namespace anveshika_slam
{

SensorInterfaceNode::SensorInterfaceNode() : Node("sensor_interface_node")
{
  declareParameters();
  rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rgb_topic_in_);
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic_in_);

  rgbd_sync_ =
      std::make_shared<message_filters::Synchronizer<RGBDSyncPolicy>>(RGBDSyncPolicy(10), *rgb_sub_, *depth_sub_);
  rgbd_sync_->registerCallback(
      std::bind(&SensorInterfaceNode::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));

  camera_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      camera_imu_topic_in_, 100, std::bind(&SensorInterfaceNode::cameraImuCallback, this, std::placeholders::_1));

  body_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      body_imu_topic_in_, 100, std::bind(&SensorInterfaceNode::bodyImuCallback, this, std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_in_, 10, std::bind(&SensorInterfaceNode::lidarCallback, this, std::placeholders::_1));

  rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_topic_out_, 10);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_out_, 10);
  camera_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(camera_imu_topic_out_, 50);
  body_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(body_imu_topic_out_, 50);
  lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_out_, 10);
}

void SensorInterfaceNode::rgbdCallback(sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                                       sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Synced RGB and Depth Images");
  rgb_pub_->publish(*rgb_msg);
  depth_pub_->publish(*depth_msg);
}

void SensorInterfaceNode::cameraImuCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Camera IMU Data");
  camera_imu_pub_->publish(*imu_msg);
}

void SensorInterfaceNode::bodyImuCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Body IMU Data");
  body_imu_pub_->publish(*imu_msg);
}

void SensorInterfaceNode::lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Lidar Scan Data");
  lidar_pub_->publish(*lidar_msg);
}

void SensorInterfaceNode::declareParameters()
{
  this->declare_parameter<std::string>("rgb_topic_in", "/camera/rgb/image_raw");
  this->declare_parameter<std::string>("depth_topic_in", "/camera/depth/image_raw");
  this->declare_parameter<std::string>("camera_imu_topic_in", "/camera/imu");
  this->declare_parameter<std::string>("body_imu_topic_in", "/robot/imu");
  this->declare_parameter<std::string>("lidar_topic_in", "/lidar/points");

  this->declare_parameter<std::string>("rgb_topic_out", "/slam/rgb/image");
  this->declare_parameter<std::string>("depth_topic_out", "/slam/depth/image");
  this->declare_parameter<std::string>("camera_imu_topic_out", "/slam/imu/cam");
  this->declare_parameter<std::string>("body_imu_topic_out", "/slam/imu/body");
  this->declare_parameter<std::string>("lidar_topic_out", "/slam/lidar/points");

  rgb_topic_in_ = this->get_parameter("rgb_topic_in").as_string();
  depth_topic_in_ = this->get_parameter("depth_topic_in").as_string();
  camera_imu_topic_in_ = this->get_parameter("camera_imu_topic_in").as_string();
  body_imu_topic_in_ = this->get_parameter("body_imu_topic_in").as_string();
  lidar_topic_in_ = this->get_parameter("lidar_topic_in").as_string();

  rgb_topic_out_ = this->get_parameter("rgb_topic_out").as_string();
  depth_topic_out_ = this->get_parameter("depth_topic_out").as_string();
  camera_imu_topic_out_ = this->get_parameter("camera_imu_topic_out").as_string();
  body_imu_topic_out_ = this->get_parameter("body_imu_topic_out").as_string();
  lidar_topic_out_ = this->get_parameter("lidar_topic_out").as_string();
}

}  // namespace anveshika_slam
