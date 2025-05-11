#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class AnveshikaSLAM : public rclcpp::Node
{
public:
  AnveshikaSLAM() : Node("anveshika_slam")
  {
    // Declare parameters
    this->declare_parameter("camera_frame", "camera_link");
    this->declare_parameter("queue_size", 10);

    // Initialize subscribers with message filters for synchronized RGB-D
    _color_sub.subscribe(this, "/realsense_d435i/color/image");
    _depth_sub.subscribe(this, "/realsense_d435i/depth/depth_image");
    _color_info_sub.subscribe(this, "/realsense_d435i/color/camera_info");
    _depth_info_sub.subscribe(this, "/realsense_d435i/color/camera_info");

    // Synchronize RGB-D messages
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                        sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;
    _synchronizer = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), _color_sub, _depth_sub,
                                                                                _color_info_sub, _depth_info_sub);

    _synchronizer->registerCallback(std::bind(&AnveshikaSLAM::_image_callback, this, std::placeholders::_1,
                                              std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    RCLCPP_INFO(this->get_logger(), "AnveshikaSLAM node initialized");
  }

private:
  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> _color_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> _depth_sub;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> _color_info_sub;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> _depth_info_sub;

  // Synchronizer
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>>>
      _synchronizer;

  // Latest frame storage
  cv::Mat _current_color_frame;
  cv::Mat _current_depth_frame;

  void _image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr& color_info_msg,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depth_info_msg)
  {
    try
    {
      // Convert ROS images to OpenCV format
      _current_color_frame = cv_bridge::toCvShare(color_msg, "bgr8")->image;
      _current_depth_frame = cv_bridge::toCvShare(depth_msg)->image;

      RCLCPP_INFO(this->get_logger(), "Processing frames...");

      // Process frames
      process_frames();
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  }

  void process_frames()
  {
    if (_current_color_frame.empty() || _current_depth_frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Empty frames received");
      return;
    }

    // TODO: Implement SLAM processing here
    // 1. Feature detection
    // 2. Feature matching
    // 3. Pose estimation
    // 4. Map updating
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AnveshikaSLAM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}