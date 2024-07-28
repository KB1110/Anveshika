ros2 run cartographer_ros cartographer_node -configuration_directory ~/Anveshika/Anveshika_ws/install/slam_lidar/share/slam_lidar/configuration -configuration_basename config.lua

ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0

ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/Anveshika/Anveshika_ws/install/anveshika_detection/share/anveshika_detection/params/realsense.yaml

