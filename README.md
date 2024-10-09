Anveshika: Autonomous Sample Retrieving Planetary Rover

Leading a dedicated team into the finals of the ISRO Robotics Challenge 2024, we've engineered an Autonomous Sample Retrieving Planetary Rover, poised to revolutionize extraterrestrial exploration:

Engineered an innovative 4-wheeled Dual Bogie Differential Suspension System, specifically tailored for off-terrain stability on extraterrestrial landscapes. This groundbreaking design ensures unparalleled maneuverability and adaptability in challenging environments.

Ensured the rover's construction maintained an exceptional balance between robustness and efficiency, meticulously crafting an extremely light and compact form factor. This optimization allows for enhanced mobility and minimized energy consumption, crucial for extended missions in space exploration.

Utilized ROS2 as middleware to orchestrate seamless communication and collaboration between various software components. This integration ensures the rover's onboard systems operate harmoniously, facilitating efficient data processing and decision-making.

Leveraged cutting-edge technologies for autonomous navigation, employing Cartographer for real-time Simultaneous Localization and Mapping (SLAM) with seamless path planning and navigation. These integrated systems enable the rover to autonomously traverse complex terrains with precision and reliability. Also, incorporated Inverse Kinematics, enabling precise control of the robotic arm for sample retrieval tasks. 

All processing tasks, from sensor data fusion to decision-making algorithms, are performed onboard the rover itself. This onboard processing capability enhances autonomy and reduces reliance on external resources, making the rover well-equipped for independent exploration missions in remote environments.

ros2 run cartographer_ros cartographer_node -configuration_directory ~/Anveshika/Anveshika_ws/install/slam_lidar/share/slam_lidar/configuration -configuration_basename config.lua

ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0

ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/Anveshika/Anveshika_ws/install/anveshika_detection/share/anveshika_detection/params/realsense.yaml

