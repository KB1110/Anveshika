cmake_minimum_required(VERSION 3.8)
project(anveshika_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

include_directories(include)

add_executable(anveshika_slam
    src/main.cpp
    src/sensor_interface.cpp
    src/vio.cpp
    src/lio.cpp
    src/pose_graph.cpp
    src/octomap_mapper.cpp
)

ament_target_dependencies(anveshika_slam
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    octomap_msgs
    octomap
)

install(TARGETS
  anveshika_slam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
