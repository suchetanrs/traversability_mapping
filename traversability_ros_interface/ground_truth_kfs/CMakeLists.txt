cmake_minimum_required(VERSION 3.5)
project(ground_truth_kfs)

# Add dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(traversability_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(slam_keyframe_simulator src/slam_keyframe_simulator.cpp)
ament_target_dependencies(slam_keyframe_simulator rclcpp nav_msgs traversability_msgs tf2_ros)

# Add the executable
add_executable(slam_keyframe_pcl_simulator src/slam_keyframe_pcl_simulator.cpp)
ament_target_dependencies(slam_keyframe_pcl_simulator rclcpp nav_msgs traversability_msgs tf2_ros)

# Install the executable
install(TARGETS
  slam_keyframe_simulator slam_keyframe_pcl_simulator
  DESTINATION lib/${PROJECT_NAME})

# Finalize
ament_package()