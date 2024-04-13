cmake_minimum_required(VERSION 3.5)
project(traversability_mapping_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(traversability_mapping_common REQUIRED)

set(dependencies
  ament_cmake_auto
  ament_cmake
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  traversability_mapping_common
)

# INCLUDE
include_directories(
  include
)

link_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

set(include_dependencies
    include
)
# ~INCLUDE

add_library(PointCloudBuffer
  src/PointCloudBuffer.cpp
)
target_link_libraries(PointCloudBuffer
    ${traversability_mapping_common_LIBRARIES}
)
ament_target_dependencies(PointCloudBuffer PUBLIC ${dependencies})

install(TARGETS PointCloudBuffer
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories(${include_dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()