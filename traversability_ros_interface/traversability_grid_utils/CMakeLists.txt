cmake_minimum_required(VERSION 3.5)
project(traversability_grid_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traversability_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  traversability_msgs
  grid_map_core
  grid_map_ros
  Eigen3
)

add_library(${PROJECT_NAME} SHARED
  src/traversability_grid.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(traversability_grid_consumer examples/traversability_grid_consumer.cpp)
target_link_libraries(traversability_grid_consumer ${PROJECT_NAME})
ament_target_dependencies(traversability_grid_consumer rclcpp)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS traversability_grid_consumer
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
