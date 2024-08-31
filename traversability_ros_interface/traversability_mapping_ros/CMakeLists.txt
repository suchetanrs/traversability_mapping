cmake_minimum_required(VERSION 3.5)
project(traversability_mapping_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traversability_msgs REQUIRED)

find_package(Sophus REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(traversability_mapping REQUIRED)
find_package(traversability_mapping_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_eigen REQUIRED)

set(dependencies
    ament_cmake
    rclcpp
    traversability_msgs

    Sophus
    Eigen3
    traversability_mapping
    traversability_mapping_common
    sensor_msgs
    grid_map_core
    grid_map_ros
    nav_msgs
    yaml-cpp
    tf2_eigen
)

include_directories(
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${traversability_mapping_common_INCLUDE_DIRS}
    ${traversability_mapping_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${grid_map_core_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# Add executable
add_executable(traversability_node src/traversability_node.cpp)
ament_target_dependencies(traversability_node ${dependencies})
target_link_libraries(traversability_node ${Sophus_LIBRARIES} ${EIGEN3_LIBS} ${grid_map_core_LIBRARIES} ${YAML_CPP_LIBRARIES} traversabilitySystem type_conversion)

# Add executable
add_executable(multi_map_merge src/multi_map_merge.cpp)
ament_target_dependencies(multi_map_merge ${dependencies})
target_link_libraries(multi_map_merge ${Sophus_LIBRARIES} ${EIGEN3_LIBS} ${grid_map_core_LIBRARIES} ${YAML_CPP_LIBRARIES} traversabilitySystem type_conversion)

# Add executable
add_executable(local_traversability src/local_traversability.cpp)
ament_target_dependencies(local_traversability ${dependencies})
target_link_libraries(local_traversability ${Sophus_LIBRARIES} ${EIGEN3_LIBS} ${grid_map_core_LIBRARIES} ${YAML_CPP_LIBRARIES} traversabilitySystem type_conversion)

# Add executable
add_executable(threshold_occupancy src/threshold_occupancy.cpp)
ament_target_dependencies(threshold_occupancy ${dependencies})
target_link_libraries(threshold_occupancy ${Sophus_LIBRARIES} ${EIGEN3_LIBS} ${grid_map_core_LIBRARIES} ${YAML_CPP_LIBRARIES} traversabilitySystem type_conversion)

# Install executable
install(TARGETS
    traversability_node multi_map_merge local_traversability threshold_occupancy
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch params
    DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(FILES
    package.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_package()