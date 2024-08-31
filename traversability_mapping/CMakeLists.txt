cmake_minimum_required(VERSION 3.5)
project(traversability_mapping VERSION 1.0)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(Sophus REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
# find_package(PCL)
find_package(traversability_mapping_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

ADD_DEFINITIONS("-DWITH_TRAVERSABILITY_MAP2")

# Create library target
add_library(traversabilitySystem SHARED
              src/Parameters.cpp
              src/Helpers.cpp
              src/TraversabilityGrid.cpp
              src/KeyFrame.cpp
              src/LocalMap.cpp
              src/PointCloudBuffer.cpp
              src/System.cpp)

# Set include directory
target_include_directories(traversabilitySystem
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        # ${PCL_INCLUDE_DIRS}
        ${traversability_mapping_common_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${grid_map_core_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(traversabilitySystem
    PRIVATE ${Sophus_LIBRARIES}
    PRIVATE ${EIGEN3_LIBS}
    # PRIVATE ${PCL_LIBRARIES}
    # PRIVATE ${sensor_msgs_LIBRARIES}
    PRIVATE ${grid_map_core_LIBRARIES}
    PRIVATE ${YAML_CPP_LIBRARIES}
    type_conversion
)

# Generate and install CMake configuration file
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

# Add executable
add_executable(traversabilityMapping src/main.cpp)

# Link the executable with the traversabilitySystem library
target_link_libraries(traversabilityMapping traversabilitySystem)

install(TARGETS traversabilitySystem traversabilityMapping
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(EXPORT export_${PROJECT_NAME}
    FILE export_${PROJECT_NAME}.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

export(EXPORT export_${PROJECT_NAME}
    FILE "${CMAKE_CURRENT_BINARY_DIR}/export_${PROJECT_NAME}.cmake"
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY params/ DESTINATION params)
