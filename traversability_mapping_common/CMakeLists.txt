cmake_minimum_required(VERSION 3.5)
project(traversability_mapping_common VERSION 1.0)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  Sophus
  Eigen3
)

# Create library target
add_library(type_conversion src/type_conversion.cpp)

# Set include directory
target_include_directories(type_conversion
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(type_conversion
    ${Sophus_LIBRARIES}
    ${EIGEN3_LIBS}
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

install(TARGETS type_conversion
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
