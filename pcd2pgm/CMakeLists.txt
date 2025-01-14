cmake_minimum_required(VERSION 3.5)
project(pcd2pgm)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions  REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Include directories
#include_directories(include ${PCL_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(pcd2pgm src/test.cpp)
ament_target_dependencies(pcd2pgm
        rclcpp
        sensor_msgs
        nav_msgs
        pcl_ros
        pcl_conversions
)

add_executable(livox_repub src/livox_repub.cpp)
ament_target_dependencies(livox_repub
        rclcpp
        sensor_msgs
        nav_msgs
        pcl_ros
        pcl_conversions
        livox_ros_driver2
)


# Install targets
install(TARGETS pcd2pgm livox_repub
        DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs nav_msgs pcl_ros pcl_conversions livox_ros_driver2)

ament_package()
