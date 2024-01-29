cmake_minimum_required(VERSION 3.10)
project(pcd2pgm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd2pgm
#  CATKIN_DEPENDS nav_msgs pcl roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)

include_directories(
  include
		${PCL_INCLUDE_DIRS})


add_executable(pcd2topic src/test.cpp)
target_link_libraries(pcd2topic ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )


