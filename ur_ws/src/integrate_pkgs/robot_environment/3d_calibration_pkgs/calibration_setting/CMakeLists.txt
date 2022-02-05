cmake_minimum_required(VERSION 3.0.2)
project(calibration_setting)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calibration_setting
#  CATKIN_DEPENDS roscpp tf2_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibration_3d src/calibration_3d.cpp)
target_link_libraries(calibration_3d
  ${catkin_LIBRARIES}
)

