cmake_minimum_required(VERSION 3.0.2)
project(real_final_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  denso_srvs
  denso_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES real_final_package
#  CATKIN_DEPENDS denso_msgs cv_bridge image_geometry image_transport message_filters message_generation pcl_ros roscpp sensor_msgs std_msgs tf tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_save src/image_save.cpp)
target_link_libraries(image_save
  ${catkin_LIBRARIES}
)

add_executable(image_subscriber src/image_subscriber.cpp)
target_link_libraries(image_subscriber
  ${catkin_LIBRARIES}
)

add_executable(image_service_server src/image_service_server.cpp)
target_link_libraries(image_service_server
  ${catkin_LIBRARIES}
)

add_executable(image_service_client src/image_service_client.cpp)
target_link_libraries(image_service_client
  ${catkin_LIBRARIES}
)