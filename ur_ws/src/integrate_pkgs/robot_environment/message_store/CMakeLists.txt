cmake_minimum_required(VERSION 3.0.2)
project(message_store)

# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  denso_msgs
  denso_srvs
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES message_store
#  CATKIN_DEPENDS cv_bridge denso_msgs denso_srvs pcl_ros roscpp sensor_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(message_receiver src/message_receiver.cpp)
target_link_libraries(message_receiver
  ${catkin_LIBRARIES}
)