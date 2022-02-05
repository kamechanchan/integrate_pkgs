cmake_minimum_required(VERSION 3.0.2)
project(denso_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
)


add_message_files(
  FILES
  object_kiriwake.msg
  out_segmentation.msg
  yolo_bridge.msg
  new_array.msg
  new_1_array.msg
  denso_pointcloud2.msg
  dummy_pcl.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs  # Or other packages containing msgs
)


catkin_package(
 CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

