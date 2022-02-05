cmake_minimum_required(VERSION 3.0.2)
project(all_estimator)


# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  tf2_ros
  denso_srvs
  denso_msgs
  denso_recognition_srvs
  tf
  pcl_ros
)

find_package(PCL REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES all_estimator
#  CATKIN_DEPENDS cv_bridge roscpp rospy tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(yolo_server_2D src/yolo_server_2D.cpp)
target_link_libraries(yolo_server_2D
  ${catkin_LIBRARIES}
)

add_executable(2D_to_3D_insnum src/2D_to_3D_insnum.cpp)
target_link_libraries(2D_to_3D_insnum
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(Acc_2D_to_3D_insnum src/Acc_2D_to_3D_insnum.cpp)
target_link_libraries(Acc_2D_to_3D_insnum
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(2D_to_3D_insnum_multi src/2D_to_3D_insnum_multi.cpp)
target_link_libraries(2D_to_3D_insnum_multi
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(quaternion_convert src/quaterion_convert.cpp)
target_link_libraries(quaternion_convert
  ${catkin_LIBRARIES}
)

add_executable(calcurate_error src/calcurate_error.cpp)
target_link_libraries(calcurate_error
  ${catkin_LIBRARIES}
)

add_executable(message_server src/message_server.cpp)
target_link_libraries(message_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(ICP_server src/ICP_server.cpp)
target_link_libraries(ICP_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

