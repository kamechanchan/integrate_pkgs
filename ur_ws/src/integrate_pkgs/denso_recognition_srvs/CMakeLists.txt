cmake_minimum_required(VERSION 2.8.3)
project(denso_recognition_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  CropCloud.srv
  ExtractObject.srv
  InversePose.srv
  MakeInput.srv
  PoseEstimate.srv
  RegistrationPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
