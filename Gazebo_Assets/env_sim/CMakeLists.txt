cmake_minimum_required(VERSION 3.0.2)
project(env_sim)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  urdf
  xacro
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Model.msg
  LogicalCameraImage.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  vacuumGripper.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS 
 	geometry_msgs 
 	roscpp 
 	rospy 
 	std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)