cmake_minimum_required(VERSION 3.0.2)
project(semantic_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  trajectory_msgs
  message_generation
  message_runtime
)

find_package(OpenCV REQUIRED)


add_service_files(
  FILES
  CamToReal.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES semantic_slam
CATKIN_DEPENDS roscpp rospy std_msgs moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)