cmake_minimum_required(VERSION 2.8.3)

project(wheelchair_robot4)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
  controller_manager
  message_generation
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

