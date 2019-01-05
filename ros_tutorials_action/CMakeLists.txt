cmake_minimum_required(VERSION 2.8.3)
project(ros_tutorials_action)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  actionlib
  rospy
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(FILES Fibonacci.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  LIBRARIES ros_tutorials_action
  CATKIN_DEPENDS std_msgs actionlib_msgs actionlib rospy
)

include_directories(${catkin_INCLUDE_DIRS})
