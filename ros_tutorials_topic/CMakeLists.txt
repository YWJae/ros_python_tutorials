cmake_minimum_required(VERSION 2.8.3)
project(ros_tutorials_topic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(FILES MsgTutorial.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES ros_tutorials_topic
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

