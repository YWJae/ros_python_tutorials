cmake_minimum_required(VERSION 2.8.3)
project(ros_tutorials_service)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

add_service_files(FILES SrvTutorial.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES ros_tutorials_service
  CATKIN_DEPENDS std_msgs rospy
)

include_directories(${catkin_INCLUDE_DIRS})
