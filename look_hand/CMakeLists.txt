cmake_minimum_required(VERSION 2.8.3)
project(look_hand)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  rospy
  tf
  message_generation
)

catkin_package(
)

 add_message_files(
   FILES
#   Message1.msg
#   Message2.msg
   arm_actions.msg
 )

 add_service_files(
   FILES
#   Service1.srv
#   Service2.srv
   can_detection.srv
   grip.srv
   shelf_detection.srv
    
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mission1
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(PROGRAMS
   scripts/look_hand.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
