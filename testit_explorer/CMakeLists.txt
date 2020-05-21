cmake_minimum_required(VERSION 2.8.3)
project(testit_explorer)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  genmsg
  rospy
  roscpp
)

catkin_python_setup()

add_message_files(
   DIRECTORY msg
   FILES
   Action.msg
   Actions.msg
   State.msg
   MoveStrategyInit.msg
)

add_service_files(
   DIRECTORY srv
   FILES
   MoveStrategy.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  
)

include_directories()
