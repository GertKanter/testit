cmake_minimum_required(VERSION 2.8.3)
project(testit_learn)

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
   ClusterPoint.msg
   StateMachine.msg
   UppaalAutomata.msg
)

add_service_files(
   DIRECTORY srv
   FILES
   LogToCluster.srv
   ClusterToStateMachine.srv
   StateMachineToUppaal.srv
   WriteUppaalModel.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  
)

include_directories()
