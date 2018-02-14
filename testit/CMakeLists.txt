cmake_minimum_required(VERSION 2.8.3)
project(testit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
  FILES
  Command.srv
  Status.srv
)

catkin_python_setup()

generate_messages(
  # DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(PROGRAMS bin/${PROJECT_NAME}.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
