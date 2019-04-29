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

install(PROGRAMS bin/testit_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Configure Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  execute_process(COMMAND docker pull ros:latest)
  execute_process(COMMAND rosrun testit build_local_testit_docker.sh)

  add_rostest(test/01/config.test)
endif()
