cmake_minimum_required(VERSION 2.8.3)
project(testit_tutorials)

find_package(catkin REQUIRED
  rospy
  testit_oracles
)

catkin_python_setup()

catkin_package()

