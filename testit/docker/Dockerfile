# VERSION 0.0.1
FROM ubuntu:xenial
MAINTAINER Gert Kanter <gert.kanter@ttu.ee>
LABEL Description="TestIt! ROS Testing toolkit base docker image"
RUN apt-get update && \
    apt-get install -y wget vim git && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sh -c 'apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116' && \
    apt-get update && \
    apt-get install -y ros-lunar-desktop-full && \
    rosdep init && \
    rosdep update && \
    mkdir -p /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/lunar/setup.bash && cd /catkin_ws/src && catkin_init_workspace" && \
    cd /catkin_ws/src && \
    git clone https://github.com/GertKanter/testit.git && \
    /bin/bash -c "source /opt/ros/lunar/setup.bash && cd /catkin_ws && catkin_make"
