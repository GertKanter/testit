# VERSION 0.0.1
FROM ubuntu:bionic
MAINTAINER Gert Kanter <gert.kanter@taltech.ee>
LABEL Description="TestIt! ROS Testing toolkit base docker image"
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y wget vim git gnupg2 && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sh -c 'apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116' && \
    apt-get update && \
    apt-get install -y ros-melodic-desktop-full && \
    rosdep init && \
    rosdep update && \
    mkdir -p /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /catkin_ws/src && catkin_init_workspace" && \
    cd /catkin_ws/src && \
    git clone https://github.com/GertKanter/testit.git && \
    git clone https://github.com/GertKanter/testit_msgs.git && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /catkin_ws && catkin_make"
