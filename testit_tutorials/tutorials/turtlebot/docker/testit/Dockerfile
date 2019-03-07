FROM testitros/testit:latest
MAINTAINER Gert Kanter <gert.kanter@taltech.ee>
LABEL Description="TestIt! ROS Testing toolkit docker image"
RUN apt-get update && \
    apt-get install -y ros-lunar-move-base-msgs && \
    cd /catkin_ws/src && \
    git clone https://github.com/mission-control-ros/mission_control && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/lunar/setup.bash && catkin_make"
CMD bash
