FROM testitros/testit:latest
MAINTAINER Gert Kanter <gert.kanter@ttu.ee>
LABEL Description="TestIt! ROS Testing toolkit docker image"
RUN apt-get update && \
    apt-get install -y ros-lunar-move-base-msgs && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/lunar/setup.bash && catkin_init_workspace" && \
    git clone https://github.com/mission-control-ros/mission_control && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/lunar/setup.bash && catkin_make"
CMD bash
