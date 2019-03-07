FROM ros:kinetic-robot-xenial
MAINTAINER Gert Kanter <gert.kanter@taltech.ee>
LABEL Description="TestIt! ROS Testing toolkit tutorial SUT image"
RUN apt-get update && \
    apt-get install -y ros-kinetic-turtlebot-navigation ros-kinetic-turtlebot-gazebo wget xvfb && \
    /bin/bash -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main\" > /etc/apt/sources.list.d/gazebo-stable.list" && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get upgrade -y && \
    mkdir -p /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws/src && catkin_init_workspace" && \
    cd /catkin_ws/src && \
    git clone https://github.com/mission-control-ros/mission_control && \
    git clone https://github.com/ros/executive_smach.git && \
    git clone https://github.com/ros-perception/slam_gmapping.git && \
    cd slam_gmapping && \
    git fetch origin pull/56/head:nodelet_fix && \
    git checkout nodelet_fix && \
    cd .. && \
    /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make"
CMD ['/bin/bash', ' -c "source /catkin_ws/devel/setup.bash && rosrun mission_control start_move_base_in_docker.sh && tail -f /dev/null"']
