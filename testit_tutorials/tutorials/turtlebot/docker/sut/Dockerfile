FROM ros:kinetic-robot-xenial
MAINTAINER Gert Kanter <gert.kanter@taltech.ee>
LABEL Description="TestIt! ROS Testing toolkit tutorial SUT image"
RUN echo "APT::Acquire::Retries \"30\";" > /etc/apt/apt.conf.d/80-retries
RUN apt-get update && \
    apt-get install -y lsb-release wget && \
    apt-get install -y libasound2 libxcb1
RUN apt-get install -y binutils cpp cpp-5 dkms fakeroot gcc gcc-5 kmod libasan2 libatomic1 libc-dev-bin libc6-dev libcc1-0 libcilkrts5 libfakeroot libgcc-5-dev libgmp10 libgomp1 libisl15 libitm1 liblsan0 libmpc3 libmpfr4 libmpx0 libquadmath0 libssl-dev libssl-doc libtsan0 libubsan0 libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic linux-libc-dev make manpages manpages-dev menu patch zlib1g-dev
RUN apt-get install -y libssl-dev libssl-doc libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic zlib1g-dev

RUN apt-get download ros-kinetic-librealsense && \
    dpkg-deb -R ros-kinetic-librealsense*.deb ros-rslib/ && \
    wget https://gist.githubusercontent.com/dizz/404ef259a15e1410d692792da0c27a47/raw/3769e80a051b5f2ce2a08d4ee6f79c766724f495/postinst && \
    chmod +x postinst && \
    cp postinst ros-rslib/DEBIAN && \
    dpkg-deb -b ./ros-rslib/ ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb && \
    dpkg -i ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb && \
    apt-mark hold ros-kinetic-librealsense
RUN apt-get install -y libavformat-dev
RUN apt-get install -y libgbm1 libwacom2
RUN apt-get install -y ros-kinetic-kobuki-driver
RUN apt-get install -y libxcb-util1
RUN apt-get install -y libqt5sql5
RUN apt-get install -y libvtk6-dev
RUN apt-get install -y ros-kinetic-rotate-recovery
RUN apt-get install -y ros-kinetic-image-transport
RUN apt-get install -y ros-kinetic-create-description
RUN apt-get install -y libavahi-core-dev
RUN apt-get install -y libyaml-cpp-dev
RUN apt-get install -y ros-kinetic-std-capabilities
RUN apt-get install -y geoip-database
RUN apt-get install -y libftdi1
RUN apt-get install -y libopenni2-dev
RUN apt-get install -y ros-kinetic-create-node
RUN apt-get install -y ros-kinetic-camera-calibration-parsers
RUN apt-get install -y ros-kinetic-ecl-streams
RUN apt-get install -y ros-kinetic-freenect-camera
RUN apt-get install -y ros-kinetic-freenect-launch
RUN apt-get install -y ros-kinetic-yocs-velocity-smoother
RUN apt-get install -y ros-kinetic-openni2-launch
RUN apt-get install -y ros-kinetic-kobuki-bumper2pc
RUN apt-get install -y libusb-dev
RUN apt-get install -y ros-kinetic-yocs-controllers
RUN apt-get install -y ros-kinetic-astra-launch
RUN apt-get install -y libpulse-dev
RUN apt-get install -y ros-kinetic-kobuki-random-walker
RUN apt-get install -y libconfuse-common
RUN apt-get install -y ros-kinetic-rocon-semantic-version

RUN apt-get install -y ros-kinetic-turtlebot-navigation
RUN apt-get install -y ros-kinetic-turtlebot-gazebo xvfb
RUN /bin/bash -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main\" > /etc/apt/sources.list.d/gazebo-stable.list" && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update
RUN apt-get install -y gazebo7
RUN apt-get install -y ros-kinetic-executive-smach
RUN mkdir -p /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws/src && catkin_init_workspace" && \
    cd /catkin_ws/src && \
    git clone https://github.com/mission-control-ros/mission_control && \
    #git clone https://github.com/ros/executive_smach.git && \
    git clone https://github.com/ros-perception/slam_gmapping.git && \
    cd slam_gmapping && \
    git fetch origin pull/56/head:nodelet_fix && \
    git checkout nodelet_fix && \
    cd .. && \
    /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make"
CMD ['/bin/bash', ' -c "source /catkin_ws/devel/setup.bash && rosrun mission_control start_move_base_in_docker.sh && tail -f /dev/null"']
