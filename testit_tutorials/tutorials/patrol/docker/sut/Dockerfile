FROM testitros/testit_patrol_sut:latest
MAINTAINER Gert Kanter <gert.kanter@taltech.ee>
LABEL Description="TestIt! ROS Testing toolkit learn tutorial SUT image"
COPY patches/*.patch /
RUN for i in /patrol.patch spinning.patch; do patch -p0 < $i; done && \
	  cd /catkin_ws/src && \
	  git clone https://github.com/ingmarliibert/testit-patrol-unknown-object.git && \
	  cd .. && \
	  /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /catkin_ws && catkin_make"
CMD bash
