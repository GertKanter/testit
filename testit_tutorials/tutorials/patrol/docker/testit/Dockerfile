FROM testitros/testit_patrol_testit:latest
LABEL Description="TestIt! ROS Testing toolkit docker image for testit_patrol_learn"

RUN cd /catkin_ws/src/testit && \
    git remote set-url origin https://github.com/ingmarliibert/testit.git && \
    git checkout develop && \
    git pull && \
    cd testit_learn && \
    rosdep install --from-paths . --ignore-src --rosdistro=melodic -y && \
    cd ../../testit_dtron && \
    git remote set-url origin https://github.com/ingmarliibert/testit_dtron.git && \
    git pull && \
    git checkout master && \
    cd ../.. && \
    /bin/bash -c "source /catkin_ws/devel/setup.bash && cd /catkin_ws/ && catkin_make"
CMD bash
