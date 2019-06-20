#!/bin/bash
if [ "$#" -ne 2 ]; then
  echo "Usage: rosrun testit run_daemon_docker.sh testit_project_directory testit_launch_file"
  echo "E.g., rosrun testit run_daemon_docker.sh /home/user/catkin_ws/src/my_testit_project testit.launch"
  exit
fi
docker run --rm -v /var/run/docker.sock:/var/run/docker.sock -v "$1":"$1" --name testit_daemon -t testitros/testit:latest /bin/bash -c "source /catkin_ws/devel/setup.bash && roslaunch $1/launch/$2"
