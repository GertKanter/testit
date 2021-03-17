#!/bin/bash
if [ "$#" -ne 2 ]; then
  echo "Usage"
  echo "====="
  echo "$0 host_project_directory launch_file_name"
  echo "host_project_directory - The directory outside Docker environment where the TestIt project is located."
  echo "E.g., $0 /home/user/catkin_ws/src/my_testit_project testit.launch"
  exit
fi
# -v \"$1/output\":\"/output\" 
CMD="docker run --rm -v //var//run//docker.sock:/var/run/docker.sock -v \"$1\":\"/catkin_ws/src/testit_project\" --name testit_daemon -t testitros/testit:latest /bin/bash -c \"source /catkin_ws/devel/setup.bash && catkin_make && roslaunch /catkin_ws/src/testit_project/launch/$2\""
echo $CMD
$CMD
