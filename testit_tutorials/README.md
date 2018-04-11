TestIt: a Scalable Long-Term Autonomy Testing Toolkit for ROS tutorials
=======================================================================

## Turtlebot navigation
### Prerequisites
#### Dependencies
You need to have [Docker](https://www.docker.com/) installed for this tutorial.
#### Turtlebot docker image
You need to have both SUT (System Under Test) and TestIt Turtlebot docker images installed on your machine. To do that we use
```
rosrun testit_tutorials build_turtlebot_docker_containers.sh
```
### Execution
#### Daemon
To run the the tutorial use
```
roslaunch testit_tutorials turtlebot.launch
```
If you see an error such as
```
[turtlebot.launch] is neither a launch file in package [testit_tutorials] nor is [testit_tutorials] a launch file name
The traceback for the exception was written to the log file
```
then you need to source the catkin workspace before running the command. To do this, for example, if the TestIt package is at `~/catkin_ws/src/testit` and you have used `catkin_make` in the workspace you should run
```
. ~/catkin_ws/devel/setup.bash
roslaunch testit_tutorials turtlebot.launch
```
#### TestIt Commands
If you see something like
```
[INFO] [1521626203.463729]: Loading configuration from /home/user/catkin_ws/src/testit/testit_tutorials/tutorials/turtlebot/cfg/config.yaml...
[INFO] [1521626203.550807]: TestIt daemon started...
```
then you can run TestIt CLI commands to run the tests. To execute the tests, we need to bring up the server(s) that the tests are executed at. For that, we use
```
rosrun testit testit_command.py bringup
```
And you should see the following at the console
```
[INFO] [1521626468.757544]: Start all pipelines...
[INFO] [1521626468.761876]: [Pipeline #1] Setting state to BRINGUP
[INFO] [1521626468.762480]: Pipeline #1 starting...
[INFO] [1521626468.763255]: [Pipeline #1] Executing bringup SUT...
[INFO] [1521626468.768747]: [Pipeline #1] Done!
[INFO] [1521626468.770127]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1521626468.770874]: [Pipeline #1] Waiting for the bringup to finish...
[INFO] [1521626469.766326]: ...
[INFO] [1521626469.772620]: [Pipeline #1] Done!
[INFO] [1521626469.774256]: [Pipeline #1] Executing bringup TestIt...
[INFO] [1521626469.780159]: [Pipeline #1] Done!
[INFO] [1521626469.781330]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1521626469.781843]: [Pipeline #1] Waiting for the bringup to finish...
[INFO] [1521626470.783518]: [Pipeline #1] Done!
[INFO] [1521626471.771646]: Pipeline #1 finished with True
```
Next, we can run the tests by using
```
rosrun testit testit_command.py test
```
