TestIt: a Scalable Long-Term Autonomy Testing Toolkit for ROS tutorials
=======================================================================

## Turtlebot navigation
### Dependencies
You need to have [Docker](https://www.docker.com/) installed for this tutorial.
### Execution
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
