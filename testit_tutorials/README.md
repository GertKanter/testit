TestIt: a Scalable Long-Term Autonomy Testing Toolkit for ROS tutorials
=======================================================================

## Turtlebot navigation
### Description
#### Overview
This tutorial demonstrates a typical use case for testing a mobile robot. We use Gazebo simulator with the Turtlebot model as the mobile robot we wish to test. We use the standard ROS navigation stack for navigation. Navigation goals are given by [`mission_control_ros`](https://github.com/mission-control-ros/mission_control/) package. The system under test (SUT) is configured to start the simulation and start navigating without launching specific test scenarios.

#### Files
The directory structure is as follows: 
```
.
└── turtlebot
    ├── cfg
    │   └── config.yaml
    ├── docker
    │   ├── sut
    │   │   └── Dockerfile
    │   └── testit
    │       └── Dockerfile
    ├── scripts
    │   └── build_turtlebot_docker_containers.sh
    └── testit_tests
        ├── 01
        │   └── oracle
        │       └── oracle.py
        └── 02
            └── oracle
                └── oracle.py
```
The `cfg` directory holds the `config.yaml` file which is the TestIt configuration file. This file defines how TestIt tests the system under test (SUT).

In the [`docker`](tutorials/turtlebot/docker) directory, we can see two directories - `sut` and `testit`. The `sut` Dockerfile describes the system under test (the robot software we want to test). The `testit` Dockerfile defines the TestIt configuration for this particular project (e.g., in case you need to extend the regular functionality of TestIt with some extra packages).

The `scripts` directory has the script for building the Docker containers.

The test scenarios for testing the software are in the `testit_tests` directory.

#### Configuration file (config.yaml)

The configuration file is divided into three sections: `tests`, `configuration` and `pipelines`.

##### Tests

The tests section looks like this:
```yaml
tests:
  - tag: "Scenario #1" # identifier for reporting
    pipeline: "" # empty for any
    launch: "" # how to execute this test (run command) in TestIt container, if empty, then assumed that test is not explicitly executed (already started at runSUT and oracle is used to determine pass/fail)
    oracle: "/testit/testit_tests/01/oracle/oracle.py" # determining whether pass/fail, if empty = "launch" execution result will be used to determine pass/fail
    timeout: 120 # time in seconds for timeout (0 for no timeout)
    timeoutVerdict: False # if timeout occurs, declare the test as this (False = fail, True = success)
    bagEnabled: False # True=rosbag record, False=don't bag
    bagMaxSplits: "" # empty = use default
    bagDuration: "" # empty = use default
    bagTopicRegex: "" # empty = use default (specified in configuration)
    bagTopicExcludeRegex: "" # empty = use default (specified in configuration)
  - tag: "Scenario #2" # identifier for reporting
    pipeline: "" # empty for any
    launch: "" # how to execute this test (run command) in TestIt container, if empty, then assumed that test is not explicitly executed (already started at runSUT and oracle is used to determine pass/fail)
    oracle: "/testit/testit_tests/02/oracle/oracle.py" # determining whether pass/fail, if empty = "launch" execution result will be used to determine pass/fail
    timeout: 180 # time in seconds for timeout (0 for no timeout)
    timeoutVerdict: False # if timeout occurs, declare the test as this (False = fail, True = success)
    bagEnabled: True # True=rosbag record, False=don't bag
    bagMaxSplits: "" # empty = use default
    bagDuration: "" # empty = use default
    bagTopicRegex: "" # empty = use default (specified in configuration)
    bagTopicExcludeRegex: "" # empty = use default (specified in configuration)
```
Here we can see that there are two test scenarios (`Scenario #1` and `Scenario #2`). In the first scenario, we aim to assert that the robot will reach a position where its x coordinate is less than 11.0 meters (`oracle.robot.pose['position']['x'] < 11.0`). The oracle that asserts it looks like this:
```python
import rospy
import testit_oracles.testit_gazebo
import sys

if __name__ == "__main__":
    rospy.init_node("testit_tb_tutorial")
    oracle = testit_oracles.testit_gazebo.GazeboOracle("mobile_base")
    rate = rospy.Rate(2) # 2 Hz
    while not rospy.is_shutdown():
        if oracle.callback_received and oracle.robot.pose['position']['x'] < 11.0:
            sys.exit(0) # success
        rate.sleep()
```
TestIt determines whether the test was successful or not by the exit code. In case of success, zero is expected and any other value is interpreted as a test failure.

The second scenario is designed to always fail as the oracle tries to assert that the robot will reach pose with x coordinate over 11000.0 meters (`oracle.robot.pose['position']['x'] > 11000.0`).

TestIt supports recording rosbags of the test executions and will by default only keep the files if the test fails. This aids in failure analysis - you can use `rosbag play` to see what happened afterwards.

The first scenario is configured to not record the bag files with the failure but the second scenario is configured to record the bag files (`bagEnabled: True`).

##### pipelines

TestIt is uses the notion of pipelines as the testing unit. A pipeline consists of the SUT (the software that is being tested) and TestIt (the set of tools that simplifies integration testing).

The `pipelines` section configures the pipelines to
- bring up the SUT server (optional)
- run the SUT
- stop the SUT
- stop the SUT server (optional)
- bring up the TestIt server (optional)
- run the tests in the TestIt container
- stop the TestIt container
- stop the TestIt server

##### configuration

The `configuration` section defines the default values for pipelines and tests and also some general parameters for TestIt daemon.

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
