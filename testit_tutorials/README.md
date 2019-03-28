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

##### Pipelines

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

##### Configuration

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
#### TestIt CLI commands
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
And you should see the following at the console
```
[INFO] [1553777577.408230]: [Pipeline #1] Executing SUT to run...
[INFO] [1553777577.408745]: [Pipeline #1] Executing "docker run --rm --net=rosnetwork --env ROS_HOSTNAME=sut1 --env ROS_MASTER_URI=http://sut1:11311 --name sut1 -dt testit_tb_sut:latest /bin/bash -c "source /catkin_ws/devel/setup.bash && rosrun mission_control run_example_in_docker.sh && tail -f /dev/null""
c2fb01cd54cae6c5601919927eba7305a756ce36f4ec5a624caf5139279df805
[WARN] [1553777577.908604]: Test 'Scenario #2' waiting for a free pipeline...
[INFO] [1553777578.646075]: [Pipeline #1] Waiting for delay duration (10)...
[INFO] [1553777588.647103]: [Pipeline #1] Execution done!
[INFO] [1553777588.647765]: [Pipeline #1] Running TestIt...
[INFO] [1553777588.648250]: [Pipeline #1] Executing TestIt to run...
[INFO] [1553777588.648682]: [Pipeline #1] Executing "docker run --rm --volume=$(rospack find testit_tutorials)/tutorials/turtlebot/:/testit/ --net=rosnetwork --env ROS_VERSION=lunar --env ROS_HOSTNAME=testit1 --env ROS_MASTER_URI=http://sut1:11311 --name testit1 -dt testit_tb_testit /bin/bash -c "tail -f /dev/null""
30c7bfad48346361fa71a30c3c571e6c87924bfb01ad6336068a097adc96d4c7
[INFO] [1553777589.528964]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1553777589.530056]: [Pipeline #1] Execution done!
[INFO] [1553777589.530733]: [Pipeline #1] Executing tests in TestIt container...
[INFO] [1553777589.531420]: [Pipeline #1] Launching test 'Scenario #1'
[INFO] [1553777589.532063]: Resolved 'verbose' to default 'False'
[INFO] [1553777589.532722]: [Pipeline #1] Executing oracle...
[WARN] [1553777607.956916]: Test 'Scenario #2' waiting for a free pipeline...
[WARN] [1553777638.007460]: Test 'Scenario #2' waiting for a free pipeline...
[WARN] [1553777668.053762]: Test 'Scenario #2' waiting for a free pipeline...
[WARN] [1553777698.103043]: Test 'Scenario #2' waiting for a free pipeline...
[WARN] [1553777709.533065]: [Pipeline #1] TEST TIMEOUT (False)!
[INFO] [1553777709.533528]: Resolved 'postCommand' to default ''
[INFO] [1553777709.533994]: Resolved 'postSuccessCommand' to default ''
[INFO] [1553777709.534397]: Resolved 'postFailureCommand' to default ''
[INFO] [1553777709.534791]: [Pipeline #1] Stopping TestIt container...
[INFO] [1553777709.535146]: [Pipeline #1] Executing TestIt to stop...
[INFO] [1553777709.535440]: [Pipeline #1] Executing "docker kill testit1"
testit1
[INFO] [1553777710.038547]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1553777710.039427]: [Pipeline #1] Execution done!
[INFO] [1553777710.040026]: [Pipeline #1] Stopping SUT...
[INFO] [1553777710.040460]: [Pipeline #1] Executing SUT to stop...
[INFO] [1553777710.041001]: [Pipeline #1] Executing "docker kill sut1"
sut1
[INFO] [1553777710.476265]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1553777710.476857]: [Pipeline #1] Execution done!
[INFO] [1553777710.477152]: Freeing pipeline 'Pipeline #1'
[INFO] [1553777710.620876]: Acquired pipeline Pipeline #1
[INFO] [1553777710.621936]: [Pipeline #1] Running SUT...
[INFO] [1553777710.622544]: [Pipeline #1] Executing SUT to run...
[INFO] [1553777710.623064]: [Pipeline #1] Executing "docker run --rm --net=rosnetwork --env ROS_HOSTNAME=sut1 --env ROS_MASTER_URI=http://sut1:11311 --name sut1 -dt testit_tb_sut:latest /bin/bash -c "source /catkin_ws/devel/setup.bash && rosrun mission_control run_example_in_docker.sh && tail -f /dev/null""
7ebfb90e875703d26a487a05f5f7512ff37ebcd019b6d33c9711a7b4968e9926
[INFO] [1553777711.562338]: [Pipeline #1] Waiting for delay duration (10)...
[INFO] [1553777721.569522]: [Pipeline #1] Execution done!
[INFO] [1553777721.569951]: [Pipeline #1] Running TestIt...
[INFO] [1553777721.570225]: [Pipeline #1] Executing TestIt to run...
[INFO] [1553777721.570506]: [Pipeline #1] Executing "docker run --rm --volume=$(rospack find testit_tutorials)/tutorials/turtlebot/:/testit/ --net=rosnetwork --env ROS_VERSION=lunar --env ROS_HOSTNAME=testit1 --env ROS_MASTER_URI=http://sut1:11311 --name testit1 -dt testit_tb_testit /bin/bash -c "tail -f /dev/null""
4deca8a96a71599c78bf173ffd310ed9eb42d858e6e241ea1d2dd5594cb1f94f
[INFO] [1553777722.490960]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1553777722.491695]: [Pipeline #1] Execution done!
[INFO] [1553777722.492110]: [Pipeline #1] Executing tests in TestIt container...
[INFO] [1553777722.492528]: Resolved 'testItVolume' to '$(rospack find testit_tutorials)/tutorials/turtlebot/'
[INFO] [1553777722.492936]: Resolved 'resultsDirectory' to 'testit_tests/results/'
[INFO] [1553777722.493352]: Resolved 'testItVolume' to '$(rospack find testit_tutorials)/tutorials/turtlebot/'
[INFO] [1553777722.493884]: Resolved 'resultsDirectory' to 'testit_tests/results/'
[INFO] [1553777722.546719]: [Pipeline #1] Start rosbag recording...
[INFO] [1553777722.547692]: Resolved 'sharedDirectory' to '/testit/'
[INFO] [1553777722.548270]: Executing 'docker exec -d testit1 /bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash && mkdir -p /testit/testit_tests/results/ && cd /testit/testit_tests/results/ && rosbag record --split --max-splits=3 --duration=15 -O "Scenario #2" --regex "(.*)gazebo/(.*)|/odom(.*)|/amcl_pose(.*)" __name:=testit_rosbag_recorder''
[INFO] [1553777722.627524]: [Pipeline #1] rosbag record returned 0
[INFO] [1553777722.628229]: [Pipeline #1] Launching test 'Scenario #2'
[INFO] [1553777722.628582]: Resolved 'verbose' to default 'False'
[INFO] [1553777722.628914]: [Pipeline #1] Executing oracle...
[WARN] [1553777902.629481]: [Pipeline #1] TEST TIMEOUT (False)!
[INFO] [1553777902.630189]: [Pipeline #1] Stop rosbag recording...
killing /testit_rosbag_recorder
killed
[INFO] [1553777907.224321]: [Pipeline #1] Setting privileges...
[INFO] [1553777907.329950]: [Pipeline #1] Merging bag files...
[INFO] [1553777907.356477]: Merging 'Scenario #2_3.bag'... (1/4)
[INFO] [1553777917.833442]: Merging 'Scenario #2_4.bag'... (2/4)
[INFO] [1553777930.206083]: Merging 'Scenario #2_5.bag'... (3/4)
[INFO] [1553777940.863712]: Merging 'Scenario #2_6.bag'... (4/4)
[INFO] [1553777947.807506]: [Pipeline #1] Done!
[INFO] [1553777947.807982]: Resolved 'postCommand' to default ''
[INFO] [1553777947.808375]: Resolved 'postSuccessCommand' to default ''
[INFO] [1553777947.808703]: Resolved 'postFailureCommand' to default ''
[INFO] [1553777947.809301]: [Pipeline #1] Stopping TestIt container...
[INFO] [1553777947.809672]: [Pipeline #1] Executing TestIt to stop...
[INFO] [1553777947.810067]: [Pipeline #1] Executing "docker kill testit1"
testit1
[INFO] [1553777948.374999]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1553777948.375877]: [Pipeline #1] Execution done!
[INFO] [1553777948.376482]: [Pipeline #1] Stopping SUT...
[INFO] [1553777948.377152]: [Pipeline #1] Executing SUT to stop...
[INFO] [1553777948.377948]: [Pipeline #1] Executing "docker kill sut1"
sut1
[INFO] [1553777948.851600]: [Pipeline #1] Waiting for delay duration (0)...
[INFO] [1553777948.852187]: [Pipeline #1] Execution done!
[INFO] [1553777948.852594]: Freeing pipeline 'Pipeline #1'
```
The general breakdown of the output is simple
- we run the SUT stack
- we run the TestIt container
- we run the oracle inside TestIt container
- the oracle will return zero or non-zero (success or fail) or timeout (configured to mean fail in this use case)
- we stop the SUT
- we stop the TestIt container
And we do this for each test scenario (defined in `tests` in the configuration file).

#### Analysis
##### Results JUnit
After the tests are finished, we can get the results as JUnit XML with the following command:
```
rosrun testit testit_command.py -v results
```
And we should see something like this
```
[INFO] [1553778161.307410]: result: True
message: "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n<testsuite tests=\"2\"><testcase name=\"\
  fail\" classname=\"Scenario #1\"><failure message=\"FAILURE\">Failure text</failure></testcase><testcase\
  \ name=\"fail\" classname=\"Scenario #2\"><failure message=\"FAILURE\">Failure text</failure></testcase></testsuite>\n"
```
In case we want to get the XML into a file, we can use the command:
```
rosrun testit testit_command.py results -o results.xml
```
And we should see
```
[INFO] [1553784715.702665]: Writing results to 'results.xml'
```
And we can use this result for example in Jenkins build job.

##### Rosbag
As we have configured test "Scenario #2" to record a rosbag in case of test failure we can use a TestIt CLI command to retrieve it from the pipeline.

The files are stored at the location specified in the configuration. In this use case, they are stored at the `testit_tests/results` directory

```
testit_tests/
├── 01
│   └── oracle
│       └── oracle.py
├── 02
│   └── oracle
│       └── oracle.py
└── results
    └── Scenario #2.bag
```
We can now use the following command to retrieve the bag file from the pipeline workspace into the daemon data directory (`dataDirectory` in configuration):
```
rosrun testit testit_command.py bag collect
```
And we should see an output like this
```
[INFO] [1553785688.186251]: Copying files from pipelines...
[INFO] [1553785688.211655]: Copying files from pipeline 'Pipeline #1'...
[INFO] [1553785688.239864]: Executing command 'cp /home/user/testit_catkin/src/testit/testit_tutorials/tutorials/turtlebot/testit_tests/results/*bag /home/user/testit_catkin/src/testit/testit/data/'
[INFO] [1553785688.580736]: Done!
```
Now we can use `rosbag play` command to replay the failed scenario to analyze what went wrong.
