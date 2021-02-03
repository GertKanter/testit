import rosparam
import rospy
import rosnode
import pytest
import testit.testit_daemon
import subprocess


def run_scenario(mode, job, args=['--blocking'], outputs=["result: True", "{mode}"]):
    cmd = "/bin/bash -c 'rosrun testit testit_command.py bringup && rosrun testit testit_command.py -v %s"
    for arg in args:
        cmd += " "
        cmd += arg
    cmd += " \"%s\"' && sleep 2"

    process = subprocess.Popen(cmd % (mode, job), stdout=subprocess.PIPE, shell=True)
    out, err = process.communicate()

    for output in outputs:
        assert output.replace('{mode}', mode).replace('{job}', job) in out
    assert process.returncode == 0


def test_bringup_and_test():
    run_scenario("test", "Test")


def test_explorer():
    run_scenario("explore", "Explore")


def test_learn():
    run_scenario("learn", "Learn", args=[])


def test_model_refine():
    run_scenario("refine-model", "Refine")
