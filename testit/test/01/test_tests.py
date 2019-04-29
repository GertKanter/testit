import rospy
import pytest
import testit.testit_daemon
import subprocess

def test_bringup_and_test():
    process = subprocess.Popen("/bin/bash -c 'rosrun testit testit_command.py bringup && rosrun testit testit_command.py -v test --blocking \"Scenario #1\"' && sleep 2", stdout=subprocess.PIPE, shell=True)
    out, err = process.communicate()
    assert "result: True" in out
    assert process.returncode == 0
