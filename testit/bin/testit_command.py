#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018 Gert Kanter.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Gert Kanter

import rospy
import rospkg
import argparse
import testit.testit_common
import testit.srv

class TestIt:
    def __init__(self, opt):
        self.opt = opt
        rospy.wait_for_service('testit/bringup')
        rospy.wait_for_service('testit/teardown')
        rospy.wait_for_service('testit/status')
        rospy.wait_for_service('testit/test')
        rospy.wait_for_service('testit/results')
        rospy.wait_for_service('testit/bag')
        self.bringup_service = rospy.ServiceProxy('testit/bringup', testit.srv.Command)
        self.teardown_service = rospy.ServiceProxy('testit/teardown', testit.srv.Command)
        self.status_service = rospy.ServiceProxy('testit/status', testit.srv.Command)
        self.test_service = rospy.ServiceProxy('testit/test', testit.srv.Command)
        self.results_service = rospy.ServiceProxy('testit/results', testit.srv.Command)
        self.bag_service = rospy.ServiceProxy('testit/bag', testit.srv.Command)

    def execute(self, command):
        """Execute the command.

        command - start runs the pipeline(s) bringup(s)
        """
        getattr(self, command)()

    def call_service(self, service):
        try:
            response = service("".join(self.opt.pipeline))
            print response
        except rospy.ServiceException, e:
            rospy.logerr("Calling service failed: %s" % e)

    def bringup(self):
        rospy.loginfo("Bringing up pipeline(s)...")
        self.call_service(self.bringup_service)

    def teardown(self):
        rospy.loginfo("Tearing down pipeline(s)...")
        self.call_service(self.teardown_service)

    def reload(self):
        rospy.loginfo("Reloading configuration to ROS parameter server...")
        testit_common.load_config_to_rosparam(testit_common.parse_yaml(opt.config))

    def status(self):
        self.call_service(self.status_service)

    def test(self):
        self.call_service(self.test_service)

    def results(self):
        self.call_service(self.results_service)

    def bag(self):
        self.call_service(self.bag_service)


if __name__ == '__main__':
    rospy.init_node('testit_cmdline', anonymous=True, disable_signals=True)
    rospack = rospkg.RosPack()
    parser = argparse.ArgumentParser(description="TestIt Command Line Interface")
    parser.add_argument("command", choices=["bringup", "test", "teardown", "status", "results", "log", "bag", "reload", "report"])
    parser.add_argument("-c", "--config", action="store", default=rospack.get_path('testit')+'/cfg/config.yaml',
                    help="Configuration file location")
    parser.add_argument("-d", "--docker", action="store", default='testitros/testit:latest',
                    help="TestIt docker image tag")
    parser.add_argument("pipeline", nargs="*")
    opt = parser.parse_args(rospy.myargv()[1:])
    
    testit = TestIt(opt)
    testit.execute(opt.command)
