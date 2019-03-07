#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018,2019 Gert Kanter.
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
    def __init__(self):
        rospy.wait_for_service('testit/bringup')
        rospy.wait_for_service('testit/teardown')
        rospy.wait_for_service('testit/status')
        rospy.wait_for_service('testit/test')
        rospy.wait_for_service('testit/results')
        rospy.wait_for_service('testit/bag')
        rospy.wait_for_service('testit/uppaal/annotate/coverage')
        self.bringup_service = rospy.ServiceProxy('testit/bringup', testit.srv.Command)
        self.teardown_service = rospy.ServiceProxy('testit/teardown', testit.srv.Command)
        self.status_service = rospy.ServiceProxy('testit/status', testit.srv.Command)
        self.test_service = rospy.ServiceProxy('testit/test', testit.srv.Command)
        self.results_service = rospy.ServiceProxy('testit/results', testit.srv.Command)
        self.bag_service = rospy.ServiceProxy('testit/bag', testit.srv.Command)
        self.uppaal_annotate_coverage_service = rospy.ServiceProxy('testit/uppaal/annotate/coverage', testit.srv.Command)

    def call_service(self, service, args):
        try:
            if "pipeline" not in args:
                args.pipeline = []
            response = service("".join(args.pipeline))
            print response
        except rospy.ServiceException, e:
            rospy.logerr("Calling service failed: %s" % e)

    def bringup(self, args):
        rospy.loginfo("Bringing up pipeline(s)...")
        self.call_service(self.bringup_service, args)

    def teardown(self, args):
        rospy.loginfo("Tearing down pipeline(s)...")
        self.call_service(self.teardown_service, args)

    def reload(self, args):
        rospy.loginfo("Reloading configuration to ROS parameter server...")
        testit.testit_common.load_config_to_rosparam(testit.testit_common.parse_yaml(args.config))

    def status(self, args):
        self.call_service(self.status_service, args)

    def test(self, args):
        self.call_service(self.test_service, args)

    def log(self, args):
        rospy.loginfo("Log")

    def results(self, args):
        self.call_service(self.results_service, args)

    def bag(self, args):
        self.call_service(self.bag_service, args)

    def uppaal(self, args):
        # Uppaal subcommands handling here
        self.call_service(self.uppaal_annotate_coverage_service, args)


if __name__ == '__main__':
    rospy.init_node('testit_cmdline', anonymous=True, disable_signals=True)
    rospack = rospkg.RosPack()
    testit_instance = TestIt()
    parser = argparse.ArgumentParser(description="TestIt Command Line Interface")
    subparsers = parser.add_subparsers(help="sub-command help")
    
    parser_reload = subparsers.add_parser("reload", help="reload help")
    parser_reload.add_argument("-c", "--config", action="store", default=rospack.get_path('testit')+'/cfg/config.yaml',
                    help="Configuration file location")
    parser_reload.set_defaults(func=testit_instance.reload)
    parser_bringup = subparsers.add_parser("bringup", help="bringup help")
    parser_bringup.add_argument("pipeline", nargs="*")
    parser_bringup.set_defaults(func=testit_instance.bringup)
    parser_test = subparsers.add_parser("test", help="test help")
    parser_test.set_defaults(func=testit_instance.test)
    parser_teardown = subparsers.add_parser("teardown", help="teardown help")
    parser_teardown.add_argument("pipeline", nargs="*")
    parser_teardown.set_defaults(func=testit_instance.teardown)
    parser_status = subparsers.add_parser("status", help="status help")
    parser_status.set_defaults(func=testit_instance.status)
    parser_results = subparsers.add_parser("results", help="results help")
    parser_results.set_defaults(func=testit_instance.results)
    parser_log = subparsers.add_parser("log", help="log help")
    parser_log.set_defaults(func=testit_instance.log)
    parser_bag = subparsers.add_parser("bag", help="bag help")
    parser_bag.set_defaults(func=testit_instance.bag)
    parser_uppaal = subparsers.add_parser("uppaal", help="uppaal help")
    parser_uppaal.set_defaults(func=testit_instance.uppaal)
    testit.opt = parser.parse_args(rospy.myargv()[1:])
    testit.opt.func(testit.opt)

