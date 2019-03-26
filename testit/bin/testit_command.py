#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019 Gert Kanter.
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
        rospy.wait_for_service('testit/clean')
        rospy.wait_for_service('testit/uppaal/annotate/coverage')
        self.bringup_service = rospy.ServiceProxy('testit/bringup', testit.srv.Command)
        self.teardown_service = rospy.ServiceProxy('testit/teardown', testit.srv.Command)
        self.status_service = rospy.ServiceProxy('testit/status', testit.srv.Command)
        self.test_service = rospy.ServiceProxy('testit/test', testit.srv.Command)
        self.results_service = rospy.ServiceProxy('testit/results', testit.srv.Command)
        self.bag_service = rospy.ServiceProxy('testit/bag', testit.srv.Command)
        self.clean_service = rospy.ServiceProxy('testit/clean', testit.srv.Command)
        self.uppaal_annotate_coverage_service = rospy.ServiceProxy('testit/uppaal/annotate/coverage', testit.srv.Command)

    def call_service(self, service, args, callback=None):
        try:
            if "pipeline" not in args:
                args.pipeline = []
            # Enclose in quotes if there is a space
            for i, pipeline in enumerate(args.pipeline):
                if " " in pipeline:
                    args.pipeline[i] = "\"" + pipeline + "\""
            response = service(" ".join(args.pipeline))
            verbose = 0
            if args.verbose is not None:
                verbose = args.verbose
            if (response.result and verbose > 0) and response.result:
                rospy.loginfo(response)
            elif not response.result:
                rospy.logerr(response)
            if callback is not None and response.result:
                callback(response, args)
        except rospy.ServiceException, e:
            rospy.logerr("Calling service failed: %s" % e)

    def clean(self, args):
        rospy.loginfo("Cleaning workspace(s)...")
        if args.all:
            args.pipeline.insert(0, "--all")
        self.call_service(self.clean_service, args)

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
        rospy.loginfo("Starting testing...")
        args.pipeline = args.scenario
        if args.blocking:
            args.pipeline.insert(0, "--blocking")
        self.call_service(self.test_service, args)

    def log(self, args):
        rospy.loginfo("Log")

    def results(self, args):
        if args.xml_sys_out:
            args.pipeline = ['--xml-sys-out']
        self.call_service(self.results_service, args, self.results_callback)

    def results_callback(self, response, args):
        if "output" in args and args.output != "":
            # "--output" specified
	    try:
		rospy.loginfo("Writing results to '%s'" % args.output)
		with open(args.output, 'w') as outfile:
		    outfile.write(response.message)
	    except Exception as e:
		import traceback
		traceback.print_exc()

    def bag(self, args):
        self.call_service(self.bag_service, args)

    def uppaal(self, args):
        def unrecognized(args):
            rospy.logerr("Unrecognized subcommand (%s)!" % args)
        # Uppaal subcommands handling
        if len(args.command) == 2:
            if args.command[0] == "":
                # placeholder for custom
                rospy.logerr("Custom command placeholder")
            if args.command[0] == "extract":
                if "fail" in args.command[1]:
                    rospy.loginfo("Executing Uppaal TA trace failure extraction...")
                else:
                    unrecognized(args)
        elif len(args.command) == 1:
            if args.command[0] == "extract":
                rospy.loginfo("Usage: testit_command.py uppaal extract failure")
            elif args.command[0] == "annotate":
                if args.file == "":
                    rospy.logerr("Please specify the file filter for which to optimize the scenario (-f)!")
                else:
                    rospy.loginfo("Executing Uppaal TA model annotation...")
                    args.pipeline = args.file
                    self.call_service(self.uppaal_annotate_coverage_service, args)
            else:
                unrecognized(args)
        else:
            unrecognized(args)


if __name__ == '__main__':
    rospy.init_node('testit_cmdline', anonymous=True, disable_signals=True)
    rospack = rospkg.RosPack()
    testit_instance = TestIt()
    parser = argparse.ArgumentParser(description="TestIt Command Line Interface")
    parser.add_argument("-v", "--verbose", action="count", help="Verbosity level")
    subparsers = parser.add_subparsers(help="sub-command help")
    
    parser_clean = subparsers.add_parser("clean", help="Clean workspaces from old results")
    parser_clean.add_argument("-a", "--all", action="store_true", default=False, help="Delete results from daemon")
    parser_clean.add_argument("pipeline", nargs="*")
    parser_clean.set_defaults(func=testit_instance.clean)
    parser_reload = subparsers.add_parser("reload", help="reload help")
    parser_reload.add_argument("-c", "--config", action="store", default=rospack.get_path('testit')+'/cfg/config.yaml',
                    help="Configuration file location")
    parser_reload.set_defaults(func=testit_instance.reload)
    parser_bringup = subparsers.add_parser("bringup", help="bringup help")
    parser_bringup.add_argument("pipeline", nargs="*")
    parser_bringup.set_defaults(func=testit_instance.bringup)
    parser_test = subparsers.add_parser("test", help="test help")
    parser_test.add_argument("-b", "--blocking", action="store_true", default=False, help="Block until finished")
    parser_test.add_argument("scenario", nargs="*")
    parser_test.set_defaults(func=testit_instance.test)
    parser_teardown = subparsers.add_parser("teardown", help="teardown help")
    parser_teardown.add_argument("pipeline", nargs="*")
    parser_teardown.set_defaults(func=testit_instance.teardown)
    parser_status = subparsers.add_parser("status", help="status help")
    parser_status.set_defaults(func=testit_instance.status)
    parser_results = subparsers.add_parser("results", help="Output the results")
    parser_results.add_argument("-o", "--output", action="store", default='', help="Optional file to write results")
    parser_results.add_argument("-x", "--xml-sys-out", action="store_true", default=False, help="Add system-out to results")
    parser_results.set_defaults(func=testit_instance.results)
    parser_log = subparsers.add_parser("log", help="log help")
    parser_log.set_defaults(func=testit_instance.log)
    parser_bag = subparsers.add_parser("bag", help="bag help")
    parser_bag.set_defaults(func=testit_instance.bag)
    parser_uppaal = subparsers.add_parser("uppaal", help="Uppaal TA related commands")
    parser_uppaal.add_argument("command", action="store", nargs="+", help="Uppaal subcommands")
    parser_uppaal.add_argument("-f", "--file", action="store", default="", help="Specify the file to work with")
    parser_uppaal.set_defaults(func=testit_instance.uppaal)
    testit.opt = parser.parse_args(rospy.myargv()[1:])
    testit.opt.func(testit.opt)

