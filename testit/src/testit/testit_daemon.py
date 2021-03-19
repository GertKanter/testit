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
import testit.srv
import rosparam
import rospkg

from std_msgs.msg import Bool

import testit_common
import threading
import time
import sys
import re
import subprocess
import testit.junit
import io
import xml.etree.ElementTree
import os
import rosbag
import testit_uppaal
import tempfile
import testit_optimizer
import uuid


class TestItDaemon:
    def __init__(self):
        self.output_log = [] # [(timestamp, message), ...]
        rospy.Service('testit/bringup', testit.srv.Command, self.handle_bringup)
        rospy.Service('testit/teardown', testit.srv.Command, self.handle_teardown)
        rospy.Service('testit/status', testit.srv.Command, self.handle_status)
        rospy.Service('testit/test', testit.srv.Command, self.handle_test)
        rospy.Service('testit/learn', testit.srv.Command, self.handle_learn)
        rospy.Service('testit/results', testit.srv.Command, self.handle_results)
        rospy.Service('testit/bag/collect', testit.srv.Command, self.handle_bag_collect)
        rospy.Service('testit/coverage', testit.srv.Command, self.handle_coverage)
        rospy.Service('testit/uppaal/annotate/coverage', testit.srv.Command, self.handle_uppaal_annotate_coverage)
        rospy.Service('testit/uppaal/extract/failure', testit.srv.Command, self.handle_uppaal_extract_failure)
        rospy.Service('testit/clean', testit.srv.Command, self.handle_clean)
        rospy.Service('testit/shutdown', testit.srv.Command, self.handle_shutdown)
        rospy.Service('testit/credits', testit.srv.Command, self.handle_credits)
        rospy.Service('testit/optimize', testit.srv.Command, self.handle_optimize_log_scenario)
        rospy.Service('testit/online', testit.srv.Command, self.handle_online_test)
        rospy.Service('testit/log', testit.srv.Command, self.handle_log)

        self.initialize()

    def initialize(self):
        self.load_config_from_file()
        self.threads = {}
        self.test_threads = {}
        self.testing = False
        self.call_result = {}
        self.configuration = rospy.get_param('testit/configuration', None)
        if self.configuration is None:
            rospy.logerror("No configuration defaults defined in configuration!")
            sys.exit(-1)
        tests = rospy.get_param('testit/tests', [])
        tests += rospy.get_param('testit/jobs', [])
        rospy.set_param('testit/tests', tests)
        rospy.set_param('testit/jobs', tests)
        tests = self.rosparam_list_to_dict(rospy.get_param('testit/tests', {}), 'tag')
        self.tests = self.set_defaults(tests, self.configuration)
        self.tests = self.substitute_replacement_values(self.tests)
        if self.tests is None:
            rospy.logerror("No tests defined in configuration!")
            sys.exit(-1)
        self.pipelines = self.rosparam_list_to_dict(rospy.get_param('testit/pipelines', None), 'tag')
        if self.pipelines is None:
            rospy.logerror("No pipelines defined in configuration!")
            sys.exit(-1)
        self.pipelines = self.substitute_replacement_values(
            self.set_defaults(self.pipelines,
                              self.configuration))
        self.output_timestamp = rospy.Time.now()

    def substitute_replacement_values(self, params, auxiliary={}, regex='(\[\[.*?\]\])', replacement_index=2):
        """
        Substitute the values wrapped with '[[]]' with the key value inside the brackets.

        In case the substitution is not possible within params, try to find a substitution in auxiliary dictionary
        
        Arguments:
        params -- dict, with potential values to substitute
        auxiliary -- dict with auxiliary values to look for substitution values
        """
        for param in params:
            for key in params[param]:
                m = re.findall(regex, str(params[param][key]))
                if m is not None:
                    for replacement in m:
                        substitution = params[param].get(
                            replacement[replacement_index:-replacement_index if replacement_index > 0 else None], None)
                        if substitution is not None:
                            if type(params[param][key]) == str:
                                params[param][key] = params[param][key].replace(replacement, substitution, 1)
                            else:
                                for i in range(len(params[param][key])):
                                    params[param][key][i] = params[param][key][i].replace(replacement, substitution, 1)
                        else:
                            # Unable to find substitution on the same dictionary level, try auxiliary dictionary
                            if len(auxiliary) > 0:
                                substitution = auxiliary.get(replacement[
                                                             replacement_index:-replacement_index if replacement_index > 0 else None],
                                                             None)
                                if substitution is not None:
                                    if type(params[param][key]) == str:
                                        params[param][key] = params[param][key].replace(replacement, substitution, 1)
                                    else:
                                        for i in range(len(params[param][key])):
                                            params[param][key][i] = params[param][key][i].replace(replacement,
                                                                                                  substitution, 1)
                                else:
                                    if replacement != "[[testUuid]]":
                                        rospy.logwarn(
                                            "Unable to ground substition '%s' key '%s'" % (param, replacement))
        return params

    def set_defaults(self, params, defaults):
        for param in params:
            for key in params[param]:
                try:
                    if params[param][key] == '' and defaults[key] != '':
                        # set default
                        params[param][key] = defaults[key]
                except:
                    pass
        return params

    def rosparam_list_to_dict(self, param, key):
        if param is None:
            return
        return_value = {}
        for item in param:
            return_value[item[key]] = item
        return return_value

    def get_configuration_schema(self):
        rospack = rospkg.RosPack()
        filename = rospack.get_path('testit') + '/cfg/config.yaml'
        self.log("Resolved schema file to '%s'!" % filename, False, "info")
        try:
            return testit_common.parse_yaml(filename)
        except Exception as e:
            import traceback
            traceback.print_exc()
        return None

    def add_test_to_config_file(self, dictionary):
        filename = rospy.get_param('~config')
        schema = self.get_configuration_schema()
        filtered = {'tests': [{}]}
        for key in dictionary['tests'][0]:  # TODO support any schema check not just tests
            if key in schema['tests'][0]:
                if dictionary['tests'][0][key] is None:
                    filtered['tests'][0][key] = ""
                else:
                    filtered['tests'][0][key] = dictionary['tests'][0][key]
        data = "  " + "\n  ".join(testit_common.dump_yaml(filtered).split("\n")[1:]).rstrip() + "\n"
        self.log("Writing configuration to '%s'..." % filename, False, "info")
        config_file_data = ""
        try:
            with open(filename, 'r') as infile:
                for line in infile:
                    config_file_data += line
                    if line.startswith("tests:"):
                        config_file_data += data
            with open(filename, 'w') as outfile:
                outfile.write(config_file_data)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return False
        return True

    def load_config_from_file(self):
        filename = rospy.get_param('~config')
        self.log("Loading configuration from " + filename + "...", False, "info")
        testit_common.load_config_to_rosparam(testit_common.parse_yaml(filename))

    def execution_sleep(self, tag, prefix, instance, i=None):
        start_time = rospy.Time.now()
        timeout = self.pipelines[tag][prefix + instance + 'Timeout'] if i is None or type(
            self.pipelines[tag][prefix + instance + 'Timeout']) != list else \
            self.pipelines[tag][prefix + instance + 'Timeout'][i]
        trigger = self.pipelines[tag][prefix + instance + 'FinishTrigger'] if i is None or type(
            self.pipelines[tag][prefix + instance + 'FinishTrigger']) != list else \
            self.pipelines[tag][prefix + instance + 'FinishTrigger'][i]
        while timeout == 0 or (rospy.Time.now() - start_time).to_sec() < timeout:
            if trigger != '-':
                if subprocess.call(trigger, shell=True) == 0:
                    self.log('[%s] Done!' % tag, False, "info")
                    return True
            else:
                self.log('[%s] Done!' % tag, False, "info")
                return True
            time.sleep(1.0)
        self.log('[%s] Timed out!' % tag, False, "err")
        return False

    def single_instance_execution(self, tag, prefix, instance, set_result, i=None):
        command = self.pipelines[tag][prefix + instance] if i is None else self.pipelines[tag][prefix + instance][i]
        self.log("[%s] Command is '%s'" % (tag, command), False, "info")
        if subprocess.call(command, shell=True) == 0:
            self.log('[%s] Done!' % tag, False, "info")
            self.log('[%s] Waiting for delay duration (%s)...' % (tag, self.pipelines[tag][prefix + instance + 'Delay'] if i is None or type(self.pipelines[tag][prefix + instance + 'Delay']) != list else self.pipelines[tag][prefix + instance + 'Delay'][i]), False, "info")
            time.sleep(self.pipelines[tag][prefix + instance + 'Delay'] if i is None or type(
                self.pipelines[tag][prefix + instance + 'Delay']) != list else
                       self.pipelines[tag][prefix + instance + 'Delay'][i])
            self.log('[%s] Waiting for the %s to finish...' % (tag, prefix), False, "info")
            if not self.execution_sleep(tag, prefix, instance, i):
                # Timed out
                if set_result:
                    self.threads[tag]['result'] = False
                return False
            if self.pipelines[tag].get('state', "OFFLINE") != "TEARDOWN":
                if set_result:
                    self.threads[tag]['result'] = True
                return True
            else:
                self.log("Pipeline in TEARDOWN state!", False, "err")
                if set_result:
                    self.threads[tag]['result'] = False
                return False
        else:
            self.log("[%s] Failed to execute %s!" % (tag, instance), False, "err")
            if set_result:
                self.threads[tag]['result'] = False
            return False

    def instance_execution(self, tag, prefix, instance, set_result):
        if type(self.pipelines[tag][prefix + instance]) == str:
            self.log('[%s] Executing %s %s...' % (tag, prefix, instance), False, "info")
            return self.single_instance_execution(tag, prefix, instance, set_result)
        else:
            for i in range(len(self.pipelines[tag][prefix + instance])):
                self.log('[%s] Executing %s %s (%s of %s)...' % (tag, prefix, instance, i + 1, len(self.pipelines[tag][prefix + instance])), False, "info")
                if not self.single_instance_execution(tag, prefix, instance, set_result if set_result and i == len(
                        self.pipelines[tag][prefix + instance]) - 1 else False, i):
                    return False
            return True

    def thread_worker(self, tag, prefix, post_states):
        rospy.logdebug('[%s] thread_worker started!' % tag)
        sut_result = self.instance_execution(tag, prefix, "SUT", False)
        testit_result = self.instance_execution(tag, prefix, "TestIt", True)
        if sut_result and testit_result:
            self.pipelines[tag]['state'] = post_states['True']
            return True
        self.pipelines[tag]['state'] = post_states['False']
        return False

    def multithreaded_command(self, verb, req, prefix, pre_state, post_states, extra_commands=[]):
        rospy.logdebug(verb + " requested")
        pipelines = []
        if req.args == "":
            self.log(verb + " all pipelines...", False, "info")
        else:
            pipelines = set(self.tokenize_arguments(req.args))  # Remove duplicates
            self.log(verb + "ing " + req.args + "...", False, "info")
        matched = False
        for pipe in rospy.get_param('testit/pipelines', []):
            if req.args == '' or pipe['tag'] in pipelines:
                self.log("[%s] Setting state to %s" % (pipe['tag'], pre_state), False, "info")
                self.pipelines[pipe['tag']]['state'] = pre_state
                if prefix == "teardown":
                    # run stop just in case
                    sut_prefix, sut_suffix = self.get_command_wrapper("sutConnection", "ssh", pipe['tag'])
                    testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipe['tag'])
                    self.execute_system(pipe['tag'], 'SUT', 'stop', sut_prefix, sut_suffix)
                    self.execute_system(pipe['tag'], 'TestIt', 'stop', testit_prefix, testit_suffix)
                # Run extra_commands before executing the main command
                for command in extra_commands:
                    command(pipe['tag'])
                self.log(pipe['tag'] + " " + verb.lower() + "ing...", False, "info")
                thread = threading.Thread(target=self.thread_worker, args=(pipe['tag'], prefix, post_states))
                self.threads[pipe['tag']] = {'thread': thread, 'result': None}
                thread.start()
                matched = True
        if not matched:
            self.log("Unable to recognize pipeline!", False, "warn")
        result = True
        message = ""
        while len(self.threads) > 0:
            for thread in self.threads:
                if not self.threads[thread]['result'] is None:
                    msg = '%s finished with %r' % (thread, self.threads[thread]['result'])
                    self.log(msg, False, "info")
                    if not self.threads[thread]['result']:
                        # Report if experienced a failure...
                        message += msg + "\n"
                        self.pipelines[thread][prefix] = False
                        result = False
                    else:
                        # Success!
                        self.pipelines[thread][prefix] = True
                    del self.threads[thread]
                    break
            time.sleep(1.0)
            rospy.loginfo_throttle(15.0, '...')
        return (result, message)

    def remove_bags(self, tag):
        self.log("removing bags from tag = %s" % tag, False, "info")

    def handle_log(self, req):
        rospy.logdebug("Log requested")
        all_entries = False
        message = ""
        if len(req.args) > 0:
            if "--all" in req.args:
                all_entries = True
                req.args = req.args.replace("--all", "", 1)
        result = True
        timestamp = rospy.Time.now()
        try:
            for entry in self.output_log:
                if entry[0] > timestamp or all_entries:
                    message += "[" + str(entry[0]) + "] " + str(entry[1]) + "\r\n"
        except:
            result = False
        if result:
            self.output_timestamp = timestamp
        return testit.srv.CommandResponse(result, message)

    def handle_bringup(self, req):
        result = self.multithreaded_command("Start", req, "bringup", "BRINGUP", {'True': "READY", 'False': "FAILED"})
        return testit.srv.CommandResponse(result[0], result[1])

    def handle_teardown(self, req):
        self.testing = False
        result = self.multithreaded_command("Stop", req, "teardown", "TEARDOWN",
                                            {'True': "OFFLINE", 'False': "OFFLINE"}, extra_commands=[self.remove_bags])
        return testit.srv.CommandResponse(result[0], result[1])

    def handle_status(self, req):
        rospy.logdebug("Status requested")
        message = ""
        result = True
        try:
            for pipeline in self.pipelines:  # dict
                message += "[%s] %s\n" % (
                    self.pipelines[pipeline]['tag'], self.pipelines[pipeline].get('state', "OFFLINE"))
        except:
            result = False
        return testit.srv.CommandResponse(result, message)

    def acquire_pipeline(self, tag):
        """
        Return:
        pipeline tag
        """
        self.log("Acquiring pipeline for test \'%s\'" % tag, False, "info")
        while True:
            for pipeline in self.pipelines:
                if self.pipelines[pipeline].get('state', "OFFLINE") == "READY":
                    # Check whether test thread pool has a higher priority test queued. If yes, then continue sleeping
                    priority = self.tests[tag].get('priority', 0)
                    sleep = False
                    for thread in self.test_threads:
                        thread_tag = self.test_threads[thread]['tag']
                        thread_test_priority = self.tests[thread_tag].get('priority', 0)
                        queued = self.test_threads[thread].get('queued', False)
                        if queued and thread_test_priority > priority:
                            rospy.logwarn(
                                "Higher priority is queued, continuing to sleep for test scenario '%s'..." % tag)
                            sleep = True
                            break
                    if not sleep:
                        self.pipelines[pipeline]['state'] = "BUSY"
                        return pipeline
            time.sleep(0.2)
            rospy.logwarn_throttle(30.0, 'Test \'%s\' (priority %s) waiting for a free pipeline...' % (
                tag, self.tests[tag].get('priority', 0)))

    def single_execute_system(self, pipeline, system, mode, command, i=None):
        self.log("[%s] Executing \"%s\"" % (pipeline, command), False, "info")
        if command is not None and subprocess.call(command, shell=True) == 0:
            delay = self.pipelines[pipeline][mode + system + 'Delay'] if i is None or type(
                self.pipelines[pipeline][mode + system + 'Delay']) != list else \
                self.pipelines[pipeline][mode + system + 'Delay'][i]
            self.log('[%s] Waiting for delay duration (%s)...' % (pipeline, delay), False, "info")
            time.sleep(delay)
            start_time = rospy.Time.now()
            timeout = self.pipelines[pipeline][mode + system + 'Timeout'] if i is None or type(
                self.pipelines[pipeline][mode + system + 'Timeout']) != list else \
                self.pipelines[pipeline][mode + system + 'Timeout'][i]
            trigger = self.pipelines[pipeline][mode + system + 'FinishTrigger'] if i is None or type(
                self.pipelines[pipeline][mode + system + 'FinishTrigger']) != list else \
                self.pipelines[pipeline][mode + system + 'FinishTrigger'][i]
            while self.pipelines[pipeline]['state'] not in ["TEARDOWN", "FAILED", "OFFLINE"] and (
                    timeout == 0 or (rospy.Time.now() - start_time).to_sec() < timeout):
                if trigger != '-':
                    if subprocess.call(trigger, shell=True) == 0:
                        self.log('[%s] Trigger successful!' % pipeline, False, "info")
                        return True
                else:
                    # No trigger means we do not wait for timeout and break immediately
                    self.log('[%s] Execution finished!' % pipeline, False, "info")
                    return True
                rospy.loginfo_throttle(15.0, '[%s] (%s) ..' % (pipeline, mode))
                time.sleep(1.0)
            if self.pipelines[pipeline]['state'] not in ["TEARDOWN", "FAILED", "OFFLINE"]:
                return True
            self.log('[%s] Execution timed out!' % pipeline, False, "info")
        else:
            self.log('[%s] Execution failed!' % pipeline, False, "err")
        return False

    def execute_system(self, pipeline, system, mode, prefix="", suffix=""):
        """
        blocking
        Returns:
        true -- if successful, false otherwise
        """
        self.log("[%s] Executing %s to %s..." % (pipeline, system, mode), False, "info")
        quote_termination = ""
        if type(prefix) == str:
            if prefix != "":
                quote_termination = "'\\''"
            command = "/bin/bash -c '" + prefix[:-1] + quote_termination + self.pipelines[pipeline].get(mode + system,
                                                                                                        None) + suffix[
                                                                                                                :-1] + quote_termination + "'"
            return self.single_execute_system(pipeline, system, mode, command)
        else:
            for i in range(len(prefix)):
                if prefix[i] != "":
                    quote_termination = "'\\''"
                configured_command = self.pipelines[pipeline].get(mode + system, None)
                if type(configured_command) == list:
                    configured_command = configured_command[i]
                command = "/bin/bash -c '" + prefix[i][:-1] + quote_termination + configured_command + suffix[i][
                                                                                                       :-1] + quote_termination + "'"
                if not self.single_execute_system(pipeline, system, mode, command, i):
                    return False
            return True

    def thread_call(self, tag, command):
        self.call_result[tag] = -1  # -1 means timeout
        self.call_result[tag] = subprocess.call(command, shell=True)
        self.log("Thread call finished with: " + str(self.call_result[tag]), False, "info")

    def resolve_configuration_value(self, target_dictionary, pipeline, key, default=None):
        """
        Resolve the configuration value based on - use default if missing.

        Returns:
        The resolved configuration value.
        """
        if key in self.pipelines[pipeline]:
            target_dictionary[key] = self.pipelines[pipeline][key]
            self.log("Resolved '%s' to '%s'" % (key, target_dictionary[key]), False, "info")
        else:
            if target_dictionary.get(key, None) is None:
                if default is not None:
                    target_dictionary[key] = default
                    self.log("Resolved '%s' to default '%s'" % (key, target_dictionary[key]), False, "info")
                else:
                    self.log("Unable to resolve '%s'" % key, False, "err")
                    return None
        return target_dictionary[key]

    def delete_bag_files(self, pipeline, test, prefix, suffix):
        """
        Remove bag files from results.

        Returns:
        True if successful (no errors)
        """
        self.resolve_configuration_value(self.tests[test], pipeline, 'testItVolume')
        self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
        if self.tests[test]['testItVolume'] is not None:
            if self.tests[test]['resultsDirectory'] is not None:
                bags_directory = self.ground_path(
                    self.tests[test]['testItVolume'] + self.tests[test]['resultsDirectory'], prefix, suffix)
                # Handle spaces in tag names
                split_prefix = test.split(" ")
                if len(split_prefix) > 1:
                    split_prefix = split_prefix[0:1] + ["\\ " + x for x in split_prefix[1:]]
                split_prefix = "".join(split_prefix)
                quote_termination = ""
                if prefix != "":
                    quote_termination = "'\\''"
                delete_command = "/bin/bash -c '" + prefix[
                                                    :-1] + quote_termination + "rm -f " + bags_directory + split_prefix + "*bag" + suffix[
                                                                                                                                   :-1] + quote_termination + "'"
                self.log("Executing '%s'" % delete_command, False, "info")
                return True if subprocess.call(delete_command, shell=True) == 0 else False
        return False

    def get_launch(self, mode, launch):
        launch_suffix = ""
        launch_suffix += " && " if launch != "" and mode in ("explore", "refine-model", "learn", "tron") else ""
        if mode == "explore":
            launch_suffix += "rosrun testit_explorer explorer.py"
        elif mode == "refine-model":
            launch_suffix += "(rosrun testit_learn services.py &); rosrun testit_explorer explorer.py"
        elif mode == "learn":
            launch_suffix += "(rosrun testit_learn services.py &); rosrun testit_learn launcher.py"

        return launch + launch_suffix

    def execute_in_testit_container(self, pipeline, test, keep_bags, prefix, suffix):
        """
        Returns:
        True if test successful, False otherwise
        """
        # Generate scenario UUID if needed
        if self.tests[test].get('uuid', None) is None:
            self.tests[test]['uuid'] = str(uuid.uuid4())
            self.tests = self.substitute_replacement_values(self.tests)
            self.log("[%s] Generated UUID is '%s'" % (pipeline, self.tests[test]['uuid']), False, "info")
        # Clear the previous test UUID
        if self.tests[test].get('testUuid', None) is not None:
            self.log("Clearing old", False, "warn")
            self.tests = self.substitute_replacement_values(self.tests,
                                                            auxiliary={self.tests[test]['testUuid']: '[[testUuid]]'},
                                                            regex='(' + self.tests[test]['testUuid'] + ')',
                                                            replacement_index=0)
            self.log(self.tests[test], False, "info")
        # Generate UUID for the test
        self.tests[test]['testUuid'] = str(uuid.uuid4())
        self.tests = self.substitute_replacement_values(self.tests)
        # execute preLaunchCommand, if this returns 0, proceed, if not, fail
        prelaunch_command = self.tests[test].get('preLaunchCommand', None)
        if prelaunch_command is not None:
            self.log("[%s] Executing pre-launch command..." % pipeline, False, "info")
            quote_termination = "'"
            if prefix != "":
                quote_termination = "'\\''"
            command = prefix + "docker exec " + self.pipelines[pipeline][
                'testItContainerName'] + " /bin/bash -c " + quote_termination + "source /catkin_ws/devel/setup.bash && " + prelaunch_command + quote_termination + suffix
            self.log("Executing '%s'" % command, False, "info")
            return_value = subprocess.call(command, shell=True)
            self.log("[%s] Pre-launch command returned %s" % (pipeline, return_value), False, "info")
            if return_value != 0:
                self.log("Pre-launch command failed! Test failed!", False, "err")
                return False
        self.resolve_configuration_value(self.tests[test], pipeline, 'sharedDirectory')
        self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
        bag_return = 1
        bag_enabled = self.tests[test].get('bagEnabled', False)
        mode = self.tests[test].get('mode', 'test')
        if bag_enabled:
            # Delete old rosbags if present
            if not self.delete_bag_files(pipeline, test, prefix, suffix):
                self.log("[%s] Rosbag deletion failed!" % pipeline, False, "warn")

            self.log("[%s] Start rosbag recording..." % pipeline, False, "info")
            max_splits = self.tests[test].get('bagMaxSplits', None)
            if max_splits is None:
                self.log("[%s] bagMaxSplits is not defined, defaulting to 2" % pipeline, False, "warn")
                max_splits = 2
            duration = self.tests[test].get('bagDuration', None)
            if duration is None:
                self.log("[%s] bagDuration is not defined, defaulting to 30" % pipeline, False, "warn")
                duration = 30
            topic_regex = self.tests[test].get('bagTopicRegex', None)
            if topic_regex is None:
                self.log("[%s] bagTopicRegex is not defined, defaulting to 'all'" % pipeline, False, "warn")
                topic_regex = ""
            topics = "-a "
            if topic_regex != "":
                topics = "--regex \"" + str(topic_regex) + "\" "
            topic_exclude = self.tests[test].get('bagTopicExcludeRegex', None)
            if topic_exclude is None:
                self.log("[%s] bagTopicExcludeRegex is not defined, defaulting to ''" % pipeline, False, "warn")
                topic_exclude = ""
            exclude = ""
            if topic_exclude != "":
                exclude = "--exclude \"" + str(topic_exclude) + "\" "
            quote_termination = "'"
            if prefix != "":
                quote_termination = "'\\''"
            command = prefix + "docker exec -d " + self.pipelines[pipeline][
                'testItContainerName'] + " /bin/bash -c " + quote_termination + "source /catkin_ws/devel/setup.bash && mkdir -p " + str(
                self.tests[test]['sharedDirectory']) + str(self.tests[test]['resultsDirectory']) + " && cd " + str(
                self.tests[test]['sharedDirectory']) + str(
                self.tests[test]['resultsDirectory']) + " && rosbag record --split --max-splits=" + str(
                max_splits) + " --duration=" + str(
                duration) + " -O \"" + test + "\" " + exclude + topics + "__name:=testit_rosbag_recorder" + quote_termination + suffix
            self.log("Executing '%s'" % command, False, "info")
            bag_return = subprocess.call(command, shell=True)
            self.log("[%s] rosbag record returned %s" % (pipeline, bag_return), False, "info")
        # Run logger
        if self.tests[test].get('loggerConfiguration', None) is not None:
            self.log("Starting logger...", False, "info")
            quote_termination = "'"
            if prefix != "":
                quote_termination = "'\\''"
            command = prefix + "docker exec -d " + self.pipelines[pipeline][
                'testItContainerName'] + " /bin/bash -c " + quote_termination + "source /catkin_ws/devel/setup.bash && mkdir -p " + str(
                self.tests[test]['sharedDirectory']) + str(self.tests[test]['resultsDirectory']) + " && cd " + str(
                self.tests[test]['sharedDirectory']) + str(
                self.tests[test]['resultsDirectory']) + " && rosrun testit testit_logger.py _config:=" + str(
                self.tests[test]['sharedDirectory']) + str(self.tests[test]['loggerConfiguration']) + " _test:=\"" + \
                      self.tests[test]['tag'].replace(" ", "\\ ") + "\" _log:=" + str(
                self.tests[test]['sharedDirectory']) + str(
                self.tests[test]['resultsDirectory']) + "logger.log" + quote_termination + suffix
            self.log("Executing '%s'" % command, False, "info")
            logger_return = subprocess.call(command, shell=True)
            self.log("[%s] logger returned %s" % (pipeline, logger_return), False, "info")
            self.log("[%s] Setting privileges..." % pipeline, False, "info")
            subprocess.call(prefix + "docker exec -d " + self.pipelines[pipeline][
                'testItContainerName'] + " /bin/bash -c \"chown -R " + self.ground_path("$(id -u)", prefix,
                                                                                        suffix) + ":" + self.ground_path(
                "$(id -g)", prefix, suffix) + " " + str(self.tests[test]['sharedDirectory']) + str(
                self.tests[test]['resultsDirectory']) + "\"" + suffix, shell=True)
        else:
            self.log("Logger not configured ('loggerConfiguration'), skipping logger start!", False, "info")
            if mode in ("explore", "refine-model", "learn"):
                self.log("In mode '" + mode + "' logger must be configured!", False, "err")
                return False

        # launch test in TestIt docker in new thread (if oracle specified, run in detached mode)
        detached = ""
        self.resolve_configuration_value(self.tests[test], pipeline, 'verbose', False)
        if self.tests[test]['oracle'] != "" and not self.tests[test]['verbose']:
            # run in detached
            detached = "-d "
        self.log("[%s] Launching %s \'%s\'" % (pipeline, mode, test), False, "info")
        self.log("[%s] Launch parameter is \'%s\'" % (pipeline, self.tests[test]['launch']), False, "info")
        launch = self.tests[test].get('launch', "")
        finished_publisher = rospy.Publisher('/testit/finished/%s' % test, Bool, queue_size=1)
        start_time = rospy.Time.now()
        if launch != "" or mode in ('learn', 'explore', 'refine-model'):
            quote_termination = "'"
            if prefix != "":
                quote_termination = "'\\''"
            launch = self.tests[test].get('launch', '')
            launch = self.get_launch(mode, launch)
            source = "source /catkin_ws/devel/setup.bash"
            source += " &&" if launch.strip() != "" else ""
            thread_command = prefix + "docker exec " + detached + self.pipelines[pipeline][
                'testItContainerName'] + " stdbuf -i0 -o0 -e0 /bin/bash -c " + quote_termination + source + launch + quote_termination + suffix
            self.log("[%s] Launch command is '%s'" % (pipeline, thread_command), False, "info")
            thread = threading.Thread(target=self.thread_call,
                                      args=('launch' + str(threading.current_thread().ident), thread_command))
            thread.start()
            if not self.tests[test]['verbose'] or self.tests[test][
                'oracle'] == "":  # join only if not verbose or no oracle
                self.log("Joining thread", False, "info")
                thread.join(self.tests[test]['timeout'])
        return_value = False
        rospy.loginfo("Returned from thread with call_result: " + str(
            self.call_result['launch' + str(threading.current_thread().ident)]))
        if (launch == "" or self.call_result['launch' + str(threading.current_thread().ident)] == 0 or detached == "" or
            self.tests[test]['verbose']) and not mode == 'refine-model':
            # command returned success or in verbose mode (run oracle in parallel)
            self.log("[%s] %s PASS!" % (pipeline, mode.upper()), False, "info")
            return_value = True
        elif self.call_result['launch' + str(threading.current_thread().ident)] == -1:
            self.log("[%s] %s TIMEOUT (%s)!" % (pipeline, mode.upper(), self.tests[test]['timeoutVerdict']), False, "warn")
            if self.tests[test]['timeoutVerdict']:
                return_value = True
        else:
            self.log("[%s] %s FAIL!" % (pipeline, mode.upper()), False, "err")

        finished_publisher.publish(Bool(True))
        return return_value

    def get_command_wrapper(self, parameter, command, pipeline, add_quotes=True, add_space=True):
        """
        Returns a list of prefixes if *Connection is a list
        """
        prefix = command + " "
        suffix = "'" if add_quotes else ""
        connection = self.pipelines[pipeline].get(parameter, "-")
        identity = self.pipelines[pipeline].get('identityFile', "-")
        if identity != "-":
            prefix += "-i " + identity + " "
        if type(connection) == str:
            if connection == "-":
                connection = ""
                suffix = ""
            else:
                connection = prefix + connection + (" " if add_space else "") + ("'" if add_quotes else "")
        else:
            suffixes = []
            prefixes = []
            for i in range(len(connection)):
                if connection[i] != "-":
                    prefixes.append(prefix + connection[i] + (" " if add_space else "") + ("'" if add_quotes else ""))
                    suffixes.append("'" if add_quotes else "")
                else:
                    prefixes.append("")
                    suffixes.append("")
            return prefixes, suffixes
        return connection, suffix

    def learn_thread_worker(self, tag, pipeline=None):
        self.log("Learn thread worker: " + tag, False, "info")
        if pipeline is None:
            # TODO if specific pipeline is specified for a test, acquire that specific pipeline
            self.log("Waiting for main thread", False, "info")
            while self.test_threads.get(threading.current_thread().ident, 0) == 0:
                time.sleep(0.01)  # Wait until main thread has created the thread entry in the dictionary
            self.log("Done waiting for main thread", False, "info")

            self.test_threads[threading.current_thread().ident]['queued'] = True
            pipeline = self.acquire_pipeline(tag)  # find a free pipeline (blocking)
            self.test_threads[threading.current_thread().ident]['queued'] = False
            self.log("Acquired pipeline '%s' for test '%s'" % (pipeline, tag), False, "info")
        else:
            # We have to free the pipeline if higher priority tasks are waiting in test_thread pool
            highest_queued_priority = None
            for thread in self.test_threads:
                test = self.test_threads[thread]['tag']
                priority = self.tests[test].get('priority', 0)
                queued = self.test_threads[thread].get('queued', False)
                requested_pipeline = self.tests[test].get('pipeline', "")
                if test != tag and queued and (highest_queued_priority is None or priority > highest_queued_priority):
                    # TODO consider wildcards for pipeline
                    if requested_pipeline == "" or pipeline == requested_pipeline:
                        highest_queued_priority = priority
            self.log("Highest queued priority is %s" % highest_queued_priority, False, "info")
            priority = self.tests[tag].get('priority', 0)
            if highest_queued_priority is not None and highest_queued_priority > priority:
                self.log("Releasing the pipeline to higher priority test...", False, "info")
                self.free_pipeline(pipeline)
                self.log("Pipeline freed!", False, "info")
                self.log("Add to queue again in 5 sec...", False, "info")
                time.sleep(5.0)
                self.tests[tag]['reserved_credits'] -= 1
                self.log("Adding to queue...", False, "info")
                # FIXME this will raise recursion limit exceeded if the project has scenarios with large number of credits or complex priority/queuing!
                self.learn_thread_worker(tag)
                return
            else:
                self.log("Continuing with pipeline %s" % pipeline, False, "info")
        rospy.set_param('testit/pipeline', self.pipelines[pipeline])
        self.tests = self.substitute_replacement_values(self.tests, self.pipelines[pipeline])
        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)

        self.log("[%s] Running TestIt..." % pipeline, False, "info")
        if self.execute_system(pipeline, 'TestIt', 'run', testit_prefix, testit_suffix):
            self.log("[%s] Executing learn in TestIt container..." % pipeline, False, "info")
            self.tests[tag]['test_start_timestamp'] = rospy.Time.now()
            self.tests[tag]['result'] = self.execute_in_testit_container(pipeline, tag, False, testit_prefix,
                                                                         testit_suffix)
            self.tests[tag]['test_end_timestamp'] = rospy.Time.now()
            self.tests[tag]['executor_pipeline'] = pipeline
            # stopTestIt
            self.log("[%s] Stopping TestIt container..." % pipeline, False, "info")
            self.execute_system(pipeline, 'TestIt', 'stop', testit_prefix, testit_suffix)
        else:
            # unable to run TestIt
            self.log("[%s] Unable to run TestIt!" % pipeline, False, "err")
            rospy.sleep(1.0)

    def test_thread_worker(self, tag, keep_bags=False, pipeline=None):
        """
        Arguments:
            tag -- test tag (string)
        """
        self.tests[tag]['reserved_credits'] = self.tests[tag].get('reserved_credits', 0)
        # check whether credits > 0, or return
        self.tests[tag]['credits'] = self.tests[tag].get('credits', 0)
        self.log("Test '%s' has %s credit(s)." % (tag, self.tests[tag]['credits']), False, "info")
        if self.tests[tag]['credits'] - self.tests[tag]['reserved_credits'] <= 0:
            self.log("Test '%s' has no credits! Test not executed!" % tag, False, "info")
            self.test_threads[threading.current_thread().ident]['result'] = False
            if pipeline is not None:
                self.free_pipeline(pipeline)
            return
        self.tests[tag]['reserved_credits'] += 1
        if pipeline is None:
            # TODO if specific pipeline is specified for a test, acquire that specific pipeline
            while self.test_threads.get(threading.current_thread().ident, 0) == 0:
                time.sleep(0.01)  # Wait until main thread has created the thread entry in the dictionary
            self.test_threads[threading.current_thread().ident]['queued'] = True
            pipeline = self.acquire_pipeline(tag)  # find a free pipeline (blocking)
            self.test_threads[threading.current_thread().ident]['queued'] = False
            self.log("Acquired pipeline '%s' for test '%s'" % (pipeline, tag), False, "info")
        else:
            # We have to free the pipeline if higher priority tasks are waiting in test_thread pool
            highest_queued_priority = None
            for thread in self.test_threads:
                test = self.test_threads[thread]['tag']
                priority = self.tests[test].get('priority', 0)
                queued = self.test_threads[thread].get('queued', False)
                requested_pipeline = self.tests[test].get('pipeline', "")
                if test != tag and queued and (highest_queued_priority is None or priority > highest_queued_priority):
                    # TODO consider wildcards for pipeline
                    if requested_pipeline == "" or pipeline == requested_pipeline:
                        highest_queued_priority = priority
            self.log("Highest queued priority is %s" % highest_queued_priority, False, "info")
            priority = self.tests[tag].get('priority', 0)
            if highest_queued_priority is not None and highest_queued_priority > priority:
                self.log("Releasing the pipeline to higher priority test...", False, "info")
                self.free_pipeline(pipeline)
                self.log("Pipeline freed!", False, "info")
                self.log("Add to queue again in 5 sec...", False, "info")
                time.sleep(5.0)
                self.tests[tag]['reserved_credits'] -= 1
                self.log("Adding to queue...", False, "info")
                # FIXME this will raise recursion limit exceeded if the project has scenarios with large number of credits or complex priority/queuing!
                self.test_thread_worker(tag, keep_bags)
                return
            else:
                self.log("Continuing with pipeline %s" % pipeline, False, "info")
        rospy.set_param('testit/pipeline', self.pipelines[pipeline])
        self.tests = self.substitute_replacement_values(self.tests, self.pipelines[pipeline])
        # runSUT
        self.log("[%s] Running SUT..." % pipeline, False, "info")
        sut_prefix, sut_suffix = self.get_command_wrapper("sutConnection", "ssh", pipeline)
        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)
        if self.execute_system(pipeline, 'SUT', 'run', sut_prefix, sut_suffix):
            # runTestIt
            self.log("[%s] Running TestIt..." % pipeline, False, "info")
            if self.execute_system(pipeline, 'TestIt', 'run', testit_prefix, testit_suffix):
                self.log("[%s] Executing tests in TestIt container..." % pipeline, False, "info")
                self.tests[tag]['test_start_timestamp'] = rospy.Time.now()
                self.tests[tag]['credits'] -= 1
                self.tests[tag]['result'] = self.execute_in_testit_container(pipeline, tag, keep_bags, testit_prefix,
                                                                             testit_suffix)
                self.tests[tag]['test_end_timestamp'] = rospy.Time.now()
                self.tests[tag]['executor_pipeline'] = pipeline
                # execute postTest commands
                self.resolve_configuration_value(self.tests[tag], pipeline, 'postTestCommand', "")
                self.resolve_configuration_value(self.tests[tag], pipeline, 'postTestSuccessCommand', "")
                self.resolve_configuration_value(self.tests[tag], pipeline, 'postTestFailureCommand', "")
                if self.tests[tag]['result']:
                    if self.tests[tag]['postTestSuccessCommand'] != "":
                        rospy.loginfo(
                            "Executing post-test success command ('%s')..." % self.tests[tag]['postTestSuccessCommand'])
                        result = subprocess.call(self.tests[tag]['postTestSuccessCommand'], shell=True)
                        if result != 0:
                            self.log("Post-success command failed!", False, "err")
                else:
                    if self.tests[tag]['postTestFailureCommand'] != "":
                        rospy.loginfo(
                            "Executing post-test failure command ('%s')..." % self.tests[tag]['postTestFailureCommand'])
                        result = subprocess.call(self.tests[tag]['postTestFailureCommand'], shell=True)
                        if result != 0:
                            self.log("Post-test failure command failed!", False, "err")
                if self.tests[tag]['postTestCommand'] != "":
                    self.log("Executing post-test command ('%s')..." % self.tests[tag]['postTestCommand'], False, "info")
                    result = subprocess.call(self.tests[tag]['postTestCommand'], shell=True)
                    if result != 0:
                        self.log("Post-test command failed!", False, "err")

                # execute the post commands if credits are zero
                if self.tests[tag]['credits'] == 0:
                    self.resolve_configuration_value(self.tests[tag], pipeline, 'postCommand', "")
                    self.resolve_configuration_value(self.tests[tag], pipeline, 'postSuccessCommand', "")
                    self.resolve_configuration_value(self.tests[tag], pipeline, 'postFailureCommand', "")
                    if self.tests[tag]['result']:
                        if self.tests[tag]['postSuccessCommand'] != "":
                            self.log("Executing post-testing success command ('%s')..." % self.tests[tag]['postSuccessCommand'], False, "info")
                            result = subprocess.call(self.tests[tag]['postSuccessCommand'], shell=True)
                            if result != 0:
                                self.log("Post-testing success command failed!", False, "err")
                    else:
                        if self.tests[tag]['postFailureCommand'] != "":
                            self.log("Executing post-testing failure command ('%s')..." % self.tests[tag]['postFailureCommand'], False, "info")
                            result = subprocess.call(self.tests[tag]['postFailureCommand'], shell=True)
                            if result != 0:
                                self.log("Post-failure command failed!", False, "err")
                    if self.tests[tag]['postCommand'] != "":
                        self.log("Executing post-testing command ('%s')..." % self.tests[tag]['postCommand'], False, "info")
                        result = subprocess.call(self.tests[tag]['postCommand'], shell=True)
                        if result != 0:
                            self.log("Post-test command failed!", False, "err")
                    self.test_threads[threading.current_thread().ident]['result'] = self.tests[tag]['result']
                # stopTestIt
                self.log("[%s] Stopping TestIt container..." % pipeline, False, "info")
                self.execute_system(pipeline, 'TestIt', 'stop', testit_prefix, testit_suffix)
            else:
                # unable to run TestIt
                self.log("[%s] Unable to run TestIt!" % pipeline, False, "err")
                rospy.sleep(1.0)
            # stopSUT
            self.log("[%s] Stopping SUT..." % pipeline, False, "info")
            self.execute_system(pipeline, 'SUT', 'stop', sut_prefix, sut_suffix)
        else:
            # Unable to run SUT
            self.log("[%s] Unable to run SUT!" % pipeline, False, "err")
            rospy.sleep(1.0)
        self.tests[tag]['reserved_credits'] -= 1
        if self.tests[tag]['credits'] > 0:
            self.log("Test '%s' has %s credits remaining! Continuing..." % (tag, self.tests[tag]['credits']), False, "info")
            self.test_thread_worker(tag, keep_bags, pipeline)
        else:
            self.free_pipeline(pipeline)

    def free_pipeline(self, pipeline):
        if self.pipelines[pipeline]['state'] not in ["TEARDOWN", "OFFLINE", "FAILED"]:
            self.log("Freeing pipeline \'%s\'" % pipeline, False, "info")
            self.pipelines[pipeline]['state'] = "READY"

    def nonblocking_test_monitor(self):
        while True:
            sleep = True
            for thread in self.test_threads:
                tag = self.test_threads[thread]['tag']
                if not self.test_threads[thread]['result'] is None:
                    del self.test_threads[thread]
                    # Only set executing to false if all other threads with same tag are not None
                    executing = False
                    for check in self.test_threads:
                        if self.test_threads[check]['tag'] == tag and not self.test_threads[check]['result'] is None:
                            executing = True
                            break
                    if not executing:
                        self.tests[tag]['executing'] = False
                    sleep = False
                    break
            if len(self.test_threads) == 0:
                self.testing = False
                return  # all threads are finished
            if sleep:
                time.sleep(0.5)

    def tokenize_arguments(self, string):
        """
        Tokenize the arguments passed (pipelines, tests) as a string.

        Returns:
        list of string tokens (e.g., pipelines, tests).
        """
        # Regex for quotation marks
        m = re.findall('(["\'].+?["\'])', str(string))
        matches = []
        if m is not None:
            for match in m:
                matches.append(match.replace("\"", "").replace("'", ""))
                string = string.replace(match, "", 1)
        split = string.split(" ")
        for token in split:
            if len(token) > 0:
                matches.append(token)
        return matches

    def handle_clean(self, req):
        rospy.logdebug("Clean requested")
        result = True
        message = ""
        queue = [pipeline for pipeline in self.pipelines]
        if len(req.args) > 0:
            # Determine whether we should clean results from daemon workspace
            if req.args.startswith("--all"):
                clean_daemon = True
                req.args = req.args.replace("--all", "", 1)
                if self.configuration.get('dataDirectory', None) is not None:
                    data_directory = self.ground_path(self.configuration['dataDirectory'], "", "")
                    self.log("Cleaning daemon data directory '%s'..." % data_directory, False, "info")
                    remove_result = subprocess.call("rm -rf " + data_directory + "*", shell=True)
                    if remove_result != 0:
                        self.log("Unable to remove files from '%s'!" % data_directory, False, "err")
                    else:
                        self.log("Done!", False, "info")
                else:
                    self.log("'dataDirectory' is not defined in configuration!", False, "err")
            if len(req.args) > 0:
                queue = []
            pipes = set(self.tokenize_arguments(req.args))  # Remove duplicates
            for pipe in pipes:
                found = False
                for pipeline in self.pipelines:
                    if pipe == self.pipelines[pipeline]['tag']:
                        found = True
                        queue.append(pipe)
                if not found:
                    self.log("Unknown pipeline tag specified '%s'" % pipe, False, "warn")
        if len(queue) > 0:
            self.log("Cleaning results directories from pipelines: %s" % str(queue), False, "info")
            for pipeline in queue:
                testit_volume = self.pipelines[pipeline].get('testItVolume', None)
                results_directory = self.pipelines[pipeline].get('resultsDirectory', None)
                if results_directory is not None:
                    if testit_volume is not None:
                        self.log("Cleaning results directory at '%s'..." % pipeline, False, "info")
                        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)
                        command = testit_prefix + "rm -rf " + self.ground_path(testit_volume + results_directory,
                                                                               testit_prefix,
                                                                               testit_suffix) + "*" + testit_suffix
                        self.log("Executing command '%s'" % command, False, "info")
                        remove_result = subprocess.call(command, shell=True)
                        if remove_result != 0:
                            self.log("Unable to remove files from '%s'!" % data_directory, False, "err")
                        else:
                            self.log("Done!", False, "info")
                    else:
                        self.log("'testItVolume' is not defined in configuration!", False, "err")
                else:
                    self.log("'resultsDirectory' is not defined in configuration!", False, "err")
        else:
            result = False
            message = "Unable to clean workspace!"
        return testit.srv.CommandResponse(result, message)

    def handle_credits(self, req):
        rospy.logdebug("Credits requested")
        result = True
        message = ""
        tokens = self.tokenize_arguments(req.args)
        set_value = None
        if "--set" in tokens:
            index = tokens.index("--set")
            set_value = tokens[index + 1]
            if not str(set_value).isdigit():
                self.log("Credit amount is not numeric!", False, "err")
                result = False
            else:
                set_value = int(set_value)
            del tokens[index + 1]
            del tokens[index]
        if result:
            if len(tokens) == 0:
                tokens = [x for x in self.tests]
            for token in tokens:
                test = self.tests.get(token, None)
                if test is not None:
                    if set_value is not None:
                        self.log("Setting test '%s' credits to %s!" % (test['tag'], set_value), False, "info")
                        self.tests[test['tag']]['credits'] = set_value
                    else:
                        message += self.log("Test '%s' credits: %s" % (test['tag'], test.get('credits', 0)),
                                            True, "info")
        return testit.srv.CommandResponse(result, message)

    def handle_learn(self, req):
        rospy.logdebug("Test requested")
        result = True
        message = ""
        # Create list with tests to execute
        queue = [test for test in self.tests if self.tests[test].get('mode', 'test') == 'learn']

        self.log("Learn scenarios queued: " + str(queue), False, "info")
        for test in queue:  # key
            self.tests[test]['executing'] = self.tests[test].get('executing', False)
            if not self.tests[test]['executing']:
                self.tests[test]['result'] = None
                self.tests[test]['executing'] = True
                threads = 0
                concurrency = self.tests[test].get('concurrency', 1)
                for _ in self.pipelines:
                    thread = threading.Thread(target=self.learn_thread_worker, args=(test,))
                    thread.start()
                    self.test_threads[thread.ident] = {'thread': thread, 'result': None, 'tag': test}
                    threads += 1
                    if concurrency != 0 and threads >= concurrency:
                        break
            else:
                self.log("Learning '%s' is already executing!" % test, False, "err")

        return testit.srv.CommandResponse(result, message)

    def handle_test(self, req):
        rospy.logdebug("Test requested")
        result = True
        message = ""
        # Create list with tests to execute
        mode = "test"
        queue = [test for test in self.tests]
        blocking = False
        keep_bags = False
        no_credit_increment = False
        if len(req.args) > 0:
            # Determine whether to block or not
            if "--keep-bags" in req.args:
                keep_bags = True
                req.args = req.args.replace("--keep-bags", "", 1)
            if "--blocking" in req.args:
                blocking = True
                req.args = req.args.replace("--blocking", "", 1)
            if "--no-credit-increment" in req.args:
                no_credit_increment = True
                req.args = req.args.replace("--no-credit-increment", "", 1)
            if "--test" in req.args:
                mode = "test"
                req.args = req.args.replace("--test", "", 1)
            if "--explore" in req.args:
                mode = "explore"
                req.args = req.args.replace("--explore", "", 1)
            if "--refine-model" in req.args:
                mode = "refine-model"
                req.args = req.args.replace("--refine-model", "", 1)

            if len(req.args.strip()) > 0:
                queue = []
            scenarios = set(self.tokenize_arguments(req.args))  # Remove duplicates
            for scenario in scenarios:
                found = False
                for test in self.tests:
                    if scenario == self.tests[test]['tag']:
                        found = True
                        queue.append(scenario)
                if not found:
                    self.log("Unknown test tag specified '%s'" % scenario, False, "warn")

        queue = list(filter(lambda test: self.tests[test].get('mode', 'test') == mode, queue))
        self.log("Test scenarios queued: " + str(queue), False, "info")
        for test in queue:  # key
            self.tests[test]['executing'] = self.tests[test].get('executing', False)
            if not self.tests[test]['executing']:
                self.tests[test]['result'] = None
                if not no_credit_increment:
                    self.tests[test]['credits'] = self.tests[test].get('credits', 0)
                    if self.tests[test]['credits'] == 0:
                        self.log("Auto incrementing '%s' test credits..." % test, False, "info")
                        self.tests[test]['credits'] += 1
                self.tests[test]['executing'] = True
                threads = 0
                concurrency = self.tests[test].get('concurrency', 1)
                for _ in self.pipelines:
                    thread = threading.Thread(target=self.test_thread_worker, args=(test, keep_bags, None))
                    thread.start()
                    self.test_threads[thread.ident] = {'thread': thread, 'result': None, 'tag': test}
                    threads += 1
                    if concurrency != 0 and threads >= concurrency:
                        break
                if not self.testing:
                    thread = threading.Thread(target=self.nonblocking_test_monitor)
                    thread.start()
                    self.testing = True
            else:
                self.log("Test '%s' is already executing!" % test, False, "err")
        if blocking:
            finished = False
            while not finished:
                finished = True
                for test in queue:
                    executing = self.tests[test].get('executing', False)
                    if executing:
                        finished = False
                        break
                rospy.sleep(0.2)
        return testit.srv.CommandResponse(result, message)

    def handle_results(self, req):
        rospy.logdebug("Results requested")
        message = ""
        result = True
        testsuite = testit.junit.testsuite(tests=len(self.tests))
        output = io.cStringIO.StringIO()
        for test in self.tests:
            testcase = testit.junit.testcase(classname=test)
            start_timestamp = self.tests[test].get('test_start_timestamp', None)
            if start_timestamp is not None:
                testcase.set_timestamp(start_timestamp.to_sec())
            end_timestamp = self.tests[test].get('test_end_timestamp', None)
            if end_timestamp is not None and start_timestamp is not None:
                if end_timestamp > start_timestamp:
                    testcase.set_time((end_timestamp - start_timestamp).to_sec())
            test_result = self.tests[test].get('result', None)
            if test_result is None:
                # skipped or not executed
                skipped = testit.junit.skipped(message="SKIPPED")
                skipped.set_valueOf_("This test has not been executed.")
                testcase.add_skipped(skipped)
                testcase.set_name("skipped")
            else:
                # If "--xml-sys-out" is specified, try to include [scenario_tag]_system_out.xml to <system-out> tag in generated XML
                if req.args == "--xml-sys-out":
                    pipeline = self.tests[test].get('executor_pipeline', None)
                    if pipeline is not None:
                        self.log("Ran in %s " % pipeline, False, "info")
                        # Get XML file from pipeline
                        self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
                        filename = self.tests[test].get("resultsDirectory", "") + self.tests[test][
                            'tag'] + "_system_out.xml"
                        # ground testItVolume path
                        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)
                        path = self.ground_path(self.pipelines[pipeline]['testItVolume'], testit_prefix, testit_suffix)
                        fullname = self.ground_path("\"" + path + filename + "\"", testit_prefix, testit_suffix)
                        if self.pipelines[pipeline].get('testItConnection', "-") != "-":
                            fullname = self.get_temp_filename(pipeline, fullname)
                        # Read the file
                        self.log("Reading from file '%s'" % fullname, False, "info")
                        try:
                            with open(fullname, 'r') as sys_out:
                                data = sys_out.readlines()
                                testcase.add_system_out("".join(data))
                        except Exception as e:
                            pass
                if not test_result:
                    # failed
                    failure = testit.junit.failure(message="FAILURE")
                    failure.set_valueOf_("Failure text")
                    testcase.add_failure(failure)
                    testcase.set_name("fail")
                else:
                    # success
                    testcase.set_name("success")
            testsuite.add_testcase(testcase)
        testsuite.export(output, 0, pretty_print=False)
        message = '<?xml version="1.0" encoding="UTF-8" ?>\n' + output.getvalue() + "\n"
        return testit.srv.CommandResponse(result, message)

    def handle_bag_collect(self, req):
        """
        Collect bags from pipelines to daemon data directory.
        """
        result = True
        message = ""
        self.log("Copying files from pipelines...", False, "info")
        data_directory = self.configuration.get('dataDirectory', None)
        if data_directory is not None:
            data_directory = self.ground_path(data_directory, "", "")
            for pipeline in self.pipelines:
                self.log("Copying files from pipeline '%s'..." % pipeline, False, "info")
                testit_volume = self.pipelines[pipeline].get('testItVolume', None)
                if testit_volume is not None:
                    results_directory = self.pipelines[pipeline].get('resultsDirectory', None)
                    if results_directory is not None:
                        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)
                        bags_directory = self.ground_path(testit_volume + results_directory, testit_prefix,
                                                          testit_suffix)
                        command = testit_prefix + "cp " + bags_directory + "*bag " + data_directory + testit_suffix
                        self.log("Executing command '%s'" % command, False, "info")
                        copy_result = subprocess.call(command, shell=True)
                        if copy_result != 0:
                            self.log("Unable to copy files from pipeline '%s'!" % pipeline, False, "err")
                            result = False
                        else:
                            self.log("Done!", False, "info")
        else:
            self.log("'dataDirectory' is not defined!", False, "err")
            result = False
        return testit.srv.CommandResponse(result, message)

    def shutdown(self):
        rospy.sleep(1)
        rospy.signal_shutdown("Shutting down!")

    def handle_shutdown(self, req):
        rospy.logdebug("Shutdown requested")
        message = "Shutting down!"
        result = True
        thread = threading.Thread(target=self.shutdown)
        thread.start()
        return testit.srv.CommandResponse(result, message)

    def handle_coverage(self, req):
        rospy.logdebug("Coverage results requested")
        message = "coverage message"
        result = True
        return testit.srv.CommandResponse(result, message)

    def ground_path(self, command, prefix, suffix):
        """
        Process paths with bash commands.
        E.g., '$(rospack find testit)/data/' to '/home/user/catkin_ws/src/testit/testit/data/'
        """
        if prefix == "":
            process = subprocess.Popen(['/bin/bash', '-c', 'echo ' + command], stdout=subprocess.PIPE, encoding='utf-8')
        else:
            cmd = prefix + "/bin/bash -c '\\''echo " + command + "'\\''" + suffix
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, encoding='utf-8')
        out, err = process.communicate()
        out = out.replace("\n", "")
        return out

    def annotate_uppaal_transition(self, tree, entry):
        """
        Annotate a single uppaal transition.
        
        Only updates if entry lines

        Arguments:
        tee -- the xml.etree.ElementTree tree
        entry -- a single testit coverage entry

        Returns:
        Tuple (success, Annotated xml.etree.ElementTree)
        """
        assignments = tree.findall("./template/transition//*[@kind='assignment']")
        success = False
        for assignment in assignments:
            failed = False
            for i, variable_dict in enumerate(entry['state']):
                for variable in variable_dict:
                    match = "i_" + entry['name'] + "_" + variable + "=" + str(entry['state'][i][variable])
                    if match not in assignment.text:
                        failed = True
                        break
                if failed:
                    break
            if not failed:
                if "V=" not in assignment.text:
                    assignment.text += ", V=" + str(entry['sum'])
                    success = True
        return (success, tree)

    def get_temp_filename(self, pipeline, filename):
        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "scp", pipeline, False, False)
        temp_filename = tempfile.mkstemp()[1]
        self.log("Temp file is '%s'" % temp_filename, False, "info")
        command = testit_prefix + ":\"" + filename.replace(" ", "\\ ") + "\" " + temp_filename
        self.log("Executing '%s'" % command, False, "info")
        return_value = subprocess.call(command, shell=True)
        if return_value != 0:
            self.log("Unable to copy '%s' from pipeline '%s'!" % (filename, pipeline), False, "err")
            return None
        else:
            return temp_filename

    def get_file_from_pipeline(self, test, pipeline, filename, prefix, suffix):
        self.log("Getting file '%s' from pipeline..." % filename, False, "info")
        self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
        # ground testItVolume path
        path = self.ground_path(self.pipelines[pipeline]['testItVolume'], prefix, suffix)
        fullname = path + self.tests[test].get("resultsDirectory", "") + filename
        if self.pipelines[pipeline].get('testItConnection', "-") != "-":
            fullname = self.get_temp_filename(pipeline, fullname)
        return fullname

    def read_yaml_file(self, test, pipeline, filename, prefix, suffix):
        fullname = self.get_file_from_pipeline(test, pipeline, filename, prefix, suffix)
        # Read the file
        self.log("Reading from file '%s'" % fullname, False, "info")
        data = None
        try:
            data = testit_common.parse_yaml(fullname)
            self.log("Read %s log entries!" % len(data), False, "info")
        except:
            self.log("Unable to open log file '%s'!" % fullname, False, "err")
        return data

    def log(self, message, return_condition=False, out="info"):
        prefix = ""
        if out == "err":
            prefix = "[ERROR]"
            rospy.logerr(message)
        elif out == "warn":
            prefix = "[WARN]"
            rospy.logwarn(message)
        elif out == "info":
            prefix = "[INFO]"
            rospy.loginfo(message)
        else:
            prefix = "[DEBUG]"
            rospy.logdebug(message)
        self.output_log.append((rospy.Time.now(), "[" + str(round(rospy.get_time(), 3)).rjust(8) + "] " + prefix.rjust(7) + " " + message))
        if return_condition:
            return message + "\n"
        else:
            return ""

    def handle_uppaal_extract_failure(self, req):
        """
        Extract the scenario failure into an Uppaal model (which can be executed as a test scenario in the future).
        """
        message = ""
        result = False

        add_scenario = False
        scenario_name = ""
        tokens = self.tokenize_arguments(req.args)
        tag_conflict = False
        if "--add-scenario" in tokens:
            index = tokens.index("--add-scenario")
            scenario_name = tokens[index + 1]
            # Check whether a scenario with that tag name exists
            del tokens[index + 1]
            del tokens[index]
            if scenario_name in self.tests:
                message += self.log("Test scenario '%s' already defined in configuration!" % scenario_name, True, "err")
                tag_conflict = True
            else:
                add_scenario = True
                self.log("Cloning scenario to '%s' upon success..." % scenario_name, False, "info")
        if not add_scenario or (add_scenario and not tag_conflict):
            tests = set(tokens)  # Remove duplicates
            matched = False
            processed_scenario = None
            for test in self.tests:
                if test in tests:
                    matched = True
                    self.log("Processing '%s'..." % test, False, "info")
                    model = self.tests[test].get('uppaalModel', None)
                    if model is not None:
                        pipeline = self.tests[test].get('executor_pipeline', None)
                        if not pipeline:
                            message += self.log("Test '%s' has not been executed during this runtime, unable to match data to pipeline!" % test,
                                                self.tests[test].get('verbose', False), "err")
                        else:
                            self.log("Ran in %s " % pipeline, False, "info")
                            if not self.tests[test].get('result', True):
                                # Test execution location found
                                testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh",
                                                                                        pipeline)
                                data = self.read_yaml_file(test, pipeline, "testit_coverage.log", testit_prefix,
                                                           testit_suffix)
                                if data is not None:
                                    filtered = []
                                    last_timestamp = -1.0
                                    for entry in data:
                                        if entry['traceStartTimestamp'] == data[-1]['traceStartTimestamp']:
                                            if entry['event'] == "PRE":
                                                if entry['timestamp'] != last_timestamp:
                                                    state = {}
                                                    for variable in entry['state']:
                                                        state.update(variable)
                                                    filtered.append((entry['name'], state))
                                                    last_timestamp = entry['timestamp']
                                                    rospy.logdebug("Found transition: '%s' - %s" % (
                                                        entry['name'], str(entry['state'])))
                                    if len(filtered) > 0:
                                        message = testit_uppaal.create_sequential_uppaal_xml(filtered)
                                        processed_scenario = test
                                        result = True
                                        break  # TODO add possibility to combine scenario failures
                                if not result:
                                    message = "Unable to create failure Uppaal XML model"
                            else:
                                self.log("Last execution was not FAILURE!", False, "err")
                    else:
                        self.log("'uppaalModel' is not defined in configuration!", False, "err")
            if result and add_scenario:
                self.log("create new scenario", False, "warn")
                self.tests[scenario_name] = self.tests[processed_scenario]
                self.tests[scenario_name]['tag'] = scenario_name
                self.add_test_to_config_file({'tests': [self.tests[scenario_name]]})
            elif not matched:
                message = self.log("Unable to match any test scenarios!", True, "err")
        return testit.srv.CommandResponse(result, message)

    def handle_uppaal_annotate_coverage(self, req):
        """
        Annotate Uppaal TA model (xml file) with coverage info.
        """
        message = "annotate message"
        result = True
        # TODO check req.args for specific test to process
        for test in self.tests:
            self.log("Processing '%s'..." % test, False, "info")
            model = self.tests[test].get('uppaalModel', None)
            if model is not None:
                # Read previous processed log entries
                data_directory = self.ground_path(self.configuration['dataDirectory'], "", "")
                self.log("Data directory path is '%s'" % data_directory, False, "info")
                coverage = []
                daemon_coverage_fullname = data_directory + "testit_coverage.log"
                try:
                    coverage = testit_common.parse_yaml(daemon_coverage_fullname)
                except:
                    self.log("Could not open previous coverage file '%s'!" % daemon_coverage_fullname, False, "warn")

                self.log("Uppaal model is %s" % model, False, "info")
                pipeline = self.tests[test].get('executor_pipeline', None)
                if not pipeline:
                    self.log("Test has not been executed during this runtime, unable to match data to pipeline!", False, "warn")
                else:
                    self.log("Ran in %s " % pipeline, False, "info")
                    # Get coverage log file from pipeline
                    # TODO support finding the log file in case it has been remapped in test adapter launch file
                    testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)
                    data = self.read_yaml_file(test, pipeline, "testit_coverage.log", testit_prefix, testit_suffix)
                    if data is not None:
                        path = self.ground_path(self.pipelines[pipeline]['testItVolume'], testit_prefix, testit_suffix)
                        # Add model info to daemon coverage log file (combined from all pipelines and over runs)
                        for entry in data:
                            entry['model'] = path + model
                        coverage += data
                        # It is up to the user to use "clean" command to avoid erroneous reprocessing
                        rospy.logwarn(
                            "Use \"testit_command.py clean\" before using this command again to avoid adding these entries again!")

                if len(coverage) > 0:
                    self.log("Processing %s coverage entries..." % len(coverage), False, "info")
                    # Create pruned list (keep only relevant model entries)
                    entries = []
                    self.log("Filtering '%s' file entries..." % req.args, False, "info")
                    for entry in coverage:
                        if model in entry['model']:
                            # TODO add PRE events, but only for advanced annotation algorithm
                            if req.args in entry['file'] and entry['event'] == "POST":
                                entries.append(entry)
                    self.log("Pruned list is %s entries!" % len(entries), False, "info")
                    if len(entries) > 0:
                        # Parse Uppaal model
                        self.log("Parsing Uppaal model...", False, "info")
                        root = None
                        try:
                            tree = xml.etree.ElementTree.parse(entries[0]['model'])
                            root = tree.getroot()
                        except Exception as e:
                            self.log("Unable to parse Uppaal model!", False, "err")
                            import traceback
                            traceback.print_exc()

                        # TODO consider nondeterminism in traces
                        trace = []
                        for entry in entries:
                            if entry['traceStartTimestamp'] == entries[-1]['traceStartTimestamp']:
                                trace.append(entry)
                        self.log("Annotating model with trace size %s..." % len(trace), False, "info")
                        maxV = 0
                        for entry in trace:
                            success, tree = self.annotate_uppaal_transition(tree, entry)
                            if success and entry['sum'] > maxV:
                                maxV = entry['sum']
                        # Add variable V and add variable "maxV" as constant to model
                        declaration = tree.findall("./declaration")
                        if len(declaration) > 0:
                            declaration[0].text += " int V; const int maxV=" + str(maxV) + ";"
                        else:
                            self.log("Unable to find '<declaration>' tag in XML tree!", False, "err")

                        # Save annotated Uppaal model to file
                        annotated_file = data_directory + "annotated_models/" + model
                        annotated_directory = "/".join(annotated_file.split("/")[:-1])
                        mkdir_result = subprocess.call("mkdir -p " + annotated_directory, shell=True)
                        if mkdir_result != 0:
                            self.log("Unable to create directory '%s'!" % annotated_directory, False, "err")
                        else:
                            self.log("Writing annotated Uppaal model file...", False, "info")
                            tree.write(annotated_file)
                            message = "Wrote annotated Uppaal model file to '%s'" % annotated_file
                            self.log(message, False, "info")

                        # Save coverage list to file
                        self.log("Saving coverage list to file...", False, "info")
                        testit_common.write_yaml_to_file(coverage, daemon_coverage_fullname)
                        self.log("Finished!", False, "info")
                    else:
                        self.log("No entries found for file '%s'!" % req.args, False, "err")
                        result = False
        return testit.srv.CommandResponse(result, message)

    def handle_online_test(self, req):
        """
        Prepare for online testing by sending log file to each testItConnection
        """
        message = "Prepare for online testing"
        result = True

        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)
        # Send log

        return testit.srv.CommandResponse(result, message)

    def handle_optimize_log_scenario(self, req):
        """
        Create an optimal sequential model (optimized test scenario) from the logger log file.
        """
        message = "Optimize message"
        result = True
        # Read previous log entries
        data_directory = self.ground_path(self.configuration['dataDirectory'], "", "")
        self.log("Data directory path is '%s'" % data_directory, False, "info")
        log_data = []
        daemon_log_fullname = data_directory + "logger.log"
        try:
            log_data = testit_common.parse_json_stream_file(daemon_log_fullname)
        except:
            self.log("Could not open previous log file '%s'!" % daemon_log_fullname, False, "warn")
        tokens = set(self.tokenize_arguments(req.args))
        weights = []
        for test in self.tests:
            for token in tokens:
                if token == test:
                    pipeline = self.tests[test].get('executor_pipeline', None)
                    if not pipeline:
                        rospy.logwarn(
                            "Test has not been executed during this runtime, unable to match data to pipeline!")
                    else:
                        self.log("Ran in %s " % pipeline, False, "info")
                        # Get log file from pipeline
                        testit_prefix, testit_suffix = self.get_command_wrapper("testItConnection", "ssh", pipeline)

                        fullname = self.get_file_from_pipeline(test, pipeline, "logger.log", testit_prefix,
                                                               testit_suffix)
                        log_data += testit_common.parse_json_stream_file(fullname)

                        # It is up to the user to use "clean" command to avoid erroneous reprocessing
                        rospy.logwarn(
                            "Use \"testit_command.py clean\" before using this command again to avoid adding these entries again!")

                    optimizer = self.tests[test].get('optimizer', {})
                    weights += optimizer.get('weights', [])
        optimized_sequence = testit_optimizer.optimize(log_data, weights, test)
        message = testit_uppaal.create_sequential_uppaal_xml(
            testit_uppaal.convert_optimizer_sequence_to_uppaal_synchronizations(optimized_sequence))

        self.log("Saving log to TestIt daemon data directory...", False, "info")
        for i, entry in enumerate(log_data):
            testit_common.append_to_json_file(entry, daemon_log_fullname, mode="w" if i == 0 else "a+")
        self.log("Finished!", False, "info")
        return testit.srv.CommandResponse(result, message)


if __name__ == "__main__":
    rospy.init_node('testit_daemon')
    testit_daemon = TestItDaemon()
    testit_daemon.log("TestIt daemon started...", False, "info")
    rospy.spin()
    testit_daemon.log("Shut down everything!", False, "info")
