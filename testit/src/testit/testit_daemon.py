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
import testit_common
import threading
import time
import sys
import re
import subprocess
import testit.junit
import cStringIO
import xml.etree.ElementTree
import os
import rosbag

class TestItDaemon:
    def __init__(self):
        rospy.Service('testit/bringup', testit.srv.Command, self.handle_bringup)
        rospy.Service('testit/teardown', testit.srv.Command, self.handle_teardown)
        rospy.Service('testit/status', testit.srv.Command, self.handle_status)
        rospy.Service('testit/test', testit.srv.Command, self.handle_test)
        rospy.Service('testit/results', testit.srv.Command, self.handle_results)
        rospy.Service('testit/bag/collect', testit.srv.Command, self.handle_bag_collect)
        rospy.Service('testit/coverage', testit.srv.Command, self.handle_coverage)
        rospy.Service('testit/uppaal/annotate/coverage', testit.srv.Command, self.handle_uppaal_annotate_coverage)
        rospy.Service('testit/clean', testit.srv.Command, self.handle_clean)
        rospy.Service('testit/shutdown', testit.srv.Command, self.handle_shutdown)
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
        self.tests = self.set_defaults(self.rosparam_list_to_dict(rospy.get_param('testit/tests', None), 'tag'), self.configuration)
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

    def substitute_replacement_values(self, params, auxiliary={}):
        """
        Substitute the values wrapped with '[[]]' with the key value inside the brackets.

        In case the substitution is not possible within params, try to find a substitution in auxiliary dictionary
        
        Arguments:
        params -- dict, with potential values to substitute
        auxiliary -- dict with auxiliary values to look for substitution values
        """
        for param in params:
            for key in params[param]:
                m = re.findall('(\[\[.*?\]\])', str(params[param][key]))
                if m is not None:
                    for replacement in m:
                        substitution = params[param].get(replacement[2:-2], None)
                        if substitution is not None:
                            params[param][key] = params[param][key].replace(replacement, substitution, 1)
                        else:
                            # Unable to find substitution on the same dictionary level, try auxiliary dictionary
                            if len(auxiliary) > 0:
                                substitution = auxiliary.get(replacement[2:-2], None)
                                if substitution is not None:
                                    params[param][key] = params[param][key].replace(replacement, substitution, 1)
                                else:
                                    rospy.logerr("Unable to ground substition '%s' key '%s'" % (param, replacement))
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

    def load_config_from_file(self):
        filename = rospy.get_param('~config')
        rospy.loginfo("Loading configuration from " + filename + "...")
        testit_common.load_config_to_rosparam(testit_common.parse_yaml(filename))

    def execution_sleep(self, tag, prefix, instance):
        start_time = rospy.Time.now()
        while self.pipelines[tag]['state'] != "TEARDOWN" and (self.pipelines[tag][prefix + instance + 'Timeout'] == 0 or (rospy.Time.now() - start_time).to_sec() < self.pipelines[tag][prefix + instance + 'Timeout']):
            if self.pipelines[tag][prefix + instance + 'FinishTrigger'] != '-':
                # TODO using timeout + trigger
                pass
            time.sleep(1.0)
        rospy.loginfo('[%s] Done!' % tag)

    def instance_execution(self, tag, prefix, instance, set_result):
        rospy.loginfo('[%s] Executing %s %s...' % (tag, prefix, instance))
        if subprocess.call(self.pipelines[tag][prefix + instance], shell=True) == 0:
            rospy.loginfo('[%s] Done!' % tag)
            rospy.loginfo('[%s] Waiting for delay duration (%s)...' % (tag, self.pipelines[tag][prefix + instance + 'Delay']))
            time.sleep(self.pipelines[tag][prefix + instance + 'Delay'])
            rospy.loginfo('[%s] Waiting for the %s to finish...' % (tag, prefix))
            self.execution_sleep(tag, prefix, instance)
            if self.pipelines[tag].get('state', "OFFLINE") != "TEARDOWN":
                if set_result:
                    self.threads[tag]['result'] = True
                return True
            else:
                rospy.logerr("Pipeline in TEARDOWN state!")
                self.threads[tag]['result'] = False
                return False
        else:
            rospy.logerr("[%s] Failed to execute %s!" % (tag, instance))
            self.threads[tag]['result'] = False
            return False

    def thread_worker(self, tag, prefix, post_states):
        rospy.logdebug('[%s] thread_worker started!' % tag)
        if self.instance_execution(tag, prefix, "SUT", False):
            if self.instance_execution(tag, prefix, "TestIt", True):
                self.pipelines[tag]['state'] = post_states['True']
                return True
        self.pipelines[tag]['state'] = post_states['False']

    def multithreaded_command(self, verb, req, prefix, pre_state, post_states, extra_commands=[]):
        rospy.logdebug(verb + " requested")
        pipelines = []
        if req.args == "":
            rospy.loginfo(verb + " all pipelines...")
        else:
            pipelines = set(self.tokenize_arguments(req.args)) # Remove duplicates
            rospy.loginfo(verb + "ing " + req.args + "...")
        matched = False
        for pipe in rospy.get_param('testit/pipelines', []):
            if req.args == '' or pipe['tag'] in pipelines:
                rospy.loginfo("[%s] Setting state to %s" % (pipe['tag'], pre_state))
                self.pipelines[pipe['tag']]['state'] = pre_state
                if prefix == "teardown":
                    # run stop just in case
                    self.execute_system(pipe['tag'], 'SUT', 'stop')
                    self.execute_system(pipe['tag'], 'TestIt', 'stop')
                # Run extra_commands before executing the main command
                for command in extra_commands:
                    command(pipe['tag'])
                rospy.loginfo(pipe['tag'] + " " + verb.lower() + "ing...")
                thread = threading.Thread(target=self.thread_worker, args=(pipe['tag'], prefix, post_states))
                self.threads[pipe['tag']] = {'thread': thread, 'result': None}
                thread.start()
                matched = True
        if not matched:
            rospy.logwarn("Unable to recognize pipeline!")
        result = True
        message = ""
        while len(self.threads) > 0:
            for thread in self.threads:
                if not self.threads[thread]['result'] is None:
                    msg = '%s finished with %r' % (thread, self.threads[thread]['result'])
                    rospy.loginfo(msg)
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
        rospy.loginfo("removing bags from tag = %s" % tag)

    def handle_bringup(self, req):
        result = self.multithreaded_command("Start", req, "bringup", "BRINGUP", {'True': "READY", 'False': "FAILED"})
        return testit.srv.CommandResponse(result[0], result[1])

    def handle_teardown(self, req):
        self.testing = False
        result = self.multithreaded_command("Stop", req, "teardown", "TEARDOWN", {'True': "OFFLINE", 'False': "OFFLINE"}, extra_commands=[self.remove_bags])
        return testit.srv.CommandResponse(result[0], result[1])

    def handle_status(self, req):
        rospy.logdebug("Status requested")
        message = ""
        result = True
        try:
            for pipeline in self.pipelines: # dict
                message += "[%s] %s\n" % (self.pipelines[pipeline]['tag'], self.pipelines[pipeline].get('state', "OFFLINE"))
        except:
            result = False
        return testit.srv.CommandResponse(result, message)

    def acquire_pipeline(self, tag):
        """
        Return:
        pipeline tag
        """
        rospy.loginfo("Acquiring pipeline for test \'%s\'" % tag)
        while True:
            for pipeline in self.pipelines:
                if self.pipelines[pipeline].get('state', "OFFLINE") == "READY":
                    self.pipelines[pipeline]['state'] = "BUSY"
                    return pipeline
            time.sleep(0.5)
            rospy.logwarn_throttle(30.0, 'Test \'%s\' waiting for a free pipeline...' % tag)

    def execute_system(self, pipeline, system, mode):
        """
        blocking
        Returns:
        true -- if successful, false otherwise
        """
        rospy.loginfo("[%s] Executing %s to %s..." % (pipeline, system, mode))
        rospy.loginfo("[%s] Executing \"%s\"" % (pipeline, self.pipelines[pipeline][mode + system]))
        command = self.pipelines[pipeline].get(mode + system, None)
        if command is not None and subprocess.call(command, shell=True) == 0:
            rospy.loginfo('[%s] Waiting for delay duration (%s)...' % (pipeline, self.pipelines[pipeline][mode + system + 'Delay']))
            time.sleep(self.pipelines[pipeline][mode + system + 'Delay'])
            start_time = rospy.Time.now()
            while self.pipelines[pipeline]['state'] not in ["TEARDOWN", "FAILED", "OFFLINE"] and (self.pipelines[pipeline][mode + system + 'Timeout'] == 0 or (rospy.Time.now() - start_time).to_sec() < self.pipelines[pipeline][mode + system + 'Timeout']):
                if self.pipelines[pipeline][mode + system + 'FinishTrigger'] != '-':
                    # TODO using timeout + trigger
                    pass
                else:
                    # No trigger means we do not wait for timeout and break immediately
                    break
                rospy.loginfo_throttle(15.0, '[%s] (%s) ..' % (pipeline, mode))
                time.sleep(1.0) 
            rospy.loginfo('[%s] Execution done!' % pipeline)
            if self.pipelines[pipeline]['state'] not in ["TEARDOWN", "FAILED", "OFFLINE"]:
                return True
        else:
            rospy.logerr('[%s] Execution failed!' % pipeline)
        return False

    def thread_call(self, tag, command):
        self.call_result[tag] = -1 # -1 means timeout
        self.call_result[tag] = subprocess.call(command, shell=True)

    def resolve_configuration_value(self, target_dictionary, pipeline, key, default=None):
        """
        Resolve the configuration value based on - use default if missing.

        Returns:
        The resolved configuration value.
        """
        if key in self.pipelines[pipeline]:
            target_dictionary[key] = self.pipelines[pipeline][key]
            rospy.loginfo("Resolved '%s' to '%s'" % (key, target_dictionary[key]))
        else:
            if target_dictionary.get(key, None) is None:
                if default is not None:
                    target_dictionary[key] = default
                    rospy.loginfo("Resolved '%s' to default '%s'" % (key, target_dictionary[key]))
                else:
                    rospy.logerr("Unable to resolve '%s'" % key)
                    return None
        return target_dictionary[key]

    def delete_bag_files(self, pipeline, test):
        """
        Remove bag files from results.

        Returns:
        True if successful (no errors)
        """
        self.resolve_configuration_value(self.tests[test], pipeline, 'testItVolume')
        self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
        if self.tests[test]['testItVolume'] is not None:
            if self.tests[test]['resultsDirectory'] is not None:
                bags_directory = self.ground_path(self.tests[test]['testItVolume'] + self.tests[test]['resultsDirectory'])
                # Handle spaces in tag names
                prefix = test.split(" ")
                if len(prefix) > 1:
                    prefix = prefix[0:1] + ["\\ " + x for x in prefix[1:]]
                prefix = "".join(prefix)
                delete_command = "rm -f " + bags_directory + prefix + "*bag"
                return True if subprocess.call(delete_command, shell=True) == 0 else False
        return False

    def execute_in_testit_container(self, pipeline, test, keep_bags):
        """
        Returns:
        True if test successful, False otherwise
        """
        # execute preLaunchCommand, if this returns 0, proceed, if not, fail
        prelaunch_command = self.tests[test].get('preLaunchCommand', None)
        if prelaunch_command is not None:
            rospy.loginfo("[%s] Executing pre-launch command..." % pipeline)
            command = "docker exec " + self.pipelines[pipeline]['testItHost'] + " /bin/bash -c \'source /catkin_ws/devel/setup.bash && " + prelaunch_command + "\'"
            rospy.loginfo("Executing '%s'" % command)
            return_value = subprocess.call(command, shell=True)
            rospy.loginfo("[%s] Pre-launch command returned %s" % (pipeline, return_value))
            if return_value != 0:
                rospy.logerr("Pre-launch command failed! Test failed!")
                return False
        #TODO support ssh wrapping (currently only runs on localhost)
        bag_return = 1
        bag_enabled = self.tests[test].get('bagEnabled', False)
        if bag_enabled:
            # Delete old rosbags if present
            self.resolve_configuration_value(self.tests[test], pipeline, 'testItVolume')
            self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
            if not self.delete_bag_files(pipeline, test):
                rospy.logwarn("[%s] Rosbag deletion failed!" % pipeline)

            rospy.loginfo("[%s] Start rosbag recording..." % pipeline)
            max_splits = self.tests[test].get('bagMaxSplits', None)
            if max_splits is None:
                rospy.logwarn("[%s] bagMaxSplits is not defined, defaulting to 2" % pipeline)
                max_splits = 2
            duration = self.tests[test].get('bagDuration', None)
            if duration is None:
                rospy.logwarn("[%s] bagDuration is not defined, defaulting to 30" % pipeline)
                duration = 30
            topic_regex = self.tests[test].get('bagTopicRegex', None)
            if topic_regex is None:
                rospy.logwarn("[%s] bagTopicRegex is not defined, defaulting to 'all'" % pipeline)
                topic_regex = ""
            topics = "-a "
            if topic_regex != "":
                topics = "--regex \"" + str(topic_regex) + "\" "
            topic_exclude = self.tests[test].get('bagTopicExcludeRegex', None)
            if topic_exclude is None:
                rospy.logwarn("[%s] bagTopicExcludeRegex is not defined, defaulting to ''" % pipeline)
                topic_exclude = ""
            exclude = ""
            if topic_exclude != "":
                exclude = "--exclude \"" + str(topic_exclude) + "\" "
            self.resolve_configuration_value(self.tests[test], pipeline, 'sharedDirectory')
            command = "docker exec -d " + self.pipelines[pipeline]['testItHost'] + " /bin/bash -c \'source /opt/ros/$ROS_VERSION/setup.bash && mkdir -p " + str(self.tests[test]['sharedDirectory']) + str(self.tests[test]['resultsDirectory']) +  " && cd " + str(self.tests[test]['sharedDirectory']) + str(self.tests[test]['resultsDirectory']) + " && rosbag record --split --max-splits=" + str(max_splits) + " --duration=" + str(duration) + " -O \"" + test + "\" " + exclude + topics + "__name:=testit_rosbag_recorder\'"
            rospy.loginfo("Executing '%s'" % command)
            bag_return = subprocess.call(command, shell=True)
            rospy.loginfo("[%s] rosbag record returned %s" % (pipeline, bag_return))
        # launch test in TestIt docker in new thread (if oracle specified, run in detached mode)
        detached = ""
        if self.tests[test]['oracle'] != "":
            # run in detached
            detached = "-d "
        rospy.loginfo("[%s] Launching test \'%s\'" % (pipeline, test))
        self.resolve_configuration_value(self.tests[test], pipeline, 'verbose', False)
        if self.tests[test]['verbose']:
            rospy.loginfo("[%s] launch parameter is \'%s\'" % (pipeline, self.tests[test]['launch']))
        launch = self.tests[test].get('launch', "")
        start_time = rospy.Time.now()
        if launch != "":
            thread_command = "docker exec " + detached + self.pipelines[pipeline]['testItHost'] + " /bin/bash -c \'source /catkin_ws/devel/setup.bash && " + self.tests[test]['launch'] + "\'"
            rospy.loginfo("[%s] Docker command is '%s'" % (pipeline, thread_command))
            thread = threading.Thread(target=self.thread_call, args=('launch', thread_command))
            thread.start()
            thread.join(self.tests[test]['timeout'])
        return_value = False
        if launch == "" or self.call_result['launch'] == 0:
            # command returned success
            if detached == "":
                # test success, because we didn't run in detached
                rospy.loginfo("[%s] TEST PASS!" % pipeline)
                return_value = True
            else:
                # running detached, run oracle to assess test pass/fail
                # execute oracle in TestIt docker
                rospy.loginfo("[%s] Executing oracle..." % pipeline)
                thread = threading.Thread(target=self.thread_call, args=('oracle', "docker exec " + self.pipelines[pipeline]['testItHost'] + " /bin/bash -c \'source /catkin_ws/devel/setup.bash && " + self.tests[test]['oracle'] + "\'"))
                thread.start()
                thread.join(max(0.1, self.tests[test]['timeout'] - (rospy.Time.now() - start_time).to_sec()))
                if self.call_result['oracle'] == 0:
                    # oracle reports test pass
                    rospy.loginfo("[%s] TEST PASS!" % pipeline)
                    return_value = True
                elif self.call_result['oracle'] == -1:
                    rospy.logwarn("[%s] TEST TIMEOUT (%s)!" % (pipeline, self.tests[test]['timeoutVerdict']))
                    if self.tests[test]['timeoutVerdict']:
                        return_value = True
                else:
                    # oracle reports test failed
                    rospy.logerr("[%s] TEST FAIL!" % pipeline)
        elif self.call_result['launch'] == -1:
            rospy.logwarn("[%s] TEST TIMEOUT (%s)!" % (pipeline, self.tests[test]['timeoutVerdict']))
            if self.tests[test]['timeoutVerdict']:
                return_value = True
        else:
            rospy.logerr("[%s] Test FAIL!" % pipeline)

        if bag_return == 0 and bag_enabled:
            rospy.loginfo("[%s] Stop rosbag recording..." % pipeline)
            subprocess.call("docker exec " + self.pipelines[pipeline]['testItHost'] + " /bin/bash -c \'source /catkin_ws/devel/setup.bash && rosnode kill /testit_rosbag_recorder && sleep 4\'", shell=True)
            rospy.loginfo("[%s] Setting privileges..." % pipeline)
            subprocess.call("docker exec " + self.pipelines[pipeline]['testItHost'] + " /bin/bash -c \'chown -R " + self.ground_path("$(id -u)") + ":" + self.ground_path("$(id -g)") + " " + str(self.tests[test]['sharedDirectory']) + str(self.tests[test]['resultsDirectory']) + "\'", shell=True)
            # Delete bags if success, merge split bags and keep if fail
            if not return_value or keep_bags:
                rospy.loginfo("[%s] Merging bag files..." % pipeline)
                bags_directory = self.ground_path(self.tests[test]['testItVolume'] + self.tests[test]['resultsDirectory'])
                try:
                    files = sorted([f for f in os.listdir(bags_directory) if os.path.isfile(os.path.join(bags_directory, f)) and f.startswith(test) and f.endswith(".bag")])
                except Exception as e:
                    rospy.logerr("Path not found ('%s')!" % bags_directory)
                bags = []
                for f in files:
                   bags.append((f, os.stat(os.path.join(bags_directory, f)).st_mtime))
                bags = sorted(bags, key=lambda x: x[1])
                # Merge bags
                with rosbag.Bag(os.path.join(bags_directory, str(test) + ".bag"), 'w') as outfile:
                    for i, bag in enumerate(bags):
                        rospy.loginfo("Merging '%s'... (%s/%s)" % (bag[0], str(i+1), str(len(bags))))
                        with rosbag.Bag(os.path.join(bags_directory, bag[0]), 'r') as infile:
                            for topic, msg, t in infile:
                                outfile.write(topic, msg, t)
                        # Remove the individual bag file
                        os.remove(os.path.join(bags_directory, bag[0]))
                rospy.loginfo("[%s] Done!" % pipeline)
            else:
                # Delete bags
                rospy.loginfo("[%s] Removing bag files..." % pipeline)
                if not self.delete_bag_files(pipeline, test):
                    rospy.logwarn("[%s] Rosbag deletion failed!" % pipeline)
        return return_value

    def test_thread_worker(self, tag, keep_bags=False):
        """
        Arguments:
            tag -- test tag (string)
        """
        #TODO if specific pipeline is specified for a test, acquire that specific pipeline
        pipeline = self.acquire_pipeline(tag) # find a free pipeline (blocking)
        rospy.loginfo("Acquired pipeline %s" % pipeline)
        self.tests = self.substitute_replacement_values(self.tests, self.pipelines[pipeline])
        # runSUT
        rospy.loginfo("[%s] Running SUT..." % pipeline)
        if self.execute_system(pipeline, 'SUT', 'run'):
            # runTestIt
            rospy.loginfo("[%s] Running TestIt..." % pipeline)
            if self.execute_system(pipeline, 'TestIt', 'run'):
                rospy.loginfo("[%s] Executing tests in TestIt container..." % pipeline)
                self.tests[tag]['result'] = self.execute_in_testit_container(pipeline, tag, keep_bags)
                self.tests[tag]['executor_pipeline'] = pipeline
                self.test_threads[tag]['result'] = self.tests[tag]['result']
                # execute the post-testing commands
                self.resolve_configuration_value(self.tests[tag], pipeline, 'postCommand', "")
                self.resolve_configuration_value(self.tests[tag], pipeline, 'postSuccessCommand', "")
                self.resolve_configuration_value(self.tests[tag], pipeline, 'postFailureCommand', "")
                if self.tests[tag]['result']:
                    if self.tests[tag]['postSuccessCommand'] != "":
                        rospy.loginfo("Executing post-success command ('%s')..." % self.tests[tag]['postSuccessCommand'])
                        result = subprocess.call(self.tests[tag]['postSuccessCommand'], shell=True)
                        if result != 0:
                            rospy.logerr("Post-success command failed!")
                else:
                    if self.tests[tag]['postFailureCommand'] != "":
                        rospy.loginfo("Executing post-failure command ('%s')..." % self.tests[tag]['postFailureCommand'])
                        result = subprocess.call(self.tests[tag]['postFailureCommand'], shell=True)
                        if result != 0:
                            rospy.logerr("Post-failure command failed!")
                if self.tests[tag]['postCommand'] != "":
                    rospy.loginfo("Executing post-test command ('%s')..." % self.tests[tag]['postCommand'])
                    result = subprocess.call(self.tests[tag]['postCommand'], shell=True)
                    if result != 0:
                        rospy.logerr("Post-test command failed!")
                # stopTestIt
                rospy.loginfo("[%s] Stopping TestIt container..." % pipeline)
                self.execute_system(pipeline, 'TestIt', 'stop')
            else:
                # unable to run TestIt
                rospy.logerr("[%s] Unable to run TestIt!" % pipeline)
            # stopSUT
            rospy.loginfo("[%s] Stopping SUT..." % pipeline)
            self.execute_system(pipeline, 'SUT', 'stop')
        else:
            # Unable to run SUT
            rospy.logerr("[%s] Unable to run SUT!" % pipeline)
        if self.pipelines[pipeline]['state'] not in ["TEARDOWN", "OFFLINE", "FAILED"]:
            rospy.loginfo("Freeing pipeline \'%s\'" % pipeline)
            self.pipelines[pipeline]['state'] = "READY"

    def nonblocking_test_monitor(self):
        while True:
            sleep = True
            for thread in self.test_threads:
                if not self.test_threads[thread]['result'] is None:
                    del self.test_threads[thread]
                    sleep = False
                    break
            if len(self.test_threads) == 0:
                self.testing = False
                return # all threads are finished
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
                    data_directory = self.ground_path(self.configuration['dataDirectory'])
                    rospy.loginfo("Cleaning daemon data directory '%s'..." % data_directory)
                    remove_result = subprocess.call("rm -rf " + data_directory + "*", shell=True)
                    if remove_result != 0:
                        rospy.logerr("Unable to remove files from '%s'!" % data_directory)
                    else:
                        rospy.loginfo("Done!")
                else:
                    rospy.logerr("'dataDirectory' is not defined in configuration!")
            if len(req.args) > 0:
                queue = []
            pipes = set(self.tokenize_arguments(req.args)) # Remove duplicates
            for pipe in pipes:
                found = False
                for pipeline in self.pipelines:
                    if pipe == self.pipelines[pipeline]['tag']:
                        found = True
                        queue.append(pipe)
                if not found:
                    rospy.logwarn("Unknown pipeline tag specified '%s'" % pipe)
        if len(queue) > 0:
            rospy.loginfo("Cleaning results directories from pipelines: %s" % str(queue))
            for pipeline in queue:
                if self.pipelines[pipeline].get('testItConnection', "-") != "-":
                    #TODO add support for remote testit pipeline (scp to temp file then read)
                    # command_prefix = "scp "
                    rospy.logerr("Not implemented!")
                testit_volume = self.pipelines[pipeline].get('testItVolume', None)
                results_directory = self.pipelines[pipeline].get('resultsDirectory', None)
                if results_directory is not None:
                    if testit_volume is not None:
                        rospy.loginfo("Cleaning results directory at '%s'..." % pipeline)
                        command = "rm -rf " + self.ground_path(testit_volume + results_directory) + "*"
                        rospy.loginfo("Executing command '%s'" % command)
                        remove_result = subprocess.call(command, shell=True)
                        if remove_result != 0:
                            rospy.logerr("Unable to remove files from '%s'!" % data_directory)
                        else:
                            rospy.loginfo("Done!")
                    else:
                        rospy.logerr("'testItVolume' is not defined in configuration!")
                else:
                    rospy.logerr("'resultsDirectory' is not defined in configuration!")
        else:
            result = False
            message = "Unable to clean workspace!"
        return testit.srv.CommandResponse(result, message)

    def handle_test(self, req):
        rospy.logdebug("Test requested")
        result = True
        message = ""
        # Create list with tests to execute
        queue = [test for test in self.tests]
        blocking = False
        keep_bags = False
        if len(req.args) > 0:
            # Determine whether to block or not
            if "--keep-bags" in req.args:
                keep_bags = True
                req.args = req.args.replace("--keep-bags", "", 1)
            if "--blocking" in req.args:
                blocking = True
                req.args = req.args.replace("--blocking", "", 1)
            if len(req.args.strip()) > 0:
                queue = []
            scenarios = set(self.tokenize_arguments(req.args)) # Remove duplicates
            for scenario in scenarios:
                found = False
                for test in self.tests:
                    if scenario == self.tests[test]['tag']:
                        found = True
                        queue.append(scenario)
                if not found:
                    rospy.logwarn("Unknown test tag specified '%s'" % scenario)
        rospy.loginfo("Test scenarios queued: " + str(queue))
        if not self.testing:
            self.testing = True
            for test in queue: # key
                self.tests[test]['result'] = None
                self.tests[test]['pipeline'] = None
                thread = threading.Thread(target=self.test_thread_worker, args=(test, keep_bags))
                self.test_threads[test] = {'thread': thread, 'result': None}
                thread.start()
            thread = threading.Thread(target=self.nonblocking_test_monitor)
            thread.start()
            if blocking:
                finished = False
                while not finished:
                    finished = True
                    for test in queue:
                        result = self.tests[test].get('result', None)
                        if result is None:
                            finished = False
                            break
                    rospy.sleep(0.1)
        else:
            rospy.logerr("Unable to start tests! Tests are already executing!")
        return testit.srv.CommandResponse(result, message)

    def handle_results(self, req):
        rospy.logdebug("Results requested")
        message = ""
        result = True
        testsuite = testit.junit.testsuite(tests=len(self.tests))
        output = cStringIO.StringIO()
        for test in self.tests:
            testcase = testit.junit.testcase(classname=test)
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
                        rospy.loginfo("Ran in %s " % pipeline)
                        # Get XML file from pipeline
                        self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
                        filename = self.tests[test].get("resultsDirectory", "") + self.tests[test]['tag'] + "_system_out.xml"
                        if self.pipelines[pipeline].get('testItConnection', "-") != "-":
                            #TODO add support for remote testit pipeline (scp to temp file then read)
                            # command_prefix = "scp "
                            rospy.logerr("Not implemented!")
                        # ground testItVolume path
                        path = self.ground_path(self.pipelines[pipeline]['testItVolume'])
                        fullname = self.ground_path(path + filename)
                        # Read the file
                        rospy.loginfo("Reading from file '%s'" % fullname)
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
        rospy.loginfo("Copying files from pipelines...")
        data_directory = self.configuration.get('dataDirectory', None)
        if data_directory is not None:
            data_directory = self.ground_path(data_directory)
            for pipeline in self.pipelines:
                rospy.loginfo("Copying files from pipeline '%s'..." % pipeline)
                testit_volume = self.pipelines[pipeline].get('testItVolume', None)
                if testit_volume is not None:
                    results_directory = self.pipelines[pipeline].get('resultsDirectory', None)
                    if results_directory is not None:
                        #TODO add support for remote pipelines
                        bags_directory = self.ground_path(testit_volume + results_directory)
                        command = "cp " + bags_directory + "*bag " + data_directory
                        rospy.loginfo("Executing command '%s'" % command)
                        copy_result = subprocess.call(command, shell=True)
                        if copy_result != 0:
                            rospy.logerr("Unable to copy files from pipeline '%s'!" % pipeline)
                            result = False
                        else:
                            rospy.loginfo("Done!")
        else:
            rospy.logerr("'dataDirectory' is not defined!")
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

    def ground_path(self, command):
        """
        Process paths with bash commands.
        E.g., '$(rospack find testit)/data/' to '/home/user/catkin_ws/src/testit/testit/data/'
        """
        process = subprocess.Popen(['/bin/bash', '-c', 'echo ' + command + ''], stdout=subprocess.PIPE)
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

    def handle_uppaal_annotate_coverage(self, req):
        """
        Annotate Uppaal TA model (xml file) with coverage info.
        """
        message = "annotate message"
        result = True
        #TODO check req.args for specific test to process
        for test in self.tests:
            rospy.loginfo("Processing '%s'..." % test)
            model = self.tests[test].get('uppaalModel', None)
            if model is not None:

                # Read previous processed log entries
                data_directory = self.ground_path(self.configuration['dataDirectory'])
                rospy.loginfo("Data directory path is '%s'" % data_directory)
                coverage = []
                daemon_coverage_fullname = data_directory + "testit_coverage.log"
                try:
                    coverage = testit_common.parse_yaml(daemon_coverage_fullname)
                except:
                    rospy.logwarn("Could not open previous coverage file '%s'!" % daemon_coverage_fullname)

                rospy.loginfo("Uppaal model is %s" % model)
                pipeline = self.tests[test].get('executor_pipeline', None)
                if not pipeline:
                    rospy.logwarn("Test has not been executed during this runtime, unable to match data to pipeline!")
                else:
                    rospy.loginfo("Ran in %s " % pipeline)
                    # Get coverage log file from pipeline
                    #TODO support finding the log file in case it has been remapped in test adapter launch file
                    self.resolve_configuration_value(self.tests[test], pipeline, 'resultsDirectory')
                    filename = self.tests[test].get("resultsDirectory", "") + "testit_coverage.log"
                    if self.pipelines[pipeline].get('testItConnection', "-") != "-":
                        #TODO add support for remote testit pipeline (scp to temp file then read)
                        # command_prefix = "scp "
                        rospy.logerr("Not implemented!")
                    # ground testItVolume path
                    path = self.ground_path(self.pipelines[pipeline]['testItVolume'])
                    fullname = path + filename
                    # Read the file
                    rospy.loginfo("Reading coverage log from file '%s'" % fullname)
                    data = None
                    try:
                        data = testit_common.parse_yaml(fullname)
                        rospy.loginfo("Read %s log entries!" % len(data))
                        # Add model info to daemon coverage log file (combined from all pipelines and over runs)
                        for entry in data:
                            entry['model'] = path + 'testit_tests/' + model
                        coverage += data
                    except:
                        rospy.logerr("Unable to open log file '%s'!" % fullname)
                    if data is not None:
                        # Remove the processed log file so we don't process it again
                        rospy.loginfo("Removing log file '%s'" % fullname)
                        remove_result = subprocess.call("rm -f " + fullname, shell=True)
                        if remove_result != 0:
                            rospy.logerr("Unable to remove log file '%s'!" % fullname)

                if len(coverage) > 0:
                    rospy.loginfo("Processing %s coverage entries..." % len(coverage))
                    # Create pruned list (keep only relevant model entries)
                    entries = []
                    rospy.loginfo("Filtering '%s' file entries..." % req.args)
                    for entry in coverage:
                        if model in entry['model']:
                            #TODO add PRE events, but only for advanced annotation algorithm
                            if req.args in entry['file'] and entry['event'] == "POST":
                                entries.append(entry)
                    rospy.loginfo("Pruned list is %s entries!" % len(entries))
                    if len(entries) > 0:
                        # Parse Uppaal model
                        rospy.loginfo("Parsing Uppaal model...")
                        root = None
                        try:
                            tree = xml.etree.ElementTree.parse(entries[0]['model'])
                            root = tree.getroot()
                        except Exception as e:
                            rospy.logerr("Unable to parse Uppaal model!")
                            import traceback
                            traceback.print_exc()

                        #TODO consider nondeterminism in traces
                        trace = []
                        for entry in entries:
                            if entry['traceStartTimestamp'] == entries[-1]['traceStartTimestamp']:
                                trace.append(entry)
                        rospy.loginfo("Annotating model with trace size %s..." % len(trace))
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
                            rospy.logerr("Unable to find '<declaration>' tag in XML tree!")
                    
                        # Save annotated Uppaal model to file
                        annotated_file = data_directory + "annotated_models/" + model
                        annotated_directory = "/".join(annotated_file.split("/")[:-1])
                        mkdir_result = subprocess.call("mkdir -p " + annotated_directory, shell=True)
                        if mkdir_result != 0:
                            rospy.logerr("Unable to create directory '%s'!" % annotated_directory)
                        else:
                            rospy.loginfo("Writing annotated Uppaal model file...")
                            tree.write(annotated_file)
                            message = "Wrote annotated Uppaal model file to '%s'" % annotated_file
                            rospy.loginfo(message)

                        # Save coverage list to file
                        rospy.loginfo("Saving coverage list to file...")
                        testit_common.write_yaml_to_file(coverage, daemon_coverage_fullname)
                    else:
                        rospy.logerr("No entries found for file '%s'!" % req.args)
                        result = False
                
                
        
        return testit.srv.CommandResponse(result, message)

if __name__ == "__main__":
    rospy.init_node('testit_daemon')
    testit_daemon = TestItDaemon()
    rospy.loginfo("TestIt daemon started...")
    rospy.spin()
    rospy.loginfo("Shut down everything!")
