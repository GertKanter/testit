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
import testit.srv
import rosparam
import rospkg
import testit_common
import threading
import time
import sys
import re
import subprocess

class TestItDaemon:
    def __init__(self):
        rospy.Service('testit/bringup', testit.srv.Command, self.handle_bringup)
        rospy.Service('testit/teardown', testit.srv.Command, self.handle_teardown)
        rospy.Service('testit/status', testit.srv.Command, self.handle_status)
        rospy.Service('testit/test', testit.srv.Command, self.handle_test)
        rospy.Service('testit/results', testit.srv.Command, self.handle_results)
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
        if self.tests is None:
            rospy.logerror("No tests defined in configuration!")
            sys.exit(-1)
        self.pipelines = self.rosparam_list_to_dict(rospy.get_param('testit/pipelines', None), 'tag')
        if self.pipelines is None:
            rospy.logerror("No pipelines defined in configuration!")
            sys.exit(-1)
        self.pipelines = self.substitute_config_values(
                           self.set_defaults(self.pipelines, 
                                             self.configuration))

    def substitute_config_values(self, params):
        for param in params:
            for key in params[param]:
                m = re.findall('(\[\[.*?\]\])', str(params[param][key]))
                if m is not None:
                    for replacement in m:
                        params[param][key] = params[param][key].replace(replacement, params[param][replacement[2:-2]], 1)
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
        rospack = rospkg.RosPack()
        filename = rospy.get_param('~config')
        rospy.loginfo("Loading configuration from " + filename + "...")
        testit_common.load_config_to_rosparam(testit_common.parse_yaml(filename))

    def thread_worker(self, tag, prefix):
        rospy.loginfo('[%s] Starter thread started!' % tag)
        rospy.loginfo('[%s] Executing %s SUT...' % (tag, prefix))
        if subprocess.call(self.pipelines[tag][prefix + 'SUT'], shell=True) == 0:
            rospy.loginfo('[%s] Done!' % tag)
            rospy.loginfo('[%s] Waiting for delay duration (%s)...' % (tag, self.pipelines[tag][prefix + 'SUTDelay']))
            time.sleep(self.pipelines[tag][prefix + 'SUTDelay'])
            rospy.loginfo('[%s] Executing %s TestIt...' % (tag, prefix))
            if subprocess.call(self.pipelines[tag][prefix + 'TestIt'], shell=True) == 0:
                rospy.loginfo('[%s] Done!' % tag)
                rospy.loginfo('[%s] Waiting for delay duration (%s)...' % (tag, self.pipelines[tag][prefix + 'TestItDelay']))
                time.sleep(self.pipelines[tag][prefix + 'TestItDelay'])
                rospy.loginfo('[%s] Waiting for the %s to finish...' % (tag, prefix))
                start_time = rospy.Time.now()
                while self.pipelines[tag][prefix + 'SUTTimeout'] == 0 or (rospy.Time.now() - start_time).to_sec() < self.pipelines[tag][prefix + 'SUTTimeout']:
                    if self.pipelines[tag][prefix + 'SUTFinishTrigger'] != '-':
                        # TODO using timeout + trigger
                        pass
                    time.sleep(1.0) 
                rospy.loginfo('[%s] Done!' % tag)
                self.threads[tag]['result'] = True
            else:
                rospy.logerr("[%s] Failed to execute TestIt!" % tag)
                self.threads[tag]['result'] = False
                return False
        else:
            rospy.logerr("[%s] Failed to execute SUT!" % tag)
            self.threads[tag]['result'] = False
            return False
        return True

    def multithreaded_command(self, verb, req, prefix):
        rospy.logdebug(verb + " requested")
	if req.pipeline == "":
            rospy.loginfo(verb + " all pipelines...")
        else:
            rospy.loginfo(verb + "ing " + req.pipeline + "...")
        for pipe in rospy.get_param('testit/pipelines', []):
            if req.pipeline == '' or req.pipeline == pipe['tag']:
                if prefix == "teardown":
                    # run stop just in case
                    self.execute_system(pipe['tag'], 'SUT', 'stop')
                    self.execute_system(pipe['tag'], 'TestIt', 'stop')
                rospy.loginfo(pipe['tag'] + " " + verb.lower() + "ing...")
                thread = threading.Thread(target=self.thread_worker, args=(pipe['tag'], prefix))
                self.threads[pipe['tag']] = {'thread': thread, 'result': None}
                thread.start()
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

    def handle_bringup(self, req):
        result = self.multithreaded_command("Start", req, "bringup")
        return testit.srv.CommandResponse(result[0], result[1])

    def handle_teardown(self, req):
        self.testing = False
        result = self.multithreaded_command("Stop", req, "teardown")
        return testit.srv.CommandResponse(result[0], result[1])

    def handle_status(self, req):
        rospy.logdebug("Status requested")
        message = ""
        result = True
        try:
            for pipeline in self.pipelines: # dict
                if self.pipelines[pipeline].get('bringup', False):
                    message += "[%s] ONLINE\n" % self.pipelines[pipeline]['tag']
                else:
                    message += "[%s] OFFLINE\n" % self.pipelines[pipeline]['tag']
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
                if self.pipelines[pipeline].get('bringup', False) and not self.pipelines[pipeline].get('busy', False):
                    self.pipelines[pipeline]['busy'] = True
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
        if subprocess.call(self.pipelines[pipeline][mode + system], shell=True) == 0:
            rospy.loginfo('[%s] Waiting for delay duration (%s)...' % (pipeline, self.pipelines[pipeline][mode + system + 'Delay']))
            time.sleep(self.pipelines[pipeline][mode + system + 'Delay'])
            start_time = rospy.Time.now()
            while self.pipelines[pipeline][mode + system + 'Timeout'] == 0 or (rospy.Time.now() - start_time).to_sec() < self.pipelines[pipeline][mode + system + 'Timeout']:
                if self.pipelines[pipeline][mode + system + 'FinishTrigger'] != '-':
                    # TODO using timeout + trigger
                    pass
                rospy.loginfo_throttle(15.0, '[%s] ..' % pipeline)
                time.sleep(1.0) 
            rospy.loginfo('[%s] Execution done!' % pipeline)
            return True
        else:
            rospy.logerr('[%s] Execution failed!' % pipeline)
        return False

    def thread_call(self, tag, command):
        self.call_result[tag] = -1 # -1 means timeout
        self.call_result[tag] = subprocess.call(command, shell=True)

    def execute_in_testit_container(self, pipeline, test):
        """
        Returns:
        True if test successful, False otherwise
        """
        #TODO support ssh wrapping (currently only runs on localhost)
        # launch test in TestIt docker in new thread (if oracle specified, run in detached mode)
        if self.configuration.get('bagEnabled', False):
            rospy.loginfo("[%s] Start rosbag recording..." % pipeline)
            bag_result = subprocess.call( "docker exec -d " + self.pipelines[pipeline]['testitHost'] + " /bin/bash -c \'source /opt/ros/$ROS_VERSION/setup.bash && cd /testit_tests/01/ && rosbag record -a --split --max-splits=" + str(self.tests[test]['bagMaxSplits']) + " --duration=" + str(self.tests[test]['bagDuration']) + " __name:=testit_rosbag_recorder\'", shell=True)
            rospy.loginfo("[%s] rosbag record returned %s" % (pipeline, bag_result))
        detached = ""
        if self.tests[test]['oracle'] != "":
            # run in detached
            detached = " -d "
        rospy.loginfo("[%s] Launching test \'%s\'" % (pipeline, test))
        thread = threading.Thread(target=self.thread_call, args=('launch', "docker exec " + detached + self.pipelines[pipeline]['testitHost'] + " /bin/bash -c \'source /opt/ros/$ROS_VERSION/setup.bash && " + self.tests[test]['launch'] + "\'"))
        start_time = rospy.Time.now()
        thread.start()
        thread.join(self.tests[test]['timeout'])
        return_value = False
        if self.call_result['launch'] == 0:
            # command returned success
            if detached == "":
                # test success, because we didn't run in detached
                rospy.loginfo("[%s] TEST PASS!" % pipeline)
                return_value = True
            else:
                # running detached, run oracle to assess test pass/fail
                # execute oracle in TestIt docker
                rospy.loginfo("[%s] Executing oracle..." % pipeline)
                thread = threading.Thread(target=self.thread_call, args=('oracle', "docker exec " + self.pipelines[pipeline]['testitHost'] + " /bin/bash -c \'source /catkin_ws/devel/setup.bash && " + self.tests[test]['oracle'] + "\'"))
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
                    return_value = True
        elif self.call_result['launch'] == -1:
            rospy.logwarn("[%s] TEST TIMEOUT (%s)!" % (pipeline, self.tests[test]['timeoutVerdict']))
            if self.tests[test]['timeoutVerdict']:
                return_value = True
        else:
            rospy.logerr("[%s] Test FAIL!" % pipeline)

        if return_value and self.configuration.get('bagEnabled', False):
            rospy.loginfo("[%s] Stop rosbag recording..." % pipeline)
            subprocess.call( "docker exec " + self.pipelines[pipeline]['testitHost'] + " /bin/bash -c \'source /catkin_ws/devel/setup.bash && rosnode kill /testit_rosbag_recorder && sleep 4\'", shell=True)
        return return_value

    def test_thread_worker(self, tag):
        """
        Arguments:
            tag -- test tag (string)
        """
        pipeline = self.acquire_pipeline(tag) # find a free pipeline (blocking)
        rospy.loginfo("Acquired pipeline %s" % pipeline)
        # runSUT
        rospy.loginfo("[%s] Running SUT..." % pipeline)
        if self.execute_system(pipeline, 'SUT', 'run'):
            # runTestIt
            rospy.loginfo("[%s] Running TestIt..." % pipeline)
            if self.execute_system(pipeline, 'TestIt', 'run'):
                rospy.loginfo("[%s] Executing tests in TestIt container..." % pipeline)
                self.tests[tag]['result'] = self.execute_in_testit_container(pipeline, tag)
                self.test_threads[tag]['result'] = self.tests[tag]['result']
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
            pass
        rospy.loginfo("Freeing pipeline \'%s\'" % pipeline)
        self.pipelines[pipeline]['busy'] = False

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

    def handle_test(self, req):
        rospy.logdebug("Test requested")
        result = True
        message = ""
        if not self.testing:
            self.testing = True
            for test in self.tests: # key
                self.tests[test]['result'] = None
                thread = threading.Thread(target=self.test_thread_worker, args=(test,))
                self.test_threads[test] = {'thread': thread, 'result': None}
                thread.start()
            thread = threading.Thread(target=self.nonblocking_test_monitor)
            thread.start()
        else:
            rospy.logerr("Unable to start tests! Tests are already executing!")
        return testit.srv.CommandResponse(result, message)

    def handle_results(self, req):
        result = True
        message = ""
        for test in self.tests:
            for test in self.tests:
                message += "Test \'%s\': %s" % (test, self.tests[test].get('result', None)) + "\n"
                if not self.tests[test].get('result', None):
                    result = False
        return testit.srv.CommandResponse(result, message)

if __name__ == "__main__":
    rospy.init_node('testit_daemon')
    testit_daemon = TestItDaemon()
    rospy.loginfo("TestIt daemon started...")
    while not rospy.is_shutdown():
        pass
    rospy.loginfo("Shut down everything!")
