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
import testit_common
import sys
import actionlib
import actionlib_msgs.msg
import json
import yaml
import testit_msgs.srv
import uuid
import threading
import testit_msgs.msg
import std_msgs.msg

class TestItLogger(object):
    def __init__(self):
        self.initialize()
        self.register_services_and_subscribe()

    def initialize(self):
        self.load_config_from_file()
        self.configuration = rospy.get_param('testit/configuration', None)
        self.test = rospy.get_param('~test', "")
        rospy.loginfo("Test is '%s'" % self.test)
        self.action_proxies = []
        self.service_proxies = []
        self.buffers = {}
        self.mapping = {}
        if self.configuration is None:
            rospy.logerr("Logger configuration not defined!")
            sys.exit(-1)
        self.log_file = rospy.get_param('~log', None)
        self.coverage_enabled = True
        self.seq = 0
        if self.configuration.get('coverage', None) is not None:
            self.coverage_enabled = self.configuration['coverage'].get("enable", True)
        if self.coverage_enabled:
            self.coverage = {}
            self.coverage_mode = self.configuration['coverage'].get("mode", "srv")
            self.reporting_time_limit = self.configuration['coverage'].get("reportingTimeLimit", 1.0)
            self.coverage_lock = threading.Lock()
            if self.coverage_mode == "srv":
                self.coverage_client = rospy.ServiceProxy("/testit/flush_coverage", testit_msgs.srv.Coverage)
            else:
                #TODO topic mode
                self.coverage_publisher = rospy.Publisher("/testit/flush_coverage", std_msgs.msg.UInt32, queue_size=10)
                self.coverage_subscriber = rospy.Subscriber("/testit/flush_data", testit_msgs.msg.FlushData, self.flush_subscriber)
        if self.log_file is None:
            rospy.logerr("Log file not defined!")
            sys.exit(-1)
        self.run_id = str(uuid.uuid4())

    def process_coverage_list(self, coverage, host_id="", seq=0):
        self.coverage_lock.acquire()
        try:
            for file_coverage in coverage:
                file_coverage.host_id = host_id
                file_coverage.seq = seq
                self.coverage[host_id+str(seq)+file_coverage.filename] = file_coverage
        except Exception as e:
            rospy.logerr("Exception from flush subscriber: %s" % e)
        self.coverage_lock.release()

    def flush_subscriber(self, data):
        # Received report from SUT host, log it
        # data is:
        # str host_id
        # uint32 seq
        # FileCoverage[] -- str filename, int32[] lines
        self.process_coverage_list(data.coverage, data.host_id, data.seq)

    def register_services_and_subscribe(self):
        """
        Subscribe to topics - both input and output
        """
        rospy.loginfo("Logger subscribing to topics and services...")
        if self.configuration.get('inputs', None) is not None:
            for i, channel in enumerate(map(lambda x: (x, 'input'), self.configuration.get('inputs', [])) + map(lambda x: (x, 'output'), self.configuration.get('outputs', []))):
                self.mapping[i] = channel[0]
                channel[0]['unique_id'] = i
                identifier = channel[0].get('identifier', "")
                rospy.loginfo("Processing channel: %s" % str(channel))
                if identifier != "":
                    channel_type = channel[0].get('type', "")
                    if channel_type != "":
                        rospy.loginfo("%s" % channel[0])
                        self.do_import(channel_type)
                        rospy.loginfo("Import successful!")
                        proxy = channel[0].get('proxy', "")
                        if proxy == "":
                            channel[0]['channel'] = channel[1]
                            eval("rospy.Subscriber(\"" + identifier + "\", " + channel_type + ", self.topic_callback, callback_args=" + str(i) + ")")
                            rospy.loginfo("Logger subscribed to %s" % identifier)
                        else:
                            if channel_type.endswith("Action"):
                                # Register action server and client
                                eval("self.action_proxies.append((actionlib.SimpleActionServer(\"" + proxy + "\", " + channel_type + ", lambda x: self.action_handler(x, " + str(i) + "), auto_start = False), actionlib.SimpleActionClient(\"" + identifier + "\", " + channel_type + ")))", dict(globals().items() + [('self', self)]))
                                self.action_proxies[-1][0].start()
                                rospy.loginfo("Waiting for '%s' action server..." % identifier)
                                self.action_proxies[-1][1].wait_for_server()
                                channel[0]['ready'] = True
                                rospy.loginfo("Registered action proxy %s" % proxy)
                            else:
                                # Register service
                                rospy.loginfo("Waiting for '%s' service..." % identifier)
                                rospy.wait_for_service(identifier)
                                rospy.loginfo("Creating service proxy...")
                                eval("self.service_proxies.append((rospy.Service(\"" + proxy + "\", " + channel_type + ", lambda x: self.service_handler(x, " + str(i) + ")),rospy.ServiceProxy(\"" + identifier + "\", " + channel_type + ")))", dict(globals().items() + [('self', self)]))
                                rospy.loginfo("Registered proxy service %s" % identifier)

    def do_import(self, channel_type):
        import_string = ".".join(channel_type.split(".")[:-1])
        rospy.loginfo("Importing '%s'" % import_string)
        exec("import " + import_string, globals())

    def flush_coverage(self):
        if self.coverage_enabled:
            if self.coverage_mode == "srv":
                try:
                    response = self.coverage_client()
                    if response.result:
                        self.coverage = self.process_coverage_list(response.coverage)
                    return True
                except rospy.ServiceException, e:
                    rospy.logerr("Coverage flush failed: %s" % e)
            else:
                # Topic mode
                self.seq += 1
                message = std_msgs.msg.UInt32()
                message.data = self.seq
                self.coverage_publisher.publish(message)
                return self.seq
        return None

    def write_log_entry(self, identifier, event, data):
        """
        Write a log entry to the log file.

        Args:
        identifier - the index of the mapping dictionary
        event - triggering event (i.e., "PRE" before sending the command to the
                service or "POST" after receiving the response from the service)
        data - the request data that is sent to the proxied service
        """
        #rospy.loginfo("writing log entry...")
        #channel = self.mapping[identifier]
        #rospy.loginfo("data is: %s" % str(data))
        #rospy.loginfo("type is %s" % type(data))
        channel = {'identifier': self.mapping[identifier]['identifier'], 'proxy': self.mapping[identifier]['proxy'], 'type': self.mapping[identifier]['type']}
        entry = {'run_id': self.run_id, 'timestamp': rospy.Time.now().to_sec(), 'channel': channel, 'event': event, 'data': json.loads(str(yaml.load(str(data))).replace("'", "\"").replace("None", "null")), 'test': self.test}
        seq = self.flush_coverage()
        if seq is not None:
            self.seq = seq
            if self.seq > 0:
                t = Timer(self.reporting_time_limit, self.add_coverage_entry, args=[entry, seq])
                t.start()
                return True
            else:
                return self.add_coverage_entry(entry, seq)

    def add_coverage_entry(self, entry, seq):
        entry['coverage'] = {}
        self.coverage_lock.acquire()
        try:
            purge_keys = []
            for entry in self.coverage:
                if self.coverage[entry].seq != seq:
                    continue
                entry['coverage'][self.coverage[entry].filename] = entry['coverage'].get(self.coverage[entry].filename, {})
                entry['coverage'][self.coverage[entry].filename][self.coverage[entry].host_id] = self.coverage[entry].lines
                purge_keys.append(entry)
            # Trim added entries
            for key in purge_keys:
                del self.coverage[key]
        except Exception as e:
            rospy.logerr("Exception when adding coverage entry: %s" % e)
        self.coverage_lock.release()
        return self.add_entry(entry)

    def load_config_from_file(self):
        filename = rospy.get_param('~config')
        rospy.loginfo("Loading configuration from " + filename + "...")
        testit_common.load_config_to_rosparam(testit_common.parse_yaml(filename))

    def add_entry(self, data):
        """
        Add an entry to the JSON log file.

        Args:
        data -- dict with values to store
        """
        #rospy.loginfo("trying to write: %s" % data)
        return testit_common.append_to_json_file(data, self.log_file)

    def get_action_proxy(self, identifier):
        for action_proxy in self.action_proxies:
            if action_proxy[0].action_server.ns == identifier:
                return action_proxy
        return None

    def get_service_proxy(self, identifier):
        for service_proxy in self.service_proxies:
            rospy.loginfo("resolved name = %s" % service_proxy[0].resolved_name)
            if service_proxy[0].resolved_name == identifier:
                return service_proxy
        return None

    def topic_callback(self, data, identifier):
        if self.mapping[identifier]['channel'] == 'output':
            # Update buffer values if needed (based on buffer.hz)
            self.mapping[identifier]['buffer'] = self.mapping[identifier].get('buffer', {})
            self.mapping[identifier]['buffer']['hz'] = self.mapping[identifier]['buffer'].get('hz', 1)
            self.mapping[identifier]['update_timestamp'] = self.mapping[identifier].get('update_timestamp', rospy.Time())
            if self.mapping[identifier]['update_timestamp'].to_sec() + (1.0 / self.mapping[identifier]['buffer']['hz']) <= rospy.Time.now().to_sec():
                self.buffers[identifier] = self.buffers.get(identifier, [])
                self.mapping[identifier]['buffer']['size'] = self.mapping[identifier]['buffer'].get('size', 1)
                if len(self.buffers[identifier]) < self.mapping[identifier]['buffer']['size']:
                    self.buffers[identifier].append(data)
                else:
                    self.mapping[identifier]['buffer_index'] = self.mapping[identifier].get('buffer_index', 0)
                    self.buffers[identifier][self.mapping[identifier]['buffer_index'] % len(self.buffers[identifier])] = data
                    self.mapping[identifier]['buffer_index'] += 1
                self.mapping[identifier]['update_timestamp'] = rospy.Time().now()
        else:
            # Write a log entry
            if not self.write_log_entry(identifier, "POST", data):
                rospy.logerr("Failed to write log entry!")

    def service_handler(self, req, identifier):
        rospy.loginfo("service_handler")
        if not self.write_log_entry(identifier, "PRE", req):
            rospy.logerr("Failed to write log entry!")
        rospy.logerr(self.mapping[identifier])
        rospy.logwarn(type(req))
        service_proxy = self.get_service_proxy(self.mapping[identifier]['proxy'])
        rospy.loginfo(service_proxy)
        rospy.loginfo("Calling service...")
        result = service_proxy[1](req)
        rospy.loginfo("Service call returned!")
        # Write a log entry
        if not self.write_log_entry(identifier, "POST", req):
            rospy.logerr("Failed to write log entry!")
        return result

    def action_handler(self, goal, identifier):
        if not self.mapping[identifier].get('ready', False):
            rospy.logwarn("Action handler not ready yet (waiting for action server)!")
            return
        # Write a log entry
        if not self.write_log_entry(identifier, "PRE", goal):
            rospy.logerr("Failed to write log entry!")
        action_proxy = self.get_action_proxy(self.mapping[identifier]['proxy'])
        if action_proxy is not None:
            action_proxy[1].send_goal(goal)
            action_proxy[1].wait_for_result()
            state = action_proxy[1].get_state()
            result = action_proxy[1].get_result()
            # Write a log entry
            if not self.write_log_entry(identifier, "POST", goal):
                rospy.logerr("Failed to write log entry!")
            if state == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                action_proxy[0].set_succeeded(result)
                rospy.loginfo("set succeeded")
            elif state == actionlib_msgs.msg.GoalStatus.PREEMPTED:
                action_proxy[0].set_preempted(result)
                rospy.loginfo("set preempted")
            elif state == actionlib_msgs.msg.GoalStatus.ABORTED:
                action_proxy[0].set_aborted(result)
                rospy.loginfo("set aborted")


if __name__ == "__main__":
    rospy.init_node('testit_logger')
    testit_logger = TestItLogger()
    rospy.loginfo("TestIt logger started...")
    rospy.spin()
    rospy.loginfo("Shut down everything!")

