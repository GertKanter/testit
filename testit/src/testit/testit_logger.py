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

class TestItLogger(object):
    def __init__(self):
        self.initialize()
        self.register_services_and_subscribe()

    def initialize(self):
        self.load_config_from_file()
        self.configuration = rospy.get_param('testit/configuration', None)
        self.action_servers = []
        self.buffers = {}
        self.mapping = {}
        if self.configuration is None:
            rospy.logerr("Logger configuration not defined!")
            sys.exit(-1)
        self.log_file = rospy.get_param('~log', None)
        if self.log_file is None:
            rospy.logerr("Log file not defined!")
            sys.exit(-1)

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
                        proxy = channel[0].get('proxy', "")
                        if proxy == "":
                            channel[0]['channel'] = channel[1]
                            eval("rospy.Subscriber(\"" + identifier + "\", " + channel_type + ", self.topic_callback, callback_args=" + str(i) + ")")
                            rospy.loginfo("Logger subscribed to %s" % identifier)
                        else:
                            if "Action" in channel_type:
                                # Register actionserver
                                eval("self.action_servers.append(actionlib.SimpleActionServer(\"" + proxy + "\", " + channel_type + ", lambda x: self.action_handler(x, " + str(i) + ")))", dict(globals().items() + [('self', self)]))
                                rospy.loginfo("Registered proxy actionserver %s" % proxy)
                            else:
                                # Register service
                                eval("rospy.Service(\"" + proxy + "\", " + channel_type + ", lambda x: self.service_handler(x, " + str(i) + "))", dict(globals().items() + [('self', self)]))
                                rospy.loginfo("Registered proxy service %s" % identifier)

    def do_import(self, channel_type):
        import_string = ".".join(channel_type.split(".")[:-1])
        rospy.loginfo("Importing '%s'" % import_string)
        exec("import " + import_string, globals())

    def write_log_entry(self, trigger):
        rospy.loginfo("writing log entry...")
        return self.add_entry({'trigger': trigger})

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
        return testit_common.append_to_json_file(data, self.log_file)

    def get_action_server(self, identifier):
        for action_server in self.action_servers:
            if action_server.action_server.ns == identifier:
                return action_server
        return None

    def topic_callback(self, data, identifier):
        rospy.loginfo("Topic callback: '%s'" % self.mapping[identifier])
        if self.mapping[identifier]['channel'] == 'output':
            # Update buffer values
            self.buffers[identifier] = self.buffers.get(identifier, [])
            if len(self.buffers[identifier]) < self.mapping[identifier].get('bufferSize', 1):
                self.buffers[identifier].append(data)
            else:
                buffer_index = self.mapping[identifier].get('buffer_index', 0)
                self.buffers[identifier][buffer_index%len(self.buffers[identifier])] = data
                self.mapping[identifier]['buffer_index'] += 1
            rospy.loginfo("%s  %s" % (self.mapping[identifier]['buffer_index'], self.buffers[identifier]))
        else:
            # Write a log entry
            if not self.write_log_entry('trigger'):
                rospy.logerr("Failed to write log entry!")

    def service_handler(self, req, mapping):
        rospy.loginfo("service_handler")
        rospy.logerr(self.configuration)
        rospy.logerr(self.mapping[mapping])
        rospy.logwarn(type(req))
        return ()

    def action_handler(self, goal, mapping):
        rospy.loginfo("action_handler")
        rospy.logerr(self.configuration)
        rospy.logerr(self.action_servers)
        rospy.logwarn(type(goal))
        action_server = self.get_action_server(self.mapping[mapping]['proxy'])
        if action_server is not None:
            action_server.set_succeeded()
            rospy.loginfo("set succeeded")

if __name__ == "__main__":
    rospy.init_node('testit_logger')
    testit_logger = TestItLogger()
    rospy.loginfo("TestIt logger started...")
    rospy.spin()
    rospy.loginfo("Shut down everything!")

