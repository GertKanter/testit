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
import oracles_common
from gazebo_msgs.msg import ModelStates

class GazeboOracle(oracles_common.BaseOracle):
    def __init__(self, model_name):
        """
        Arguments:
            model_name -- string, the name of the robot model in Gazebo simulator
        """
        super(GazeboOracle, self).__init__()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.model_name = model_name
        self.robot = oracles_common.Robot()

    def callback(self, data):
        index = -1
        for i, name in enumerate(data.name):
            if name == self.model_name:
                index = i
        if index >= 0:
            self.robot.pose['position']['x'] = data.pose[i].position.x
            self.robot.pose['position']['y'] = data.pose[i].position.y
            self.robot.pose['position']['z'] = data.pose[i].position.z
            self.robot.pose['orientation']['x'] = data.pose[i].orientation.x
            self.robot.pose['orientation']['y'] = data.pose[i].orientation.y
            self.robot.pose['orientation']['z'] = data.pose[i].orientation.z
            self.robot.pose['orientation']['w'] = data.pose[i].orientation.w
            rospy.loginfo_throttle(1.0, self.robot.pose)
            self.callback_received = True
