#!/usr/bin/python

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

def optimize(log_data, weights):
    """
    Create an optimal test scenario based on the log entries given the weights.

    Args:
      log_data -- list of entries (e.g., [{"run_id": "fb741e78-4c70-4557-a572-e51f72567a05", "timestamp": 1475.0, "coverage": {}, "test": "Scenario #1", "data": {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}, "event": "PRE", "channel": {"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}}, ... ])
      weights -- list of weights (e.g., [{'type': 'coverage', 'name': 'patrol_detector.py', 'weight': 1.0}, ... ])
    Returns:
      optimized sequence as a list of tuples (channel, data), e.g., ([({"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}, {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}), ... ])
    """
    rospy.loginfo("Optimizing %s entries with weights %s" % (len(log_data), weights))
    
    run = log_data[0]['run_id']
    sequence = []
    for entry in log_data:
        if entry['run_id'] != run:
            break
        if entry['event'] == "PRE":
            continue
        step = (entry['channel'], entry['data'])
        rospy.loginfo(step[1]['target_pose']['pose']['position'])
        sequence.append(step)

    rospy.loginfo(sequence)
    return sequence
