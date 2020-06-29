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

def convert_optimizer_sequence_to_uppaal_synchronizations(sequence):
    """
    Args:
      sequence - optimized sequence as a list of tuples (channel, data), e.g., ([({"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}, {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}), ... ])
    Returns:
    list of synchronization tuples (sync name, {arguments}, lower_bound, upper_bound) in temporal order, for example: [("robot_0_goto", {'mode': 2, 'waypoint': 3}, 0.9, 2.0), ...]
    """
    synchronizations = []
    for step in sequence:
        synchronizations.append((step[0]['identifier'][1:].split("/")[0] +"_moveto", {'x': int(step[1]['target_pose']['pose']['position']['x']*10), 'y': int(step[1]['target_pose']['pose']['position']['y']*10)}, 1, 10))
    return synchronizations

def create_sequential_uppaal_xml(synchronizations):
    """
    Create the Uppaal model based on the synchronization input.

    Args:
    synchronizations -- list of synchronization tuples (sync name, {arguments}) in temporal order, for example: [("robot_0_goto", {'mode': 2, 'waypoint': 3}), ...]
    """
    xml = []
    xml.append("""<?xml version="1.0" encoding="utf-8"?>\n<!DOCTYPE nta PUBLIC '-//Uppaal Team//DTD Flat System 1.1//EN' 'http://www.it.uu.se/research/group/darts/uppaal/flat-1_2.dtd'>\n<nta>""")
    # Global Declarations
    xml.append("""<declaration>""");
    variables = []
    for i in range(len(synchronizations)):
        for state in synchronizations[i][1]:
            variables.append("i_" + synchronizations[i][0] + "_" + state)
    variables = set(variables)
    xml.append("int " + ", ".join(variables) + ";")

    channels = set([x[0] for x in synchronizations])
    for channel in channels:
        xml.append("chan i_" + channel + ", o_" + "_".join(channel.split("_")[:-1]) + "_response;")
    xml.append("""</declaration>\n""")

    # Map template
    xml.append("""<template>\n  <name x="5" y="5">model</name>\n  <declaration>\n  // Place local declarations here.\n</declaration>""")

    # Locations...
    for i in range((len(synchronizations) * 2) + 1):
        xml.append("<location id=\"id" + str(i) + "\" x=\"8\" y=\"" + str(i*20) + "\"><name x=\"10\" y=\"" + str(i*20) + "\">Location" + str(i) + "</name></location>")
    # Init location
    xml.append("<init ref=\"id0\"/>")
    # Transitions
    for i in range(0, len(synchronizations) * 2, 2):
        assignment = ""
        separator = False
        for state in synchronizations[i//2][1]:
            if separator:
                assignment += ", "
            assignment += "i_{0}_{1}={2}".format(synchronizations[i//2][0], state, synchronizations[i//2][1][state])
            separator = True
        xml.append("""<transition>
                        <source ref="id{0}"/>
			<target ref="id{1}"/>
			<label kind="synchronisation" x="-8" y="-110">i_{4}!</label>
			<label kind="assignment" x="-8" y="-85">{2}</label>
			<nail x="-76" y="-119"/>
		</transition>
                <transition>
			<source ref="id{1}"/>
			<target ref="id{3}"/>
			<label kind="synchronisation" x="-59" y="17">o_{5}_response?</label>
			<nail x="-68" y="25"/>
		</transition>
		""".format(i, i+1, assignment, i+2, synchronizations[i//2][0], "_".join(synchronizations[i//2][0].split("_")[:-1])))

    xml.append("</template>\n")
    # Template for Dtron
    xml.append("""<template>\n  <name>sut</name>\n  <location id="id0" x="0" y="0"></location>\n  <init ref="id0"/>""")
    for channel in channels:
        xml.append("""<transition>\n  <source ref="id0"/>\n  <target ref="id0"/>\n  <label kind="synchronisation" x="-8" y="-110">i_{0}?</label>\n  <nail x="-76" y="-119"/>\n</transition>\n<transition>\n  <source ref="id0"/>\n  <target ref="id0"/>\n  <label kind="synchronisation" x="-59" y="17">o_{1}_response!</label>\n  <nail x="-68" y="25"/>\n</transition>""".format(channel, "_".join(channel.split("_")[:-1])))
    xml.append("</template>\n")
    # System declaration
    xml.append("""<system>\nProcess1 = model();\nProcess2 = sut();\nsystem Process1, Process2;\n</system>\n<queries>\n</queries>\n</nta>""")
    return "\n".join(xml)
