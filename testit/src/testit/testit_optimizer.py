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
import sys
import hashlib

class Optimizer:
    """
    graph - {state: [edge, ...]}
    state - vector of parallel monitored inputs (logged values), each monitored variable adds a new dimension to the state space (multiplicative increase, for example if there are two monitored variables with 4 individual possible values, the state space size is 4*4=16). The graph is normally sparse and branching factor is low but this depends on the concrete use case.
    edge - destination state, qty, local param vector -(global param state)-> gain
           (same "destination" edges form a probability via qty/total)
    max_depth - how many states to look into the future

    given a starting state (source) we want to find the next input to the SUT that yields the biggest increase in global parameter state value.
     * there are temporality concerns = the desired effect might happen during transitions (interaction between individual automata during state transition, e.g., two robots meet at a specific location while navigating)
    to compute this, we start by determining the first step gains
    one step is computed by examining all edges from the given state
    the edges are first pre-processed by merging together edges that lead to the same destination state (this happens because of nondeterminism due to hidden variables)
    after the merge, we can compute the individual edge gains (the increase in global parameter state value function) taking into account the predefined weights vector
    all the individual edges are then added to a processing queue
    if the max_depth parameter would be 1, we would choose the biggest gain edge in the queue and that would be the next SUT input
    if the max_depth parameter is >1, we have to look at all elements in the queue and compute the gains of each edge of that neighboring state (taking into account the change in global parameter state). This approach is necessary because the global parameter state is mutated by sending this input the SUT which in turn changes the gains of the edges. The procedure is continued until the max_depth is reached. The result is a reachability tree which can be traversed to find the best SUT inputs.


     * we can also identify the largest gain neighborhoods in the state space - we can then create scenarios that start from that state increasing test efficiency
    """
    def __init__(self, data, weights, test, max_depth=1):
        self.data = data
        self.weights = weights
        self.max_depth = max_depth
        self.test = test
        self.initialize(data, test, weights)

    def print_graph_info(self):
        rospy.loginfo("Graph has %s states!" % len(self.graph))
        total = 0
        for node in self.graph:
            total += len(self.graph[node])
            rospy.loginfo("%s : %s edges (%s)" % (node, len(self.graph[node]), [x[0] for x in self.graph[node]]))
        rospy.loginfo("Graph has total %s edges!" % total)

    def initialize(self, data, test, weights):
        """
        Create the optimization graph
        """
        self.graph = self.create_graph(data, test, weights)
        self.print_graph_info()
        self.graph = self.merge_edges(self.graph)
        self.print_graph_info()

    def get_list_hash(self, items):
        """
        vector
        """
        return hashlib.sha256("".join(map(lambda x: str(x), items))).hexdigest()

    def create_parameter_dictionary(self, update, weights):
        """
        Args:
         pre -- dictionary of parameters, if the parameter is a list (e.g., "file.py": [1, 2, 3]) , then split it to ("file.py", 1): 1.0, ("file.py", 2): 1.0, ("file.py", 3): 1.0.
                If it is a single value, then add it directly (e.g., "uncertainty_measure": 2.3), then add it as ("uncertainty_measure", 2.3): 1.0
         post --same as above
         weights -- a list of weights [{'type': 'coverage', 'name': 'patrol_detector.py', 'weight': 1.0}, ... ], any param not in list of weights will not be added to the parameter dictionary
        """
        parameters = {}
        for key in update:
            for weight in weights:
                if weight['name'] in key:
                    if type(update[key]) == list:
                        for item in update[key]:
                            parameters[(key, item)] = 1.0
                    else:
                        parameters[(key, update[key])] = 1.0
        return parameters

    def create_graph(self, data, test, weights):
        """
        Returns:
         Graph in the form of a dictionary with keys as nodes and values as a list of edges. Each edge is itself a list of [destination, quantity, parameter vector as a dictionary]
         e.g., ["
        """
        graph = {}
        state_vector = {}
        pre = {}
        run_id = ""
        current_state = "INIT" # initial state
        for entry in data:
            if entry['test'] == test:
                if entry['run_id'] != run_id:
                    # New run begins, reset state
                    state_vector = {}
                    current_state = "INIT"
                    run_id = entry['run_id']
                channel = self.get_list_hash(entry['channel'].values())
                if entry['event'] == "PRE":
                    pre[channel] = entry
                else:
                    # Only process after receiving "POST"
                    state_vector[channel] = str(entry['data'])
                    new_state = self.get_list_hash(state_vector.values())
                    edges = graph.get(current_state, [])
                    pre[channel]['coverage'].update(entry['coverage'])
                    edges.append([new_state, 1, self.create_parameter_dictionary(pre[channel]['coverage'], weights)])
                    graph[current_state] = edges
                    current_state = new_state

        return graph

    def merge_edges(self, graph):
        """
        Merge identical edges
        """
        rospy.loginfo("Merging node edges...")
        for node in graph:
            merged = True
            while merged:
                merged = False
                for i in range(len(graph[node])-1):
                    if graph[node][i][2] == graph[node][i+1][2] and graph[node][i][0] == graph[node][i+1][0]:
                        graph[node][i][1] += 1
                        del graph[node][i+1]
                        merged = True
                        break
        return graph
                    

    def compute_edge_gains(self, state):
        """
        Compute the edge gains for a state based on the global parameter vector.

        Returns:
        The edge gains as 

        """
        pass

    def compute_sequence(self, state=None, gain_limit=0.0, step_limit=sys.maxint, time_limit=-1):
        """
        Compute the optimal sequence from the given state (note that initial state is "undefined" since we do not know in what state the simulation started).

        If state is None-- we assume that all runs have the same initial state and we can use it as the start state. If the log contains traces that might have different starting state, the state must be passed into this function to find the optimal sequence.

        Terminate the sequence when exceeding any limit.
        Limit -1 means unlimited for steps and time.
        Gain limit 0.0 means no limit since gain is nonnegative by definition.

        Returns:
         sequence - optimized sequence as a list of tuples (channel, data), e.g., ([({"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}, {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}), ... ])
        """
        pass



def optimize(log_data, weights, test):
    """
    Create an optimal test scenario based on the log entries given the weights.

    Args:
      log_data -- list of entries (e.g., [{"run_id": "fb741e78-4c70-4557-a572-e51f72567a05", "timestamp": 1475.0, "coverage": {}, "test": "Scenario #1", "data": {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}, "event": "PRE", "channel": {"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}}, ... ])
      weights -- list of weights (e.g., [{'type': 'coverage', 'name': 'patrol_detector.py', 'weight': 1.0}, ... ])
    Returns:
      optimized sequence as a list of tuples (channel, data), e.g., ([({"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}, {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}), ... ])
    """
    rospy.loginfo("Optimizing %s entries with weights %s for test '%s'" % (len(log_data), weights, test))

    optimizer = Optimizer(log_data, weights, test)
    sequence = optimizer.compute_sequence(step_limit = 1)
    rospy.loginfo("Optimizer sequence is: %s" % sequence)
    
    run = log_data[0]['run_id']
    sequence = []
    for entry in log_data:
        if entry['run_id'] != run:
            break
        if entry['event'] == "PRE":
            continue
        step = (entry['channel'], entry['data'])
        #rospy.loginfo(step[1]['target_pose']['pose']['position'])
        sequence.append(step)

    rospy.loginfo(sequence)
    return sequence
