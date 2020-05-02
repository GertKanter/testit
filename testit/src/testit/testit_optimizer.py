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
import random

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


     * we can also identify the largest gain neighborhoods in the state space - we can then create scenarios that start from that state to increase test efficiency
    """
    # Modes
    PROBABILISTIC = 1 # Ignores time and state history (best for high degree of controllability systems)
    BEST_TRACE = 2 # Finds the highest gain trace in the log (useful for low controllability systems, i.e., systems where the process takes time and it is not possible to influence it directly and probabilistic approach does not yield good results)
    COMBINED = 3 # Combines best trace mode with probabilistic mode. This mode combines both optimization graphs (best trace is extended with probabilistic)

    def __init__(self, data, weights, test, mode=None):
        self.data = data
        self.weights = weights
        #self.max_depth = max_depth
        self.test = test
        if mode is None:
            self.mode = Optimizer.PROBABILISTIC
        self.channel_hashes = {}
        self.state_hashes = {}
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
        #self.print_graph_info()
        self.graph = self.combine_edges(self.merge_edges(self.graph))
        #self.print_graph_info()

    def get_list_hash(self, items):
        """
        vector
        """
        return hashlib.sha256("".join(map(lambda x: str(x), items))).hexdigest()

    def create_parameter_dictionary(self, update, weights, weighted=False):
        """
        Args:
         update -- dictionary of parameters, if the parameter is a list (e.g., "file.py": [1, 2, 3]) , then split it to ("file.py", 1): 1.0, ("file.py", 2): 1.0, ("file.py", 3): 1.0.
                If it is a single value, then add it directly (e.g., "uncertainty_measure": 2.3), then add it as ("uncertainty_measure", 2.3): 1.0
         weights -- a list of weights [{'type': 'coverage', 'name': 'patrol_detector.py', 'weight': 1.0}, ... ], any param not in list of weights will not be added to the parameter dictionary
        """
        parameters = {}
        for key in update:
            for weight in weights:
                if weight['name'] in key:
                    if type(update[key]) == list:
                        for item in update[key]:
                            parameters[(key, item)] = 1.0 if not weighted else weight['weight'] 
                    else:
                        parameters[(key, update[key])] = 1.0 if not weighted else weight['weight']
        return parameters

    def create_graph(self, data, test, weights):
        """
        Returns:
         Graph in the form of a dictionary with keys as nodes and values as a list of edges. Each edge is itself a list of [destination, quantity, parameter vector as a dictionary, parameter weights vector as a dictionary]
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
                self.channel_hashes[channel] = entry['channel']
                if entry['event'] == "PRE":
                    pre[channel] = self.flatten_coverage(entry)
                elif entry['event'] == "POST":
                    # Only process after receiving "POST"
                    state_vector[channel] = str(entry['data'])
                    new_state = self.get_list_hash(state_vector.values())
                    self.state_hashes[new_state] = (state_vector, entry['data'])
                    edges = graph.get(current_state, [])
                    entry = self.flatten_coverage(entry)
                    pre[channel] = pre.get(channel, {'coverage': {}})
                    pre[channel]['coverage'].update(entry['coverage'])
                    edges.append([new_state, 1, self.create_parameter_dictionary(pre[channel]['coverage'], weights), self.create_parameter_dictionary(pre[channel]['coverage'], weights, weighted=True)])
                    graph[current_state] = edges
                    current_state = new_state
        return graph

    def flatten_coverage(self, entry):
        for key in entry['coverage']:
            coverage = set()
            if type(entry['coverage'][key]) == dict:
                for host in entry['coverage'][key]:
                    coverage.update(entry['coverage'][key][host])
                entry['coverage'][key] = sorted(list(coverage))
        return entry

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

    def combine_edges(self, graph):
        """
        Combine edges that lead from the same node to the same destination.
        """
        rospy.loginfo("Combining edges...")
        for node in graph:
            node_total = {} # node_total['destination'] = 1+1+1 ...
            node_data = {} # node_data['destination']['(xx, 1)'] = 1.0 + 1.0
            node_weights = {}
            edges_total = 0
            for edge in graph[node]:
                node_total[edge[0]] = node_total.get(edge[0], 0) + edge[1]
                edges_total += edge[1]
                for parameter in edge[2]:
                    field = node_data.get(edge[0], {})
                    field_weight = node_weights.get(edge[0], {})
                    if len(field) == 0:
                        node_data[edge[0]] = {}
                    if len(field_weight) == 0:
                        node_weights[edge[0]] = {}
                    node_data[edge[0]][parameter] = field.get(parameter, 0.0) + edge[2][parameter] * edge[1]
                    node_weights[edge[0]][parameter] = edge[3][parameter]
            # Normalize parameters according to probability (x / total)
            for destination in node_data:
                for parameter in node_data[destination]:
                    node_data[destination][parameter] /= node_total[destination]
            # Replace previous edges with new normalized edges
            graph[node] = []
            for destination in node_data:
                graph[node].append([destination, node_total[destination] / edges_total, node_data[destination], node_weights[destination]])
        return graph


    def compute_edge_gains(self, state, parameter_state):
        """
        Compute the edge gains for a state based on the current parameter state.

        Arguments:
         state -- state to inspect (hash string)
         parameter_state -- the current probabilities as a dictionary
        Returns:
         The edge gains as a dictionary of destinations as keys and values as gain values
         {'id': [90, {newparams}]}
        """
        gains = {}
        #rospy.loginfo(self.graph)
        #rospy.logwarn(state)
        edges = self.graph.get(state, [])
        for edge in edges: # All possible neighbors of the state
            # edge[0] = destination
            # edge[2] = probabilities dictionary
            # edge[3] = weights dictionary
            new_param_state = {key: value for key, value in parameter_state.items()}
            gain = 0.0
            for parameter in edge[2]: 
                current = parameter_state.get(parameter, None)
                probability = 0
                if current is None:
                    probability = edge[2][parameter]
                    gain += probability * edge[3][parameter]
                else:
                    # Already have some probability
                    probability = current + edge[2][parameter] - current * edge[2][parameter]
                    gain += (probability - current) * edge[3][parameter] # gain is increase in probability * weight
                new_param_state[parameter] = probability
            gains[edge[0]] = [gain, new_param_state]
        return gains

    def compute_gain(self, pre_state, post_state, post_weights):
        """
        Calculate the gain between two states

        if post_state = {}, gain = 0
        Essentially gain = (post-pre)*weight
        """
        gain = 0
        for key in post_state:
            if pre_state.get(key, None) is None:
                # Does not exist in pre_state, add
                gain += post_state[key] * post_weights[key]
            else:
                # Exists in both, update probability
                probability = pre_state[key] + post_state[key] - pre_state[key] * post_state[key]
                gain += probability * post_weights[key]
        return gain

            
    def expand_tree_element(self, tree, element, max_depth, depth=0):
        """
        Element is [id (gain tree dict key), gain (diff)
        """
        element_id = element[0]
        state = element[1]
        parameter_state = element[3]
        depth += 1
        edge_gains = self.compute_edge_gains(state, parameter_state)
        tree[element_id] = []
        for edge_gain in edge_gains:
            self.new_tree_id += 1
            new_element = [self.new_tree_id, edge_gain, edge_gains[edge_gain][0], edge_gains[edge_gain][1]]
            tree[element_id].append(new_element)
            if depth <= max_depth:
                tree = self.expand_tree_element(tree, new_element, max_depth, depth=depth)
        return tree

    def update_path_gain(self, tree, node, gain, path):
        path_copy = path[:]
        path_copy.append(node)
        for edge in tree[node]:
            edge.append(edge[2] + gain)
            edge.append(path_copy)
            if tree.get(edge[0], False):
                tree = self.update_path_gain(tree, edge[0], edge[2] + gain, path_copy)
        return tree

    def compute_step(self, max_depth, initial_state, parameter_state, selection_mode=0):
        """
        Args:
         selection_mode - 0=best, 1=random, 2=worst
        """
        self.new_tree_id = 0
        gain_tree = self.expand_tree_element({}, [self.new_tree_id, initial_state, {}, parameter_state], max_depth)
        #rospy.loginfo("gain_tree = {}".format(gain_tree))
        # Find path gain
        self.update_path_gain(gain_tree, 0, 0, [])
        #rospy.loginfo("gain_tree = {}".format(gain_tree))
        best_gain = None
        best_step = None
        best_param_state = parameter_state
        options = []
        for key in gain_tree:
            for child in gain_tree[key]:
                node = gain_tree.get(child[0], None)
                #rospy.loginfo("child = {}".format(child))
                if best_gain is None and len(child[5]) > 1:
                    best_gain = child[4]
                    best_step = child[5][1]
                if node is None:
                    # Terminal node
                    if (child[4] > best_gain and selection_mode == 0) or (child[4] < best_gain and selection_mode == 2) or (selection_mode == 1) or best_gain is None:
                        options.append(child[5][1])
                        best_gain = child[4]
                        best_step = child[5][1]
        if selection_mode == 1: # random
            best_step = random.choice(options)
        selected_step = None
        for edge in gain_tree[0]:
            if selected_step is None:
                selected_step = edge[1]
                best_param_state = edge[3]
            if edge[0] == best_step:
                selected_step = edge[1]
                best_param_state = edge[3]
                break
        #rospy.logwarn("selected_step = {}".format(selected_step))
        return (selected_step, best_param_state)

    def compute_parameter_state_value(self, state):
        """
        state -- dict of params as keys and probability as value
        """
        value = 0
        for key in state:
            value += state[key]
        return value

    def compute_sequence(self, state="INIT", gain_limit=0.0, step_limit=sys.maxint, time_limit=-1, max_depth=1):
        """
        Compute the optimal sequence from the given state (note that initial state is "undefined" since we do not know in what state the simulation started).

        If state is "INIT"-- we assume that all runs have the same initial state and we can use it as the start state. If the log contains traces that might have different starting state, the state must be passed into this function to find the optimal sequence.

        Terminate the sequence when exceeding any limit.
        Limit -1 means unlimited for steps and time.
        Gain limit 0.0 means no limit since gain is nonnegative by definition.

        Returns:
         sequence - optimized sequence as a list of tuples (channel, data), e.g., ([({"identifier": "/robot_0/move_base", "type": "move_base_msgs.msg.MoveBaseAction", "proxy": "/robot_0/move_base/proxy"}, {"target_pose": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map", "seq": 0}, "pose": {"position": {"y": 36.49, "x": 5.95, "z": 0.0}, "orientation": {"y": 0.0, "x": 0.0, "z": 0.0, "w": 1.0}}}}), ... ])
        """
        sequence = []
        next_step = [state, {}, 0]
        for _ in range(step_limit):
            next_step = self.compute_step(max_depth, next_step[0], next_step[1])
            rospy.logwarn("next_step = {}".format(next_step))
            if next_step[0] is not None:
                data = self.state_hashes[next_step[0]][1]
                channel = self.channel_hashes[self.state_hashes[next_step[0]][0].keys()[0]]
                sequence.append((channel, data))
            else:
                # Dead end
                break
        return sequence


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
    sequence = optimizer.compute_sequence(step_limit = 100, max_depth = 2)
    
    return sequence
