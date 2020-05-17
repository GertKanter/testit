from collections import defaultdict
from math import sqrt

import numpy as np
import rospy

from util import flatten, lmap

try:
    from typing import *
except:
    pass


class ModelRefinementMoveStrategy:
    def __init__(self, **kwargs):
        self.state_machine = kwargs['state_machine']
        if not self.state_machine:
            rospy.logerr("stateMachine not specified in config")
            raise RuntimeError("stateMachine not specified in config")
        self.state_values = {int(key): self.state_machine['values'][key] for key in self.state_machine['values']}
        self.edges = {int(key): lmap(int, self.state_machine['edges'][key]) for key in self.state_machine['edges']}
        self.edge_labels = {eval(key): self.state_machine['labels'][key] for key in self.state_machine['labels']}
        self.timestamps = {eval(key): self.state_machine['timestamps'][key] for key in self.state_machine['timestamps']}
        self.initial_state = int(self.state_machine['initialState'])

        self.state_machine['values'] = self.state_values
        self.state_machine['edges'] = self.edges
        self.state_machine['labels'] = self.edge_labels
        self.state_machine['timestamps'] = self.timestamps

        self.topics = []
        self.actions = []
        self.action_lens = []
        self.visited = set()
        self.inaccessible = defaultdict(set)
        self.path = []
        self.path_cursor = 0
        self.prev_state = None
        self.state = None
        self.next_state = None
        self.connecting = False
        self.success = True
        self.closest_pairs = None
        self.going_back = False

    def set_initial_state(self, _):
        self.state = self.initial_state
        self.prev_state = self.state
        self.visited.add(self.state)
        self.path.append(self.state)
        self.closest_pairs = self.get_closest_pairs()

    def set_previous_states(self, states):
        pass

    def give_feedback(self, successes):
        if any(successes):
            self.success = True
            if self.state not in self.edges.get(self.prev_state, []) and self.prev_state != self.state:
                self.edges[self.prev_state].append(self.state)
                index = successes.index(True)
                self.edge_labels[(self.prev_state, self.state)] = self.topics[index]['identifier']
            if (self.prev_state, self.state) in self.timestamps:
                self.timestamps[(self.prev_state, self.state)].append(rospy.time())
            else:
                self.timestamps[(self.prev_state, self.state)] = [rospy.time()]
            self.prev_state = self.state
            self.visited.add(self.state)
            if not self.going_back:
                self.path.append(self.state)
        else:
            self.success = False
            if self.prev_state:
                self.inaccessible[self.prev_state].add(self.state)
                if self.prev_state in self.edges and self.state in self.edges[self.prev_state]:
                    self.edges[self.prev_state].remove(self.state)
                    del self.edge_labels[(self.prev_state, self.state)]
                self.state = self.prev_state

    def add(self, actions, topic):
        self.actions += actions
        self.action_lens.append(len(self.actions) // 2)
        self.topics.append(topic)

    def state_values_to_states(self, value):
        states = []
        for i, length in enumerate(self.action_lens):
            states.append(value[i][:length])
        return states

    def get_distance(self, state1, state2, convert=(True, True)):
        if convert[0]:
            state1 = flatten(self.state_values_to_states(state1))
        if convert[1]:
            state2 = flatten(self.state_values_to_states(state2))
        distance = sqrt(sum(map(lambda coords: (float(coords[1]) - float(coords[0])) ** 2, zip(state1, state2))))
        return distance

    def get_closest_pairs(self):
        pairs_by_distance = []
        for state1 in self.state_values:
            value1 = self.state_values[state1]
            for state2 in self.state_values:
                if state1 == state2 or state2 in self.edges.get(state1, []) or state2 in self.inaccessible.get(state1,
                                                                                                               []):
                    continue
                value2 = self.state_values[state2]
                distance = self.get_distance(value1, value2)
                pairs_by_distance.append((distance, (state1, state2)))

        pairs_by_distance.sort(key=lambda triple: triple[0])

        shortest_pairs_by_destination = {}
        distance_by_destination = {}
        for (distance, (x, y)) in pairs_by_distance:
            if y in shortest_pairs_by_destination:
                distance_ = distance_by_destination[y]
                if distance < distance_:
                    shortest_pairs_by_destination[y] = x
                    distance_by_destination[y] = distance
            else:
                shortest_pairs_by_destination[y] = x
                distance_by_destination[y] = distance
        return {shortest_pairs_by_destination[key]: key for key in shortest_pairs_by_destination}

    def get_state_label(self, state):
        return min(self.state_values,
                   key=lambda s: self.get_distance(state, self.state_values[s], convert=(False, True)))

    def state_value(self):
        states = self.state_values_to_states(self.state_values[self.state])
        self.timestamp = rospy.time()
        rospy.loginfo(str(states))
        return states

    def get_next_states(self):
        self.going_back = False
        if self.next_state is not None and self.success:
            self.prev_state = self.state
            self.state = self.next_state
            if self.connecting:
                rospy.loginfo("Going to closest pair state: " + str(self.state))
                self.next_state = self.prev_state
                self.connecting = False
            else:
                rospy.loginfo("Going back to path: " + str(self.state))
                self.next_state = None
            return self.state_value()

        for state in self.edges[self.prev_state]:
            if state not in self.visited and not state in self.inaccessible[self.state]:
                if state in self.closest_pairs \
                        and not self.closest_pairs[state] in self.inaccessible.get(state, []):
                    self.state = state
                    self.next_state = self.closest_pairs[state]
                    self.connecting = True
                    rospy.loginfo("Found closest pair: " + str(self.state) + " -> " + str(self.next_state))
                    rospy.loginfo("Going to state: " + str(self.state))
                    return self.state_value()
                rospy.loginfo("Going to regular state: " + str(state))
                self.state = state
                return self.state_value()

        if len(self.path) == 0:
            return None

        self.going_back = True
        self.state = self.path.pop()
        rospy.loginfo("Going back in path: " + str(self.state))
        return self.state_value()
