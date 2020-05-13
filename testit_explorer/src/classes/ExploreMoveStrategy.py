from math import sqrt
from util import flatten, lmap

import rospy

try:
    from typing import *
except:
    pass


class ExploreMoveStrategy:
    def __init__(self):
        self.initial_state = tuple()
        self.state = tuple()
        self.next_state = tuple()
        self.actions = []
        self.splits = []

        self.inaccessible = set()
        self.visited = set()
        self.path = list()
        self.backtrace = list()

        self.current_path = []

    def add(self, actions, topic):
        self.add_actions(actions)
        self.add_split()

    def set_initial_state(self, initial_state):
        initial_state = tuple(initial_state)
        self.initial_state = initial_state
        self.state = initial_state
        self.path.append(initial_state)

    def set_previous_states(self, states):
        flat_states = flatten(states)
        self.path = flat_states
        self.visited = set(flat_states)

    def add_split(self):
        self.splits.append(len(flatten(self.actions, tuple)))

    def split(self, combined_state):
        states = []
        prev_split = 0
        if combined_state is None:
            return None
        for split in self.splits:
            state = combined_state[prev_split:split]
            states.append(state)
            prev_split = split
        return states

    def add_actions(self, actions):
        self.actions.append(tuple(actions))

    def get_reward(self, state):
        distance_from_path = 0
        for s in self.path:
            distance_from_path += self.get_distance(state, s)
        if state in self.visited:
            distance_from_path = -1
        return distance_from_path

    def get_distance(self, state1, state2):
        return sqrt(sum(map(lambda coords: (float(coords[1]) - float(coords[0])) ** 2, zip(state1, state2))))

    def get_state_combinations(self, new_states, state, *i):
        combined_states = []
        if len(i) == len(new_states):
            return [state]
        for j, states in enumerate(new_states):
            if j in i:
                continue
            if not states:
                return [state]
            for s in states:
                combs = self.get_state_combinations(new_states, state + s, j, *i)
                for c in combs:
                    combined_states.append(c)
        return combined_states

    def get_combined_possible_new_states(self, new_states):
        combined_states = []
        i, max_length_states = max(enumerate(new_states), key=lambda x: len(x[1]))
        for state in max_length_states:
            s = self.get_state_combinations(new_states, state, i)
            combined_states += s
        return combined_states

    def find_path(self, source, state):
        rospy.loginfo("Finding path from " + str(source) + " to " + str(state))
        if source is None or state is None:
            return

        if self.get_distance(self.state, state) == 1:
            self.current_path = [state]
            return

        self.current_path = []
        for s in self.path:
            self.current_path.append(state)
            if s == source:
                break
        self.current_path.append(state)
        rospy.loginfo("Found path: " + str(self.current_path))

    def find_new_path(self):
        rospy.loginfo("Finding new path")
        print("Actions")
        print(self.actions)
        if not self.actions:
            self.current_path = [self.state]
            return

        backtrace = []

        new_states = lmap(lambda actions: tuple(action.to_state(self.state) for action in actions), self.actions)
        combined_possible_states = self.get_combined_possible_new_states(new_states)
        for new_state in combined_possible_states:
            if new_state in self.inaccessible or new_state in self.visited:
                continue
            reward = self.get_reward(new_state)
            backtrace.append((reward, new_state, tuple(self.state)))
        backtrace.sort(key=lambda pair: pair[0])
        self.backtrace += lmap(lambda pair: pair[1:], backtrace)
        next_state, source = self.backtrace.pop() if self.backtrace else tuple([None, None])
        self.find_path(source, next_state)

    def get_next_states(self, retry=False):
        if len(self.current_path) > 0:
            self.next_state = self.current_path.pop()
            return self.split(self.next_state)

        if retry:
            return None

        self.find_new_path()
        return self.get_next_states(True)

    def give_feedback(self, successes):
        if not any(successes):
            self.inaccessible.add(self.next_state)
            self.next_state = None
        else:
            self.path.append(self.next_state)
            self.visited.add(self.next_state)
            self.state = self.next_state
            self.next_state = None
