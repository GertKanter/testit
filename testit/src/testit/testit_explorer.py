#!/usr/bin/env python

from __future__ import print_function

import importlib
import json
import re
import subprocess
import threading
import yaml
import atexit
from collections import OrderedDict, defaultdict
from math import sqrt

import rospy
from std_msgs.msg import Bool
from testit_learn.msg import StateMachine

from testit_learn.srv import StateMachineToUppaal, StateMachineToUppaalResponse, StateMachineToUppaalRequest, \
    WriteUppaalModel, WriteUppaalModelRequest

try:
    from typing import *
except:
    pass


def flatten(array, to=list):
    return to(reduce(lambda a, b: a + b, array, to()))


def is_numeric(value):
    return unicode(value).isnumeric() or isinstance(value, float)


def dynamic_import(path):
    path_list = path.split('.')
    module = '.'.join(path_list[:-1])
    function = path_list[-1]

    return getattr(importlib.import_module(module), function)


def get_attribute(value, path):
    get_value = getattr if not isinstance(value, dict) else lambda v, a: v.get(a)
    for attribute in path.split('.'):
        value = get_value(value, attribute)
    return value


def set_attribute(object, field_path, value):
    field_path_list = field_path.split('.')
    for field in field_path_list[:-1]:
        object = getattr(object, field)
    setattr(object, field_path_list[-1], value)


def add_to_list_dict(dictionary, key, value):
    if key in dictionary:
        dictionary[key].append(value)
    else:
        dictionary[key] = [value]


def lmap(fn, xs):
    return list(map(fn, xs))


def execute_command(command, prefix='', suffix=''):
    """
    Process paths with bash commands.
    E.g., '$(rospack find testit)/data/' to '/home/user/catkin_ws/src/testit/testit/data/'
    """
    if prefix == "":
        process = subprocess.Popen(['/bin/bash', '-c', 'echo ' + command], stdout=subprocess.PIPE)
    else:
        cmd = prefix + "/bin/bash -c '\\''echo " + command + "'\\''" + suffix
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    out, err = process.communicate()
    out = out.replace("\n", "")
    return out


class Action:
    def __init__(self, index, step):
        self.index = index
        self.step = step

    def to_state(self, state):
        new_state = list(state)
        new_state[self.index] += self.step
        return tuple(new_state)

    @staticmethod
    def get_actions(steps):
        actions = []
        for index, step in enumerate(steps):
            actions.append(Action(index, step))
            if step != -step:
                actions.append(Action(index, -step))
        return actions


class ModelRefinementMoveStrategy:
    def __init__(self, **kwargs):
        self.state_machine = kwargs['state_machine']
        if not self.state_machine:
            rospy.logerr("stateMachine not specified in config")
            raise RuntimeError("stateMachine not specified in config")
        self.state_values = {int(key): self.state_machine['values'][key] for key in self.state_machine['values']}
        self.edges = {int(key): self.state_machine['edges'][key] for key in self.state_machine['edges']}
        self.edge_labels = {eval(key): self.state_machine['labels'][key] for key in self.state_machine['labels']}

        self.state_machine['values'] = self.state_values
        self.state_machine['edges'] = self.edges
        self.state_machine['labels'] = self.edge_labels

        self.topics = []
        self.actions = []
        self.action_lens = []
        self.initial_state = None
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

    def set_initial_state(self, state):
        self.initial_state = self.get_state_label(state)
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
        print(states)
        return states

    def get_next_states(self):
        self.going_back = False
        if self.next_state is not None and self.success:
            self.prev_state = self.state
            self.state = self.next_state
            if self.connecting:
                print("Going to closest pair state: " + str(self.state))
                self.next_state = self.prev_state
                self.connecting = False
            else:
                print("Going back to path: " + str(self.state))
                self.next_state = None
            return self.state_value()

        for state in self.edges[self.prev_state]:
            if state not in self.visited and not state in self.inaccessible[self.state]:
                if state in self.closest_pairs \
                        and not self.closest_pairs[state] in self.inaccessible.get(state, []):
                    self.state = state
                    self.next_state = self.closest_pairs[state]
                    self.connecting = True
                    print("Found closest pair: " + str(self.state) + " -> " + str(self.next_state))
                    print("Going to state: " + str(self.state))
                    return self.state_value()
                print("Going to regular state: " + str(state))
                self.state = state
                return self.state_value()

        if len(self.path) == 0:
            return None

        self.going_back = True
        self.state = self.path.pop()
        print("Going back in path: " + str(self.state))
        return self.state_value()


class MoveStrategy:
    def __init__(self, **kwargs):
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
        print("Finding path from " + str(source) + " to " + str(state))
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
        print("Found path: " + str(self.current_path))

    def find_new_path(self):
        print("Finding new path")
        print("self.state=" + str(self.state))
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


class RobotMover:
    def __init__(self, robot_move_strategy, topics):
        # type: (MoveStrategy, List[dict]) -> None
        self.robot_move_strategy = robot_move_strategy
        self.topic_identifiers = lmap(lambda topic: topic['identifier'], topics)
        self.topics_values = lmap(lambda topic: topic['explore'], topics)
        self.continue_enabled = all(map(lambda topic: topic['explore'].get('continue', False), topics))
        self.publishers_types = lmap(lambda topic: dynamic_import(topic['type']), topics)
        self.publishers = lmap(
            lambda topic: rospy.Publisher(topic[1]['identifier'], self.publishers_types[topic[0]], queue_size=1),
            enumerate(topics))
        self.responders = lmap(lambda topic: self.get_responder(topic), topics)

    def get_responder(self, topic):
        def responder():
            result = rospy.wait_for_message(topic['feedback']['topic'], dynamic_import(topic['feedback']['type']))
            result = get_attribute(result, topic['feedback']['field'])
            return topic['feedback'].get('success', result) == result or re.match(str(topic['feedback']['success']),
                                                                                  str(result)) is not None

        return responder

    def get_goal(self, i, state):
        goal = self.publishers_types[i]()

        for constant in self.topics_values[i].get('constants', []):
            set_attribute(goal, constant['field'], constant['value'])

        for i, variable in enumerate(self.topics_values[i].get('variables', [])):
            set_attribute(goal, variable['field'], state[i])

        return goal

    def publish_goal(self, i, state):
        goal = self.get_goal(i, state)
        rospy.loginfo(self.topic_identifiers[i] + ' publishing goal ...\n' + str(goal))
        self.publishers[i].publish(goal)

    def move_to_continuation_state(self, states_matrix):
        self.robot_move_strategy.set_previous_states(states_matrix[::-1])
        initial_state = ()
        for i, states in enumerate(states_matrix):
            for state in states:
                if self.move_to_state(i, state):
                    initial_state += state
                    break
        self.robot_move_strategy.set_initial_state(initial_state)

    def move_to_state(self, i, state):
        self.publish_goal(i, state)
        return self.responders[i]()

    def log_for_each_topic(self, msg, suffixes=[]):
        for i, topic_identifier in enumerate(self.topic_identifiers):
            suffix = ""
            if i < len(suffixes):
                suffix = ' ' + str(suffixes[i])
            rospy.loginfo(topic_identifier + ' ' + msg + suffix)

    def get_steps(self, additional_steps):
        locks = [threading.Lock() for _ in additional_steps]
        steps = []
        for i, step in enumerate(additional_steps):
            def wrapped_step():
                lock = locks[i]
                if lock.locked():
                    rospy.loginfo("Locked")
                    return
                lock.acquire()
                rospy.loginfo("Executing step")
                step()
                rospy.loginfo("Step executed")
                lock.release()

            steps.append(wrapped_step)
        return steps

    def move(self, *additional_steps):
        steps = self.get_steps(additional_steps)

        while True:
            states = self.robot_move_strategy.get_next_states()

            if not states:
                self.log_for_each_topic('is unable to get to next state')
                break

            responses = lmap(lambda args: self.move_to_state(*args), enumerate(states))

            self.robot_move_strategy.give_feedback(responses)
            self.log_for_each_topic(' goal reached?: ', responses)

            for step in steps:
                threading.Thread(target=step).start()


class Explorer:
    def __init__(self):
        self.robot_mover_factory = RobotMover
        self.move_strategy_factory = MoveStrategy

        self.goal_type = None
        self.feedback_type = None
        self.initial_state = None
        self.state_machine = None
        self.topics = None
        self.synced_topics = None
        self.log = None
        self.test_config = None
        self.actions = []
        self.robot_movers = []  # type: List[RobotMover]

        self.init_ros()
        self.read_config()
        self.read_test_config()
        self.read_log()
        self.set_move_strategy_factory()
        self.init_robot_movers()

    def init_ros(self):
        rospy.init_node('explore', anonymous=True)
        rospy.sleep(1)

    def read_config(self):
        logger_config_path = rospy.get_param('testit_logger/config')
        with open(logger_config_path, 'r') as file:
            logger_config = yaml.load(file)
        self.synced_topics = logger_config['configuration'].get('syncedExploreTopics', [])
        self.topics = logger_config['configuration']['inputs']

    def read_test_config(self):
        tests = rospy.get_param('testit/tests')
        test_tag = rospy.get_param('testit_logger/test')
        for test in tests:
            if test['tag'] == test_tag:
                self.test_config = test
                return

    def set_move_strategy_factory(self):
        if self.test_config['mode'] == 'refine-model':
            self.move_strategy_factory = ModelRefinementMoveStrategy
        else:
            self.move_strategy_factory = MoveStrategy

    def get_actions_constants_initial_of_topic(self, topic):
        constants, variables = topic['explore'].get('constants', []), topic['explore'].get('variables', [])
        steps = lmap(lambda variable: variable['step'], variables)
        constants = lmap(lambda constant: constant['value'], constants)
        initial = lmap(lambda variable: variable['initial'], variables)
        return Action.get_actions(steps), constants, initial

    def init_synced_topics_robot_movers(self):
        for synced_topics in self.synced_topics:
            initial_state = []
            robot_move_strategy = self.move_strategy_factory(
                state_machine=self.get_state_machine())  # type: MoveStrategy or ModelRefinementMoveStrategy
            synced_topics_configs = []
            for i in synced_topics:
                topic = self.topics[i]
                synced_topics_configs.append(topic)
                topic_actions, topic_constants, initial = self.get_actions_constants_initial_of_topic(topic)
                initial_state += initial
                robot_move_strategy.add(topic_actions, topic)
            robot_move_strategy.set_initial_state(initial_state)
            self.robot_movers.append(self.robot_mover_factory(robot_move_strategy, synced_topics_configs))

    def init_not_synced_topics_robot_movers(self):
        flattened_synced_topics = flatten(self.synced_topics)
        for i, topic in enumerate(self.topics):
            if i in flattened_synced_topics:
                continue
            actions, constants, initial_state = self.get_actions_constants_initial_of_topic(topic)
            robot_move_strategy = self.move_strategy_factory(state_machine=self.get_state_machine())
            robot_move_strategy.add(actions, topic)
            robot_move_strategy.set_initial_state(initial_state)
            self.robot_movers.append(self.robot_mover_factory(robot_move_strategy, topic))

    def init_robot_movers(self):
        self.init_synced_topics_robot_movers()
        self.init_not_synced_topics_robot_movers()
        rospy.sleep(1)

    def get_state_machine(self):
        print("getting state machine")
        if self.state_machine is not None:
            return self.state_machine
        path = self.test_config.get('stateMachine', None)
        if path is None:
            return None
        state_machine_path = rospy.get_param('/testit/pipeline')['sharedDirectory'] + path
        with open(state_machine_path, 'r') as file:
            self.state_machine = yaml.load(file)
        rospy.Subscriber("/testit/finished/%s" % self.test_config.get('tag'), Bool, self.maybe_write_new_model)
        return self.state_machine

    def read_log(self):
        logger_path = rospy.get_param('/testit_logger/log')
        try:
            with open(logger_path, "r") as file:
                lines = file.read().strip(' ').strip('\n').split('\n')
            self.log = lmap(json.loads, lines)
        except IOError:
            self.log = []

    def find_input_from_config(self, post_line):
        for input_config in self.topics:
            if input_config['identifier'] == post_line['channel']['identifier']:
                return input_config

    def find_response(self, lines, input_config):
        response_topic = input_config['feedback']['topic']
        for line in lines:
            if line['event'] == 'RESPONSE' and line['channel']['identifier'] == response_topic:
                return line

    def maybe_get_input_config_and_response(self, log, i, require_success_response):
        line = log[i]
        if not line['event'] == 'POST':
            return False, False

        if not require_success_response:
            return False, True

        lines = log[i + 1:]
        input_config = self.find_input_from_config(line)
        if not input_config.get('feedback', False):
            return False, True

        response = self.find_response(lines, input_config)
        if not response:
            return False, False

        return True, (input_config, response)

    def post_success(self, log, i, require_success_response=True):
        success, value = self.maybe_get_input_config_and_response(log, i, require_success_response)
        if not success:
            return value
        input_config, response = value

        field_value = get_attribute(response['data'], input_config['feedback']['field'])
        return input_config['feedback'].get('success', field_value) == field_value or re.match(
            str(input_config['feedback']['success']), str(field_value)) is not None

    def get_logs_by_tests(self):
        logs_by_tests = OrderedDict()
        for i, line in enumerate(self.log):
            test_tag = line['test']
            if line['event'] == 'RESPONSE':
                continue
            line['success'] = self.post_success(self.log, i)
            add_to_list_dict(logs_by_tests, test_tag, line)
        for test in logs_by_tests:
            run_ids = lmap(lambda line: line['run_id'], logs_by_tests[test])
            logs_by_tests[test].sort(key=lambda line: line['timestamp'])
            logs_by_tests[test].sort(key=lambda line: run_ids.index(line['run_id']))
        return logs_by_tests

    def synced_topic_success(self, log, line_nr):
        line = log[line_nr]
        line_id = line['channel']['identifier']
        topic_index = next(i for i, topic in enumerate(self.topics) if topic['identifier'] == line_id)

        before = []
        after = []
        for synced_topics in self.synced_topics:
            if topic_index in synced_topics:
                index = synced_topics.index(topic_index)
                before += synced_topics[:index + 1]
                after += synced_topics[index + 1:]

        before = [self.topics[i]['identifier'] for i in before]
        after = [self.topics[i]['identifier'] for i in after]

        if before:
            for line in log[:line_nr][::-1][:len(before) * 2]:
                if line['channel']['identifier'] in before:
                    if line.get('success', False):
                        return True
        if after:
            for line in log[line_nr + 1:][:len(after) * 2]:
                if line['channel']['identifier'] in after:
                    if line.get('success', False):
                        return True

        return False

    def find_last_states(self, log, robot_mover):
        # type: (dict, RobotMover) -> List[List[Tuple]]
        states_matrix = []
        for topic_index, topic_identifier in enumerate(robot_mover.topic_identifiers):
            states = []
            for line_nr, line in list(enumerate(log))[::-1]:
                if line['channel']['identifier'] == topic_identifier:
                    state = []
                    if not log[line_nr]['success'] or self.synced_topic_success(log, line_nr):
                        continue
                    for variable in robot_mover.topics_values[topic_index].get('variables', []):
                        state.append(get_attribute(line['data'], variable['field']))
                    states.append(tuple(state))
            states_matrix.append(states)
        return states_matrix

    def move_to_last_state_in_log(self):
        if self.test_config['mode'] != 'explore':
            return
        logs_by_tests = self.get_logs_by_tests()
        test_tag = rospy.get_param('/testit_logger/test')
        logs = logs_by_tests.get(test_tag, [])
        threads = []
        for robot_mover in self.robot_movers:
            if robot_mover.continue_enabled:
                last_states = self.find_last_states(logs, robot_mover)
                thread = threading.Thread(target=robot_mover.move_to_continuation_state, args=(last_states,))
                threads.append(thread)
                thread.start()
        for thread in threads:
            thread.join()

    def get_statemachine_msg(self):
        statemachine = StateMachine()
        statemachine.edges = json.dumps(self.state_machine['edges'])
        statemachine.labels = json.dumps({str(key): value for key, value in self.state_machine['labels'].iteritems()})
        statemachine.values = json.dumps(self.state_machine['values'])
        return statemachine

    def maybe_write_new_model(self, req=Bool(True)):
        rospy.loginfo("\nWriting refined model? " + str(req.data))
        if self.test_config['mode'] == 'refine-model' and req.data:
            statemachine_to_uppaal_service = self.test_config.get('stateMachineToUppaalService',
                                                                  '/testit/learn/statemachine/uppaal')
            get_uppaal = rospy.ServiceProxy(statemachine_to_uppaal_service, StateMachineToUppaal)
            input_types_matrix = list(
                map(lambda topics: [self.topics[i]['identifier'] for i in topics], self.synced_topics))
            for input_types in input_types_matrix:
                get_uppaal.wait_for_service()

                request = StateMachineToUppaalRequest()
                request.test = self.test_config['tag']
                request.stateMachine = self.get_statemachine_msg()
                rospy.loginfo(input_types)
                request.inputTypes = input_types

                response = get_uppaal(request)  # type: StateMachineToUppaalResponse

                file_name = self.test_config['tag'] + ''.join(
                    map(lambda id: ''.join(map(lambda x: x[0], id.strip('/').replace('/', '_').split('_'))),
                        input_types))
                model_path = file_name + '-refined_model.xml'
                file_path = rospy.get_param('/testit/pipeline')['sharedDirectory'].strip('/') + '/' + \
                            rospy.get_param('/testit/pipeline')['resultsDirectory'].strip('/') + '/' + \
                            model_path.strip('/')

                rospy.loginfo("Writing refined model to " + file_path)

                with open(file_path, 'w') as file:
                    file.write(response.uppaalModel.uppaalModel)

    def explore(self):
        self.move_to_last_state_in_log()

        threads = []
        for i, robot_mover in enumerate(self.robot_movers):
            rospy.loginfo('Exploring topics: ' + str(robot_mover.topic_identifiers))
            thread = threading.Thread(target=robot_mover.move, args=(self.maybe_write_new_model,))
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        self.maybe_write_new_model()


if __name__ == '__main__':
    explorer = Explorer()
    atexit.register(explorer.maybe_write_new_model)
    explorer.explore()
