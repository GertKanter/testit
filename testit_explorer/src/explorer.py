#!/usr/bin/env python

import atexit
import json
import re
import threading
import yaml
from collections import OrderedDict
from copy import deepcopy

import rospy
import testit_explorer
from std_msgs.msg import Bool
from testit_learn.msg import StateMachine
from testit_learn.srv import StateMachineToUppaal, StateMachineToUppaalResponse, StateMachineToUppaalRequest


from classes.Action import Action
from classes.ExploreMoveStrategy import ExploreMoveStrategy
from classes.ModelRefinementMoveStrategy import ModelRefinementMoveStrategy
from classes.MoveStrategyFromService import MoveStrategyFromService
from classes.RobotMover import RobotMover
from classes.util import lmap, flatten, get_attribute, add_to_list_dict

try:
    from typing import *
except:
    pass


class Explorer:
    def __init__(self):
        self.robot_mover_factory = RobotMover
        self.move_strategy_factory = ExploreMoveStrategy

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
            rospy.Subscriber("/testit/finished/%s" % self.test_config.get('tag'), Bool, self.maybe_write_new_model)
            self.move_strategy_factory = lambda: ModelRefinementMoveStrategy(state_machine=self.get_state_machine())
        elif self.test_config.get('moveStrategyService', '') != '':
            self.move_strategy_factory = lambda: MoveStrategyFromService(
                service=self.test_config['moveStrategyService'],
                init_topic=self.test_config.get('moveStrategyInitTopic', ''),
                state_machine=self.get_state_machine())
        else:
            self.move_strategy_factory = ExploreMoveStrategy

    def get_actions_constants_initial_of_topic(self, topic):
        constants, variables = topic['explore'].get('constants', []), topic['explore'].get('variables', [])
        steps = lmap(lambda variable: variable['step'], variables)
        constants = lmap(lambda constant: constant['value'], constants)
        initial = lmap(lambda variable: variable['initial'], variables)
        return Action.get_actions(steps), constants, initial

    def init_synced_topics_robot_movers(self):
        for synced_topics in self.synced_topics:
            initial_state = []
            robot_move_strategy = self.move_strategy_factory()
            # type: ExploreMoveStrategy or ModelRefinementMoveStrategy or MoveStrategyFromService
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
            robot_move_strategy = self.move_strategy_factory()
            robot_move_strategy.add(actions, topic)
            robot_move_strategy.set_initial_state(initial_state)
            self.robot_movers.append(self.robot_mover_factory(robot_move_strategy, topic))

    def init_robot_movers(self):
        self.init_synced_topics_robot_movers()
        self.init_not_synced_topics_robot_movers()
        rospy.sleep(1)

    def get_state_machine(self):
        if self.state_machine is not None:
            return self.state_machine
        path = self.test_config.get('stateMachine', None)
        if path is None:
            return None
        state_machine_path = rospy.get_param('/testit/pipeline')['sharedDirectory'] + path
        with open(state_machine_path, 'r') as file:
            self.state_machine = yaml.load(file)
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

    def maybe_move_to_last_state_in_log(self):
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

    def get_encoded_statemachine(self):
        state_machine = deepcopy(self.state_machine)
        state_machine['labels'] = {str(key): value for key, value in self.state_machine['labels'].iteritems()}
        return state_machine

    def get_statemachine_msg(self):
        state_machine = self.get_encoded_statemachine()
        statemachine = StateMachine()
        statemachine.edges = json.dumps(state_machine['edges'])
        statemachine.labels = json.dumps(state_machine['labels'])
        statemachine.values = json.dumps(state_machine['values'])
        statemachine.initialState = str(state_machine['initialState'])
        return statemachine

    def call_uppaal_service(self, service, input_types):
        service.wait_for_service()

        request = StateMachineToUppaalRequest()
        request.test = self.test_config['tag']
        request.stateMachine = self.get_statemachine_msg()
        request.inputTypes = input_types

        return service(request)

    def write_model(self, input_types, uppaalModel):
        file_name = self.test_config['tag'] + ''.join(
            map(lambda id: ''.join(map(lambda x: x[0], id.strip('/').replace('/', '_').split('_'))),
                input_types))
        model_path = file_name + '-refined_model.xml'
        statemachine_path = file_name + '-refined_statemachine.json'
        directory = rospy.get_param('/testit/pipeline')['sharedDirectory'].strip('/') + '/' + \
                    rospy.get_param('/testit/pipeline')['resultsDirectory'].strip('/') + '/'
        rospy.loginfo("Writing refined model to " + directory)

        with open(directory + model_path, 'w') as file:
            file.write(uppaalModel)
        with open(directory + statemachine_path, 'w') as file:
            file.write(json.dumps(self.get_encoded_statemachine()))

    def maybe_write_new_model(self, req=Bool(True)):
        if self.test_config.get('mode', 'test') != 'refine-model' or not req.data:
            return

        rospy.loginfo("\nWriting refined model")
        statemachine_to_uppaal_service = self.test_config.get('stateMachineToUppaalService',
                                                              '/testit/learn/statemachine/uppaal')
        get_uppaal = rospy.ServiceProxy(statemachine_to_uppaal_service, StateMachineToUppaal)

        input_types_matrix = list(
            map(lambda topics: [self.topics[i]['identifier'] for i in topics], self.synced_topics))
        for input_types in input_types_matrix:
            response = self.call_uppaal_service(get_uppaal, input_types)  # type: StateMachineToUppaalResponse
            self.write_model(input_types, response.uppaalModel.uppaalModel)

    def get_steps(self):
        steps = []
        if self.test_config.get('continuousUpdate', False):
            steps.append(self.maybe_write_new_model)
        return tuple(steps)

    def explore(self):
        self.maybe_move_to_last_state_in_log()

        threads = []
        for i, robot_mover in enumerate(self.robot_movers):
            rospy.loginfo('Exploring topics: ' + str(robot_mover.topic_identifiers))
            thread = threading.Thread(target=robot_mover.move, args=self.get_steps())
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        self.maybe_write_new_model()


if __name__ == '__main__':
    explorer = Explorer()
    atexit.register(explorer.maybe_write_new_model)
    explorer.explore()
