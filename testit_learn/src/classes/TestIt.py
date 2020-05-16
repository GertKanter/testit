import json
import math
import re
import yaml
from collections import OrderedDict
from util import get_attribute, add_to_list_dict

import rospy
import numpy as np


class TestIt:
    def __init__(self):
        self.log = self.read_log()
        self.logger_configs_by_tests = self.get_logger_configurations_by_tests()

    def read_config(self, path_param):
        config_path = rospy.get_param(path_param)
        with open(config_path, 'r') as file:
            return yaml.load(file)

    def read_log(self):
        log_path = rospy.get_param('/testit_logger/log')
        with open(log_path) as file:
            lines = file.read().strip(' ').strip('\n').split('\n')
        lines = list(map(json.loads, lines))
        run_ids = list(map(lambda line: line['run_id'], lines))
        lines.sort(key=lambda line: line['timestamp'])
        lines.sort(key=lambda line: run_ids.index(line['run_id']))
        return lines

    def get_coverage_line_count(self, log_line):
        coverage_by_files_and_robots = log_line['coverage']
        line_count = 0
        for file in coverage_by_files_and_robots:
            coverage_by_robots = coverage_by_files_and_robots[file]
            for robot in coverage_by_robots:
                line_count += len(coverage_by_robots[robot])
        return line_count

    def normalise(self, lists_by_test_and_input):
        for test in lists_by_test_and_input:
            test_configurations = self.logger_configs_by_tests[test]
            for input_ids in lists_by_test_and_input[test]:
                data = lists_by_test_and_input[test][input_ids]
                inputs = test_configurations['configuration']['inputs']
                input_configs = list(map(lambda input_id: self.find_input_config(inputs, input_id), input_ids))
                columns_to_normalise = self.get_columns_to_normalise(input_configs)
                self.normalise_columns(data, columns_to_normalise)

    def normalise_columns(self, points, columns):
        if not columns:
            return points
        for column in columns:
            columns_to_normalise_after = list(filter(lambda x: x not in columns, range(points.shape[1])))
            round_to = 1
            maximum_in_columns_to_normalise_after = math.ceil(
                np.amax(points[:, columns_to_normalise_after]) / round_to) * round_to
            maximum_in_normalised_column = np.amax(points[:, column])
            coefficient = maximum_in_normalised_column / maximum_in_columns_to_normalise_after
            points[:, column] = points[:, column] / coefficient

    @staticmethod
    def get_default_cluster(input_config):
        default_cluster_by = {'variables': list(range(0, len(input_config['explore'].get('variables', []))))}
        default_cluster_normalise = list(filter(lambda field: field in default_cluster_by, ['feedback', 'timestamp']))
        cluster = input_config.get('cluster', {})
        return {
            'by': cluster.get('by', default_cluster_by),
            'normalise': cluster.get('normalise', default_cluster_normalise)
        }

    def get_attributes_from_input_config(self, input_config, log_line):
        cluster_by = TestIt.get_default_cluster(input_config)['by']
        attributes = []
        attributes_by_explore = []
        for key in ('variables', 'constants'):
            fields = input_config['explore'].get(key, [])
            for field in fields:
                attributes_by_explore.append(get_attribute(log_line['data'], field['field']))
            for i in cluster_by.get(key, []):
                field = fields[i]
                attributes.append(float(get_attribute(log_line['data'], field['field'])))

        feedback = cluster_by.get('feedback', None)
        if feedback is not None:
            if log_line['success']['value']:
                attributes.append(cluster_by.get('success', 1))
            else:
                attributes.append(cluster_by.get('failure', 0))

        timestamp = cluster_by.get('timestamp', None)
        if timestamp is not None:
            if timestamp == 'feedback':
                attributes.append(log_line['success']['timestamp'])
            elif timestamp == 'input':
                attributes.append(log_line['timestamp'])

        return log_line['timestamp'], attributes_by_explore, attributes

    def find_input_config(self, inputs, input_id):
        for input_config in inputs:
            if input_id == input_config['identifier']:
                return input_config

    def synced_topic_success(self, log, line_nr, logger_config):
        synced_topics_matrix = logger_config['configuration']['syncedExploreTopics']
        topic_configs = logger_config['configuration']['inputs']
        line = log[line_nr]
        line_id = line['channel']['identifier']
        topic_index = next(i for i, topic in enumerate(topic_configs) if topic['identifier'] == line_id)

        before = []
        after = []
        for synced_topics in synced_topics_matrix:
            if topic_index in synced_topics:
                index = synced_topics.index(topic_index)
                before += synced_topics[:index + 1]
                after += synced_topics[index + 1:]

        before = [topic_configs[i]['identifier'] for i in before]
        after = [topic_configs[i]['identifier'] for i in after]

        if before:
            for line in log[:line_nr][::-1][:len(before) * 2]:
                if line['channel']['identifier'] in before:
                    if line.get('success', {}).get('value', False):
                        return True
        if after:
            for line in log[line_nr + 1:][:len(after) * 2]:
                if line['channel']['identifier'] in after:
                    if line.get('success', {}).get('value', False):
                        return True

        return False

    def get_columns_to_normalise(self, input_configs):
        normalised = []
        prev_fields_length = 0

        for input_config in input_configs:
            cluster_normalise = TestIt.get_default_cluster(input_config)['normalise']
            cluster_by = TestIt.get_default_cluster(input_config)['by']

            keys = ['variables', 'constants']
            for i, key in enumerate(keys):
                if key in cluster_normalise and key in cluster_by:
                    normalised += list(range(prev_fields_length, len(cluster_by[key]) + prev_fields_length))
                prev_fields_length += len(cluster_by.get(key, []))

            special_keys = ['feedback', 'timestamp']
            for i, key in enumerate(special_keys):
                if key in cluster_normalise and key in cluster_by:
                    normalised.append(prev_fields_length)
                if key in cluster_by:
                    prev_fields_length += 1

        return normalised

    def combine_synced_topics(self, np_arrays_by_test_and_input):
        for test in np_arrays_by_test_and_input:
            test_data_by_input = np_arrays_by_test_and_input[test]
            test_config = self.logger_configs_by_tests[test]['configuration']
            input_configs = test_config['inputs']

            for input_id in test_data_by_input.keys():
                test_data_by_input[(input_id,)] = test_data_by_input[input_id]
                del test_data_by_input[input_id]

            for synced_topics in test_config['syncedExploreTopics']:
                test_data_combined = None
                input_ids = []
                for topic_index in synced_topics:
                    input_id = input_configs[topic_index]['identifier']
                    input_ids.append(input_id)
                    test_data = test_data_by_input.get((input_id,))
                    if test_data is None:
                        continue
                    if test_data_combined is None:
                        test_data_combined = test_data
                    else:
                        if test_data.shape[0] > test_data_combined.shape[0]:
                            test_data = test_data[:test_data_combined.shape[0]]
                        elif test_data_combined.shape[0] > test_data.shape[0]:
                            test_data_combined = test_data_combined[:test_data.shape[0]]
                        test_data_combined = np.c_[test_data_combined, test_data]
                    del test_data_by_input[(input_id,)]
                test_data_by_input[tuple(input_ids)] = test_data_combined

    def get_np_arrays_by_test_and_input(self):
        lists_by_test_and_input = OrderedDict()
        dicts_by_test_and_input = OrderedDict()
        test_configurations = self.logger_configs_by_tests
        logs_by_tests = self.get_logs_by_tests()

        for test in test_configurations:
            lists_by_input = OrderedDict()
            dicts_by_input = OrderedDict()
            for line_nr, line in enumerate(logs_by_tests.get(test, [])):
                if line['event'] == 'RESPONSE' or not (
                        line['success']['value'] or self.synced_topic_success(logs_by_tests[test], line_nr,
                                                                              test_configurations[test])):
                    continue

                inputs = test_configurations[test]['configuration']['inputs']
                input_id = line['channel']['identifier']
                input_config = self.find_input_config(inputs, input_id)
                timestamp, attributes_by_explore, attributes = self.get_attributes_from_input_config(input_config, line)
                add_to_list_dict(dicts_by_input, input_id,
                                 {'success': line['success']['value'], 'timestamp': timestamp, 'attributes': attributes_by_explore})
                add_to_list_dict(lists_by_input, input_id, attributes)

            for input_id in lists_by_input:
                lists_by_input[input_id] = np.array(lists_by_input[input_id])

            if lists_by_input:
                lists_by_test_and_input[test] = lists_by_input
                dicts_by_test_and_input[test] = dicts_by_input

        self.combine_synced_topics(lists_by_test_and_input)
        self.normalise(lists_by_test_and_input)

        return lists_by_test_and_input, dicts_by_test_and_input

    def write_model(self, uppaal_automata, test, input_types, directory=""):
        file_name = directory + '/' + test + ''.join(
            map(lambda id: ''.join(map(lambda x: x[0], id.strip('/').replace('/', '_').split('_'))), input_types))
        model_path = file_name + '.xml'
        with open(model_path, 'w') as file:
            file.write(str(uppaal_automata))
        state_machine_path = file_name + '-state_machine.json'
        with open(state_machine_path, 'w') as file:
            file.write(json.dumps({'edges': uppaal_automata.edges,
                                   'labels': {str(key): uppaal_automata.edge_labels[key] for key in
                                              uppaal_automata.edge_labels},
                                   'values': uppaal_automata.centroids_by_state,
                                   'timestamps': uppaal_automata.timestamps_by_state,
                                   'initialState': str(uppaal_automata.initial_state)}, indent=2))
        model_config_path = file_name + '.yaml'
        with open(model_config_path, 'w') as file:
            file.write(yaml.safe_dump(dict(uppaal_automata.map), indent=2))
        model_adapter_config_path = file_name + '-adapter_config.yaml'
        with open(model_adapter_config_path, 'w') as file:
            file.write(yaml.safe_dump(dict(uppaal_automata.adapter_config), indent=2))

        return model_path, state_machine_path, model_config_path, model_adapter_config_path

    def get_logger_configurations_by_tests(self):
        confs_by_tests = dict()
        tests = rospy.get_param('testit/tests', [])
        for test in tests:
            conf_path = test['loggerConfiguration']
            with open(rospy.get_param('testit/pipeline/sharedDirectory') + conf_path, 'r') as file:
                confs_by_tests[test['tag']] = yaml.load(file.read())

        return confs_by_tests

    def find_input_from_config(self, post_line):
        logger_config = self.logger_configs_by_tests[post_line['test']]
        for input in logger_config['configuration']['inputs']:
            if input['identifier'] == post_line['channel']['identifier']:
                return input

    def find_response(self, lines, input):
        response_topic = input['feedback']['topic']
        for line in lines:
            if line['event'] == 'RESPONSE' and line['channel']['identifier'] == response_topic:
                return line

    def maybe_get_response_and_input_config(self, i, require_success_response):
        line = self.log[i]
        if not line['event'] == 'POST':
            return False, {'value': False, 'timestamp': None}
        if not require_success_response:
            return False, {'value': True, 'timestamp': None}

        lines = self.log[i + 1:]
        input_config = self.find_input_from_config(line)
        if not input_config.get('feedback', False):
            return False, {'value': True, 'timestamp': None}

        response = self.find_response(lines, input_config)
        if not response:
            return False, {'value': False, 'timestamp': None}

        return True, (response, input_config)

    def post_response_success(self, i, require_success_response=True):
        success, value = self.maybe_get_response_and_input_config(i, require_success_response)
        if not success:
            return False, value

        response, input_config = value
        field_value = get_attribute(response['data'], input_config['feedback']['field'])
        timestamp = get_attribute(response, 'timestamp')
        return True, {'value': input_config['feedback'].get('success', field_value) == field_value or \
                               re.match(str(input_config['feedback']['success']), str(field_value)) is not None,
                      'timestamp': timestamp}

    def get_logs_by_tests(self):
        logs_by_tests = OrderedDict()
        for i, line in enumerate(self.log):
            test_tag = line['test']
            success, response = self.post_response_success(i)
            if not success:
                continue
            line['success'] = response
            add_to_list_dict(logs_by_tests, test_tag, line)

        return logs_by_tests
