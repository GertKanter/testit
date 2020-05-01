#!/usr/bin/env python

import json
import math
import re
import seaborn as sns
import subprocess
import traceback
import warnings
import xml.etree.cElementTree as xml
import yaml
from collections import OrderedDict, defaultdict
from copy import deepcopy
from itertools import count
from sklearn.cluster import MiniBatchKMeans
from sklearn.exceptions import ConvergenceWarning
from sklearn.metrics import silhouette_score
from sklearn.neighbors import NearestCentroid

import matplotlib.pyplot as plt
import numpy as np
import rospy
import xml.dom.minidom as xmldom
from scipy.spatial import distance
from testit_learn.msg import ClusterPoint, StateMachine
from testit_learn.srv import StateMachineToUppaal, StateMachineToUppaalRequest, StateMachineToUppaalResponse, \
    WriteUppaalModel, WriteUppaalModelRequest, WriteUppaalModelResponse, LogToCluster, ClusterToStateMachine, \
    LogToClusterResponse, LogToClusterRequest, ClusterToStateMachineResponse, ClusterToStateMachineRequest


def flatten(array):
    return list(reduce(lambda x, y: x + y, array, []))


def get_attribute(value, path):
    for attribute in path.split('.'):
        value = value[attribute]
    return value


def add_to_list_dict(dictionary, key, value):
    if key in dictionary:
        dictionary[key].append(value)
    else:
        dictionary[key] = [value]


def is_numeric(value):
    return unicode(value).isnumeric() or isinstance(value, float)


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


class ServiceProvider:
    def __init__(self):
        rospy.init_node("testit_learn", anonymous=True)

        self.log_to_cluster = rospy.Service('/testit/learn/log/cluster', LogToCluster, self.log_to_cluster_service)
        self.cluster_to_statemachine = rospy.Service('/testit/learn/cluster/statemachine', ClusterToStateMachine,
                                                     self.cluster_to_statemachine_service)
        self.statemachine_to_uppaal = rospy.Service('/testit/learn/statemachine/uppaal', StateMachineToUppaal,
                                                    self.statemachine_to_uppaal_model_service)
        self.write_uppaal = rospy.Service('/testit/learn/write/uppaal', WriteUppaalModel,
                                          self.write_uppaal_model_service)

        rospy.loginfo("Started testit_learn services")
        rospy.spin()

    def get_main(self):
        return Main() \
            .set_test_it(TestIt) \
            .set_clusterer(Clusterer) \
            .set_uppaal_automata(UppaalAutomata)

    def log_to_cluster_service(self, req):
        # type: (LogToClusterRequest) -> LogToClusterResponse
        cluster_points = self.get_main().log_to_clusters(req.test, tuple(req.inputTypes))
        response = LogToClusterResponse()
        for (cluster, data) in cluster_points:
            point = ClusterPoint()
            point.cluster = cluster
            point.point = data
            response.data.append(point)
        return response

    def cluster_to_statemachine_service(self, req):
        # type: (ClusterToStateMachineRequest) -> ClusterToStateMachineResponse
        edges, edge_labels, _, centroids, initial_cluster = self.get_main().clusters_to_state_machine(req.data,
                                                                                                      req.test,
                                                                                                      tuple(
                                                                                                          req.inputTypes))
        convert = lambda d, value_to: {str(key): value_to(d[key]) for key in d}
        response = ClusterToStateMachineResponse()
        response.stateMachine.edges = json.dumps(convert(edges, lambda value: list(map(str, value))))
        response.stateMachine.labels = json.dumps(convert(edge_labels, str))
        response.stateMachine.values = json.dumps(convert(centroids, list))
        response.stateMachine.initialState = str(initial_cluster)
        return response

    def statemachine_to_uppaal_model_service(self, req):
        # type: (StateMachineToUppaalRequest) -> StateMachineToUppaalResponse
        uppaal_automata = self.get_main().uppaal_automata_from_state_machine(req.stateMachine, req.test,
                                                                             tuple(req.inputTypes))
        response = StateMachineToUppaalResponse()
        response.uppaalModel.uppaalModel = str(uppaal_automata)
        response.uppaalModel.stateMachine = req.stateMachine
        response.uppaalModel.adapterConfig = json.dumps(uppaal_automata.adapter_config)
        response.uppaalModel.modelConfig = json.dumps(uppaal_automata.map)
        return response

    def write_uppaal_model_service(self, req):
        # type: (WriteUppaalModelRequest) -> WriteUppaalModelResponse
        try:
            self.get_main().write_uppaal_automata(req.test, tuple(req.inputTypes), req.model)
            return WriteUppaalModelResponse(True)
        except Exception as e:
            rospy.logerr(e)
            traceback.print_exc()
            return WriteUppaalModelResponse(False)


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
        return attributes_by_explore, attributes

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
                attributes_by_explore, attributes = self.get_attributes_from_input_config(input_config, line)
                add_to_list_dict(dicts_by_input, input_id,
                                 {'success': line['success']['value'], 'attributes': attributes_by_explore})
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


class Clusterer:
    def __init__(self, data, dicts_by_topic, reduction):
        self.data = data
        self.dicts_by_topic = dicts_by_topic
        self.cluster = MiniBatchKMeans
        self.score = lambda clusters: silhouette_score(self.data, labels=clusters.labels_)
        self.initial_score = float('-inf')
        self.reduction_min = reduction.get('min', 2)
        self.reduction_max = reduction.get('max', len(data) / 2)
        self.better = lambda x, y: x > y

    def get_clusters(self):
        best_evaluation = self.initial_score
        best_clusters = None

        n_clusters_min = int(len(self.data) / self.reduction_max)
        n_clusters_max = int(len(self.data) / self.reduction_min)
        with warnings.catch_warnings():
            warnings.filterwarnings('error', category=ConvergenceWarning)
            warnings.filterwarnings('error', category=RuntimeWarning)
            for n_clusters in range(n_clusters_min, n_clusters_max + 1):
                try:
                    clusters = self.cluster(n_clusters=n_clusters).fit(self.data)
                    evaluation = self.score(clusters)

                    if self.better(evaluation, best_evaluation):
                        best_evaluation = evaluation
                        best_clusters = clusters
                except ConvergenceWarning:
                    break
                except Exception as e:
                    rospy.logwarn(e)

        return best_clusters

    def plot_clusters(self, states_by_clusters):
        sns.set_context('paper')
        sns.set_color_codes()

        clusters = []
        states = []
        for cluster in states_by_clusters:
            for state in states_by_clusters[cluster]:
                states.append(self.data[state])
                clusters.append(cluster)
        clusters = np.array(clusters)
        states = np.array(states)

        palette = sns.color_palette('bright', np.unique(clusters).max() + 1)
        colors = [palette[x] if x >= 0 else (0.0, 0.0, 0.0) for x in clusters]
        plt.scatter(states.T[0], states.T[1], c=colors, alpha=0.25, s=80, linewidths=0)
        plt.title(self.cluster.__name__, fontsize=14)

    def plot_triangle_arrow(self, x, y, d):
        plt.arrow(x, y, d, d)
        plt.arrow(x + d, y + d, -2 * d, 0)
        plt.arrow(x - d, y + d, d, -d, head_width=0.5, head_length=0.5, overhang=0,
                  length_includes_head=True)

    def plot_state_machine(self, state_machine):
        edges, edge_labels, _, centroids_by_state, _ = state_machine
        centroids_by_state = deepcopy(centroids_by_state)
        for state in edges:
            for next_state in edges[state]:
                x1, y1 = flatten(centroids_by_state[state])[0:2]
                x2, y2 = flatten(centroids_by_state[next_state])[0:2]
                dx = x2 - x1
                dy = y2 - y1

                if dx == 0 and dy == 0:
                    self.plot_triangle_arrow(x1, y1, 1.5)
                    plt.text(x1, y1 + 1.8, edge_labels[(state, next_state)], size=6, ha='center',
                             va='center', color='black')
                else:
                    plt.arrow(x1, y1, dx, dy, head_width=0.5, head_length=0.5, overhang=0,
                              length_includes_head=True)
                    plt.text((x1 + x2) / 2, (y1 + y2) / 2, edge_labels[(state, next_state)], size=6, ha='center',
                             va='center', color='black', rotation=45)

    def plot(self, state_machine):
        edges, edge_labels, points_by_state, centroids_by_state, _ = state_machine
        self.plot_clusters(points_by_state)
        self.plot_state_machine(state_machine)
        plt.show()

    def get_edge_adder_and_remover(self, edges, reverse_edges, edge_labels):
        def add_edge(from_node, to_nodes, label):
            to_nodes = filter(lambda node: node != from_node, to_nodes)
            if from_node in edges:
                edges[from_node].update(to_nodes)
            else:
                edges[from_node] = set(to_nodes)

            for to_node in to_nodes:
                reverse_edges[to_node] = from_node
                edge_labels[(from_node, to_node)] = label

        def remove_edge(from_node, to_nodes):
            new_to_nodes = set()
            for node in edges[from_node]:
                if node not in to_nodes:
                    new_to_nodes.add(node)
            edges[from_node] = new_to_nodes

        return add_edge, remove_edge

    def get_initial_cluster(self, initial_state, states_by_clusters):
        initial_cluster = None
        min_dist = float('inf')
        for cluster in states_by_clusters:
            states = list(map(lambda state: list(self.data[state])[:len(initial_state)], states_by_clusters[cluster]))
            dist = 0
            for state in states:
                dist += math.sqrt(
                    sum(map(lambda coords: (float(coords[1]) - float(coords[0])) ** 2, zip(initial_state, state))))
            if dist < min_dist:
                min_dist = dist
                initial_cluster = cluster
        return initial_cluster

    def divide_sync_topic_clusters(self, clusters, edges, edge_labels, reverse_edges, states_by_clusters, remove_edge,
                                   add_edge):
        cluster_counter = count(max(clusters) + 1)
        for topic in list(self.dicts_by_topic.keys())[1:]:
            dicts = self.dicts_by_topic[topic]
            for i, attributes_dict in enumerate(dicts):
                if not attributes_dict['success']:
                    continue
                cluster = clusters[i]
                all_successful = all(map(lambda state: dicts[state]['success'], states_by_clusters[cluster]))

                new_cluster = next(cluster_counter)
                next_clusters = edges[cluster]
                prev_topic = edge_labels[(reverse_edges[cluster], cluster)]

                remove_edge(cluster, next_clusters)
                add_edge(cluster, [new_cluster], topic)
                add_edge(new_cluster, next_clusters, prev_topic)

                if all_successful:
                    states_by_clusters[new_cluster] = states_by_clusters[cluster]
                else:
                    states_by_clusters[cluster].remove(i)
                    states_by_clusters[new_cluster].append(i)

    def clusters_to_state_machine(self, clusters, initial_state, get_labels=lambda clusters: clusters.labels_):
        if not clusters:
            rospy.logerr("Could not cluster")
            return

        edges, reverse_edges, edge_labels = {}, {}, {}
        states_by_clusters = defaultdict(list)
        add_edge, remove_edge = self.get_edge_adder_and_remover(edges, reverse_edges, edge_labels)

        clusters = get_labels(clusters)
        topic = next(iter(self.dicts_by_topic))
        for i, prev_cluster_label in enumerate(clusters[:-1]):
            cluster_label = clusters[i + 1]
            states_by_clusters[prev_cluster_label].append(i)
            add_edge(prev_cluster_label, [cluster_label], topic)
        states_by_clusters[cluster_label].append(i + 1)

        initial_cluster = self.get_initial_cluster(initial_state, states_by_clusters)
        self.divide_sync_topic_clusters(clusters, edges, edge_labels, reverse_edges, states_by_clusters, remove_edge,
                                        add_edge)

        return edges, edge_labels, states_by_clusters, self.get_centroids(states_by_clusters), initial_cluster

    def get_centroids(self, states_by_clusters):
        centroids_by_clusters = {}

        cluster_labels = []
        cluster_data = []
        for cluster in states_by_clusters:
            for state in states_by_clusters[cluster]:
                cluster_data.append(self.data[state])
                cluster_labels.append(cluster)

        topics_data = [list() for _ in self.dicts_by_topic[next(iter(self.dicts_by_topic))]]
        for topic in self.dicts_by_topic:
            for i, dict_by_topic in enumerate(self.dicts_by_topic[topic]):
                topics_data[i].append(dict_by_topic['attributes'])
        centroid_finder = NearestCentroid().fit(cluster_data, cluster_labels)
        for cluster in cluster_labels:
            try:
                logical_centroid = centroid_finder.centroids_[cluster]
                centroid = topics_data[
                    min(map(lambda state: (distance.euclidean(self.data[state], logical_centroid), state),
                            states_by_clusters[cluster]), key=lambda x: x[0])[1]
                ]
                centroids_by_clusters[cluster] = centroid
            except Exception as e:
                rospy.logerr(e)
        return centroids_by_clusters


class UppaalAutomata:
    def __init__(self, state_machine, test_config, input_types, model=None):
        self.edges, self.edge_labels, _, self.centroids_by_state, self.initial_state = state_machine
        if model is not None:
            self.map = json.loads(model.modelConfig)
            self.adapter_config = json.loads(model.adapterConfig)
            self.model = model.uppaalModel
            return

        self.model = None
        self.model_xml = xml.Element('nta')
        self.template_map = None
        self.template_sut = None

        self.scale_x, self.scale_y = self.get_scaling(150)
        self.test_config = test_config
        self.input_types = input_types
        self.encoding = defaultdict(dict)
        self.map = defaultdict(dict)

        self.identifier_format = "i_{identifier}_go"
        self.success_response_format = "o_{identifier}_rs_succ"
        self.failure_response_format = "o_{identifier}_rs_fail"
        self.variable_format = self.identifier_format + "_{variable}"
        self.command_format = "{variable}={coordinate}"

        self.identifier_transformer = lambda form, id: form.replace('{identifier}', ''.join(
            map(lambda x: x[0], id.strip('/').replace('/', '_').split('_'))))
        self.variable_transformer = lambda form, variable: form.replace('{variable}', ''.join(
            map(lambda x: x[0], variable.replace('.', '_').split('_'))))

        self.adapter_config = self.get_adapter_config()
        self.code_centroids()
        self.add_initial_state_to_map()

    def add_initial_state_to_map(self):
        connection = self.edges[self.initial_state][0]
        identifier = self.edge_labels[(self.initial_state, connection)]
        centroids = self.centroids_by_state[self.initial_state]
        self.get_commands(identifier, centroids)

    def get_adapter_config(self):
        config = {'test_adapter': {}}
        name = ''.join(map(lambda id: self.identifier_transformer('{identifier}', id), self.input_types))
        config['test_adapter'][name] = {'spread': {'username': name, 'ip': '127.0.0.1', 'port': '4803'}}
        config['test_adapter'][name]['dtron'] = {'sync': {'input': [], 'output': {'success': [], 'failure': []}}}

        remove_prefix = lambda format, identifier: '_'.join(
            self.identifier_transformer(format, identifier).split('_')[1:])
        for identifier in self.input_types:
            config['test_adapter'][name]['dtron']['sync']['input'].append(
                remove_prefix(self.identifier_format, identifier))
            config['test_adapter'][name]['dtron']['sync']['output']['success'].append(
                remove_prefix(self.success_response_format, identifier))
            config['test_adapter'][name]['dtron']['sync']['output']['failure'].append(
                remove_prefix(self.failure_response_format, identifier))

        return config

    def code_centroids(self):
        code_map = defaultdict(lambda gen=count(0): next(gen))
        centroids_by_state = deepcopy(self.centroids_by_state)
        for state in centroids_by_state:
            for i, (identifier, centroid) in enumerate(zip(self.input_types, centroids_by_state[state])):
                input_config = next(
                    config for config in self.test_config['inputs'] if config['identifier'] == identifier)
                fields = input_config['explore'].get('variables', []) + input_config['explore'].get('constants', [])
                for j, (arg, field) in enumerate(zip(centroid, fields)):
                    if is_numeric(arg):
                        continue
                    field_name = field['field']
                    channel_format = self.identifier_transformer(self.identifier_format, identifier)
                    code = code_map[arg]
                    command_format = '{}={}'.format(self.variable_transformer('{variable}', field_name), code)
                    if not command_format in self.encoding[channel_format]:
                        self.encoding[channel_format][command_format] = str(arg) if isinstance(arg,
                                                                                               unicode) else arg
                    self.centroids_by_state[state][i][j] = code
        self.encoding = dict(self.encoding)

    def get_scaling(self, max_coordinate):
        centroids = self.centroids_by_state.values()
        centroids = list(map(flatten, centroids))

        x_max = max(centroids, key=lambda x: x[0])[0]
        y_max = max(centroids, key=lambda x: x[1])[1]
        x_min = min(centroids, key=lambda x: x[0])[0]
        y_min = min(centroids, key=lambda x: x[1])[1]

        dx = x_max - x_min
        dy = y_max - y_min

        dx = 1 if dx == 0 else dx
        dy = 1 if dy == 0 else dy

        return int(max_coordinate / dx), int(max_coordinate / dy)

    def get_goal_id(self, state):
        return 'goal={}'.format(state)

    def get_response_id(self, state1, state2):
        return 'resp={}-{}'.format(state1, state2)

    def get_declaration(self):
        declaration_format = 'chan {channels}; int {variables};'
        post_channel_format = self.identifier_format
        success_response_channel_format = self.success_response_format
        failure_response_channel_format = self.failure_response_format
        variable_string_format = self.variable_format
        channel_format = '{}, {}, {}'

        channels = ''
        variable_strings = ''
        for topic_id in self.input_types:
            input_config = next(config for config in self.test_config['inputs'] if config['identifier'] == topic_id)

            explore_config = input_config['explore']
            variables = explore_config.get('variables', [])

            post_channel = self.identifier_transformer(post_channel_format, topic_id)
            self.map['post_channels'][post_channel] = {'topic': topic_id}
            success_response_channel = self.identifier_transformer(success_response_channel_format, topic_id)
            self.map['response_channels'][success_response_channel] = {'topic': topic_id}
            failure_response_channel = self.identifier_transformer(failure_response_channel_format, topic_id)
            self.map['response_channels'][failure_response_channel] = {'topic': topic_id}
            channels += ', ' if channels else ''
            channels += channel_format.format(post_channel, success_response_channel, failure_response_channel)
            variable_string_format_identified = self.identifier_transformer(variable_string_format, topic_id)

            for i, variable in enumerate(variables, 1):
                variable_strings += ', ' if variable_strings else ''
                variable_string = self.variable_transformer(variable_string_format_identified, variable['field'])
                self.map['variables'][variable_string] = {'topic': topic_id}
                variable_strings += variable_string

        return declaration_format.format(channels=channels, variables=variable_strings)

    def add_declaration(self):
        declaration = xml.SubElement(self.model_xml, 'declaration')
        declaration.text = self.get_declaration()
        return self

    def add_map_template(self):
        self.template_map = xml.SubElement(self.model_xml, 'template')
        name = xml.SubElement(self.template_map, 'name')
        name.set('x', '5')
        name.set('y', '5')
        name.text = 'robot_map'

        return self

    def add_map_states(self):
        for state1 in self.edges:
            location_goal = xml.SubElement(self.template_map, 'location')
            location_goal.set('id', self.get_goal_id(state1))
            coords1 = list(map(lambda coord: int(coord * 10), flatten(self.centroids_by_state[state1])))
            location_goal.set('x', str(coords1[0] * self.scale_x))
            location_goal.set('y', str(coords1[1] * self.scale_y))

            name = xml.SubElement(location_goal, 'name')
            name.text = 'goal_{}'.format(state1)

            for state2 in self.edges[state1]:
                location_response = xml.SubElement(self.template_map, 'location')
                location_response.set('id', self.get_response_id(state1, state2))
                coords2 = list(map(lambda coord: int(coord * 10), flatten(self.centroids_by_state[state2])))
                location_response.set('x', str(int((coords1[0] + coords2[0]) / 2) * self.scale_x))
                location_response.set('y', str(int((coords1[1] + coords2[1]) / 2) * self.scale_y))

                name = xml.SubElement(location_response, 'name')
                name.text = 'resp_{}_{}'.format(state1, state2)

        return self

    def add_map_init(self):
        init = xml.SubElement(self.template_map, 'init')
        init.set('ref', self.get_goal_id(self.initial_state))

        return self

    def get_commands(self, identifier, states_by_topics):
        input_config = next(config for config in self.test_config['inputs'] if config['identifier'] == identifier)

        variables = input_config['explore'].get('variables', [])
        constants = input_config['explore'].get('constants', [])

        channel_format = self.identifier_format
        channel_format = self.identifier_transformer(channel_format, input_config['identifier'])
        command_format = self.command_format

        commands = ''
        states = dict(zip(self.input_types, states_by_topics))[identifier]
        resolution = input_config.get('cluster', {}).get('resolution', 1)
        for field_type, state_start, fields, constructor, add in [
            ('commands', 0, variables, lambda command, command_info, _: {command: command_info},
             lambda container, command, command_info, _: container.update({command: command_info})),
            (
                    'constants', len(variables), constants,
                    lambda _, command_info, variable: {variable['field']: command_info},
                    lambda container, _, command_info, variable: container.update({variable['field']: command_info}))]:
            for coordinate, variable in zip(states[state_start:], fields):
                variable_field = self.variable_transformer(command_format, variable['field'])
                command = variable_field.format(coordinate=coordinate)
                encoded = self.encoding.get(channel_format, {}).get(command, None) is not None
                command = command if encoded else variable_field.format(coordinate=int(coordinate / resolution))
                command_info = {'variable': variable['field'],
                                'type': input_config['type'],
                                'value': self.encoding.get(channel_format, {}).get(command, coordinate),
                                'identifier': identifier,
                                'feedback': input_config.get('feedback'),
                                'proxy': input_config.get('proxy', ''),
                                'mode': 'topic' if input_config.get('proxy', '') == '' else 'service'}

                if channel_format not in self.map[field_type]:
                    self.map[field_type][channel_format] = constructor(command, command_info, variable)
                else:
                    add(self.map[field_type][channel_format], command, command_info, variable)

                if field_type == 'commands':
                    commands += ',\n' if commands else ''
                    commands += channel_format + '_' + command

        return commands

    def get_command_sync(self, identifier):
        string_format = self.identifier_format + '!'
        command_sync = self.identifier_transformer(string_format, identifier)
        self.map['sync_commands'][command_sync] = {'input': identifier}
        return command_sync

    def get_failure_response_sync(self, identifier):
        string_format = self.failure_response_format + '?'
        response_sync = self.identifier_transformer(string_format, identifier)
        self.map['sync_responses'][response_sync] = {'input': identifier}
        return response_sync

    def get_success_response_sync(self, identifier):
        string_format = self.success_response_format + '?'
        response_sync = self.identifier_transformer(string_format, identifier)
        self.map['sync_responses'][response_sync] = {'input': identifier}
        return response_sync

    def get_response_guard(self, identifier):
        string_format = self.response_format + '_value==1'
        response_guard = self.identifier_transformer(string_format, identifier)
        return response_guard

    def add_transition(self, parent, source_ref, target_ref, *labels):
        transition = xml.SubElement(parent, 'transition')

        source = xml.SubElement(transition, 'source')
        source.set('ref', source_ref)
        target = xml.SubElement(transition, 'target')
        target.set('ref', target_ref)

        for kind, text in labels:
            if text.strip() != "":
                label_sync = xml.SubElement(transition, 'label')
                label_sync.set('kind', kind)
                label_sync.text = text

    def add_map_transitions(self):
        for state1 in self.edges:
            for state2 in self.edges[state1]:
                identifier = self.edge_labels[(state1, state2)]
                self.add_transition(self.template_map, self.get_goal_id(state1),
                                    self.get_response_id(state1, state2),
                                    ('synchronisation', self.get_command_sync(identifier)),
                                    ('assignment', self.get_commands(identifier, self.centroids_by_state[state2])))
                self.add_transition(self.template_map, self.get_response_id(state1, state2),
                                    self.get_goal_id(state2),
                                    ('synchronisation', self.get_success_response_sync(identifier)))
                self.add_transition(self.template_map, self.get_response_id(state1, state2),
                                    self.get_goal_id(state1),
                                    ('synchronisation', self.get_failure_response_sync(identifier)))
        return self

    def add_sut_template(self):
        self.template_sut = xml.SubElement(self.model_xml, 'template')

        name = xml.SubElement(self.template_sut, 'name')
        name.set('x', '5')
        name.set('y', '5')
        name.text = 'sut'

        location = xml.SubElement(self.template_sut, 'location')
        location.set('id', 'id1')
        location.set('x', '0')
        location.set('y', '0')

        return self

    def add_sut_init(self):
        init = xml.SubElement(self.template_sut, 'init')
        init.set('ref', 'id1')

        return self

    def add_sut_transitions(self):
        for identifier in self.input_types:
            transition_moveto = xml.SubElement(self.template_sut, 'transition')

            source = xml.SubElement(transition_moveto, 'source')
            source.set('ref', 'id1')
            target = xml.SubElement(transition_moveto, 'target')
            target.set('ref', 'id1')

            label_sync = xml.SubElement(transition_moveto, 'label')
            label_sync.set('kind', 'synchronisation')
            label_sync.set('x', '10')
            label_sync.set('y', '-93')
            label_sync.text = self.identifier_transformer(self.identifier_format + '?', identifier)

            nail1 = xml.SubElement(transition_moveto, 'nail')
            nail1.set('x', '-8')
            nail1.set('y', '-102')

            nail2 = xml.SubElement(transition_moveto, 'nail')
            nail2.set('x', '127')
            nail2.set('y', '-51')

            for response_format in [self.success_response_format, self.failure_response_format]:
                transition_response = xml.SubElement(self.template_sut, 'transition')

                source = xml.SubElement(transition_response, 'source')
                source.set('ref', 'id1')
                target = xml.SubElement(transition_response, 'target')
                target.set('ref', 'id1')

                label_sync = xml.SubElement(transition_response, 'label')
                label_sync.set('kind', 'synchronisation')
                label_sync.set('x', '-170')
                label_sync.set('y', '68')
                label_sync.text = self.identifier_transformer(response_format + '!', identifier)

            nail1 = xml.SubElement(transition_response, 'nail')
            nail1.set('x', '8')
            nail1.set('y', '119')

            nail2 = xml.SubElement(transition_response, 'nail')
            nail2.set('x', '-119')
            nail2.set('y', '42')

        return self

    def add_system(self):
        system = xml.SubElement(self.model_xml, 'system')
        system.text = "\n\t\tProcess1 = robot_map();\n\t\tProcess2 = sut();\n\t\tsystem Process1, Process2;\n\t"

        return self

    def add_queries(self):
        xml.SubElement(self.model_xml, 'queries')

        return self

    @staticmethod
    def from_state_machine(state_machine, test_config, input_types):
        return UppaalAutomata(state_machine, test_config, input_types) \
            .add_declaration() \
            .add_map_template() \
            .add_map_states() \
            .add_map_init() \
            .add_map_transitions() \
            .add_sut_template() \
            .add_sut_init() \
            .add_sut_transitions() \
            .add_system() \
            .add_queries()

    @staticmethod
    def from_model(model, state_machine):
        return UppaalAutomata(state_machine, None, None, model)

    def __str__(self):
        if self.model is not None:
            return self.model
        return xmldom.parseString(xml.tostring(self.model_xml)).toprettyxml()


class Main:
    def __init__(self):
        self.test_it = None
        self.clusterer_factory = None
        self.uppaal_automata = None

        self.data_by_test_and_input = None
        self.dicts_by_test_and_input = None
        self.test_configs = None

    def set_test_it(self, test_it_factory):
        # type: (type(TestIt)) -> Main
        self.test_it = test_it_factory()
        return self

    def set_clusterer(self, clusterer_factory):
        # type: (type(Clusterer)) -> Main
        self.clusterer_factory = clusterer_factory  # type: type(Clusterer)
        return self

    def set_uppaal_automata(self, uppaal_automata):
        # type: (type(UppaalAutomata)) -> Main
        self.uppaal_automata = uppaal_automata  # type: type(UppaalAutomata)
        return self

    def get_state_machine(self, test_data, dicts_by_topic, test_config, plot=False):
        clusterer = self.clusterer_factory(test_data, dicts_by_topic,
                                           test_config.get('cluster_reduction_factor', {}))
        clusters = clusterer.get_clusters()
        state_machine = clusterer.clusters_to_state_machine(clusters, )
        if plot:
            clusterer.plot(state_machine)
        return state_machine

    def get_test_config(self, input_types, test):
        return next(test_conf for test_conf in self.test_configs[test]['configuration']['inputs'] if
                    test_conf['identifier'] in input_types)

    def get_clusterer(self, test, input_types):
        self.data_by_test_and_input, self.dicts_by_test_and_input = self.test_it.get_np_arrays_by_test_and_input()
        self.test_configs = self.test_it.logger_configs_by_tests

        test_data = self.data_by_test_and_input[test][input_types]
        dicts_by_topic = OrderedDict(
            (input_type, self.dicts_by_test_and_input[test][input_type]) for input_type in
            input_types)
        test_config = self.test_configs[test]['configuration']
        return self.clusterer_factory(test_data, dicts_by_topic,
                                      test_config.get('cluster_reduction_factor', {}))

    def log_to_clusters(self, test, input_types):
        clusterer = self.get_clusterer(test, input_types)
        test_data = self.data_by_test_and_input[test][input_types]
        clusters = clusterer.get_clusters()
        return zip(clusters.labels_, test_data)

    def clusters_to_state_machine(self, clusters, test, input_types):
        clusterer = self.get_clusterer(test, input_types)
        variables = self.get_test_config(input_types, test).get('explore', {}).get('variables', [])
        initial_state = list(map(lambda variable: variable['initial'], variables))

        return clusterer.clusters_to_state_machine(clusters, initial_state, get_labels=lambda clusters: list(
            map(lambda cluster: cluster.cluster, clusters)))

    def convert_from_state_machine_msg_to_state_machine_tuple(self, state_machine):
        # type: (StateMachine) -> tuple
        edges_ = json.loads(state_machine.edges)
        edges = {int(key): list(map(int, edges_[key])) for key in edges_}
        values_ = json.loads(state_machine.values)
        values = {int(key): values_[key] for key in values_}
        labels_ = json.loads(state_machine.labels)
        labels = {eval(key): labels_[key] for key in labels_}
        return edges, labels, None, values, int(state_machine.initialState)

    def uppaal_automata_from_state_machine(self, state_machine, test, input_types):
        self.test_configs = self.test_it.logger_configs_by_tests
        test_config = self.test_configs[test]['configuration']
        state_machine = self.convert_from_state_machine_msg_to_state_machine_tuple(state_machine)
        return self.uppaal_automata.from_state_machine(state_machine, test_config, input_types)

    def get_uppaal_automatas(self, test):
        self.data_by_test_and_input, self.dicts_by_test_and_input = self.test_it.get_np_arrays_by_test_and_input()

        self.test_configs = self.test_it.logger_configs_by_tests

        for test in self.data_by_test_and_input:
            test_data_by_input = self.data_by_test_and_input[test]
            test_config = self.test_configs[test]['configuration']
            for input_types in test_data_by_input:
                test_data = test_data_by_input[input_types]

                dicts_by_input = OrderedDict(
                    (input_type, self.dicts_by_test_and_input[test][input_type]) for input_type in
                    input_types)
                state_machine = self.get_state_machine(test_data, dicts_by_input, test_config, plot=True)

                uppaal_automata = self.uppaal_automata.from_state_machine(state_machine, test_config, input_types)
                self.test_it.write_model(uppaal_automata, test, input_types)

    def write_uppaal_automata(self, test, input_types, model):
        directory = rospy.get_param('/testit/pipeline')['sharedDirectory'].strip('/') + '/' + \
                    rospy.get_param('/testit/pipeline')['resultsDirectory'].strip('/')
        state_machine = self.convert_from_state_machine_msg_to_state_machine_tuple(model.stateMachine)
        automata = UppaalAutomata.from_model(model, state_machine)
        self.test_it.write_model(automata, test, input_types,
                                 directory=directory)


if __name__ == '__main__':
    _ = ServiceProvider()
