#!/usr/bin/env python

import matplotlib

matplotlib.use('Agg')

import json
import traceback
from collections import OrderedDict

import rospy
from testit_learn.msg import ClusterPoint, StateMachine
from testit_learn.srv import StateMachineToUppaal, StateMachineToUppaalRequest, StateMachineToUppaalResponse, \
    WriteUppaalModel, WriteUppaalModelRequest, WriteUppaalModelResponse, LogToCluster, ClusterToStateMachine, \
    LogToClusterResponse, LogToClusterRequest, ClusterToStateMachineResponse, ClusterToStateMachineRequest

from classes.Clusterer import Clusterer
from classes.TestIt import TestIt
from classes.UppaalAutomata import UppaalAutomata


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

    def get_services(self):
        return Services() \
            .set_test_it(TestIt) \
            .set_clusterer(Clusterer) \
            .set_uppaal_automata(UppaalAutomata)

    def log_to_cluster_service(self, req):
        # type: (LogToClusterRequest) -> LogToClusterResponse
        cluster_points = self.get_services().log_to_clusters(req.test, tuple(req.inputTypes))
        response = LogToClusterResponse()
        for (cluster, data) in cluster_points:
            point = ClusterPoint()
            point.cluster = cluster
            point.point = data
            response.data.append(point)
        return response

    def cluster_to_statemachine_service(self, req):
        # type: (ClusterToStateMachineRequest) -> ClusterToStateMachineResponse
        edges, edge_labels, _, centroids, initial_cluster = self.get_services() \
            .clusters_to_state_machine(req.data, req.test, tuple(req.inputTypes))
        convert = lambda d, value_to: {str(key): value_to(d[key]) for key in d}
        response = ClusterToStateMachineResponse()
        response.stateMachine.edges = json.dumps(convert(edges, lambda value: list(map(str, value))))
        response.stateMachine.labels = json.dumps(convert(edge_labels, str))
        response.stateMachine.values = json.dumps(convert(centroids, list))
        response.stateMachine.initialState = str(initial_cluster)
        return response

    def statemachine_to_uppaal_model_service(self, req):
        # type: (StateMachineToUppaalRequest) -> StateMachineToUppaalResponse
        uppaal_automata = self.get_services().uppaal_automata_from_state_machine(req.stateMachine, req.test,
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
            self.get_services().write_uppaal_automata(req.test, tuple(req.inputTypes), req.model)
            return WriteUppaalModelResponse(True)
        except Exception as e:
            rospy.logerr(e)
            traceback.print_exc()
            return WriteUppaalModelResponse(False)


class Services:
    def __init__(self):
        self.test_it = None
        self.clusterer_factory = None
        self.uppaal_automata = None

        self.data_by_test_and_input = None
        self.dicts_by_test_and_input = None
        self.test_configs = None
        self.test_tag = None
        self.config = None

        self.read_config()

    def set_test_it(self, test_it_factory):
        # type: (type(TestIt)) -> Services
        self.test_it = test_it_factory()
        return self

    def set_clusterer(self, clusterer_factory):
        # type: (type(Clusterer)) -> Services
        self.clusterer_factory = clusterer_factory  # type: type(Clusterer)
        return self

    def set_uppaal_automata(self, uppaal_automata):
        # type: (type(UppaalAutomata)) -> Services
        self.uppaal_automata = uppaal_automata  # type: type(UppaalAutomata)
        return self

    def read_config(self):
        tests = rospy.get_param('testit/tests')
        self.test_tag = rospy.get_param('testit_logger/test')
        for test in tests:
            if test['tag'] == self.test_tag:
                self.config = test
                return

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
                                      test_config.get('clusterReductionFactor', {}))

    def log_to_clusters(self, test, input_types):
        clusterer = self.get_clusterer(test, input_types)
        test_data = self.data_by_test_and_input[test][input_types]
        clusters = clusterer.get_clusters()
        return zip(clusters.labels_, test_data)

    def clusters_to_state_machine(self, clusters, test, input_types):
        clusterer = self.get_clusterer(test, input_types)
        variables = self.get_test_config(input_types, test).get('explore', {}).get('variables', [])
        initial_state = list(map(lambda variable: variable['initial'], variables))

        state_machine = clusterer.clusters_to_state_machine(clusters, initial_state, self.config.get('stateMachine'))
        file_path = rospy.get_param('testit/pipeline/sharedDirectory') + rospy.get_param(
            'testit/pipeline/resultsDirectory') + "/" + self.config['learnBy'] + ''.join(
            map(lambda id: ''.join(map(lambda x: x[0], id.strip('/').replace('/', '_').split('_'))),
                input_types)) + "-statemachine-cluster"
        clusterer.plot(state_machine, file_path, self.config.get('plot', False))
        return state_machine

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

    def write_uppaal_automata(self, test, input_types, model):
        directory = rospy.get_param('/testit/pipeline')['sharedDirectory'].strip('/') + '/' + \
                    rospy.get_param('/testit/pipeline')['resultsDirectory'].strip('/')
        state_machine = self.convert_from_state_machine_msg_to_state_machine_tuple(model.stateMachine)
        automata = UppaalAutomata.from_model(model, state_machine)
        self.test_it.write_model(automata, test, input_types,
                                 directory=directory)


if __name__ == '__main__':
    _ = ServiceProvider()
