#!/usr/bin/env python

import yaml

import rospy
from testit_learn.srv import LogToCluster, ClusterToStateMachine, StateMachineToUppaal, LogToClusterRequest, \
    LogToClusterResponse, ClusterToStateMachineRequest, StateMachineToUppaalRequest, ClusterToStateMachineResponse, \
    StateMachineToUppaalResponse, WriteUppaalModel, WriteUppaalModelRequest, WriteUppaalModelResponse


class Launcher:
    def __init__(self):
        rospy.init_node('testit_learn_launcher', anonymous=True)

        self.logger_config = None
        self.test_config = None
        self.test_tag = None

        self.read_test_config()
        self.read_config()

    def read_config(self):
        logger_config_path = rospy.get_param('testit/pipeline/sharedDirectory') + self.test_config[
            'loggerConfiguration']
        with open(logger_config_path, 'r') as file:
            self.logger_config = yaml.load(file)

    def read_test_config(self):
        tests = rospy.get_param('testit/tests')
        self.test_tag = rospy.get_param('testit_logger/test')
        for test in tests:
            if test['tag'] == self.test_tag:
                self.test_config = test
                return

    def get_log(self):
        logger_path = rospy.get_param('/testit_logger/log')
        with open(logger_path) as file:
            lines = file.read().strip(' ').strip('\n').split('\n')
        return lines

    def log_to_cluster(self, service, log):
        # type: (rospy.ServiceProxy, list) -> LogToClusterResponse
        rospy.loginfo("Waiting for log to cluster service")
        service.wait_for_service()
        request = LogToClusterRequest()
        request.test = self.test_tag
        request.log = log
        return service(request)

    def cluster_to_state_machine(self, service, cluster):
        # type: (rospy.ServiceProxy, LogToClusterResponse) -> ClusterToStateMachineResponse
        rospy.loginfo("Waiting for cluster to state machine service")
        service.wait_for_service()
        request = ClusterToStateMachineRequest()
        request.test = self.test_tag
        request.data = cluster.data
        return service(request)

    def state_machine_to_uppaal(self, service, state_machine):
        # type: (rospy.ServiceProxy, ClusterToStateMachineResponse) -> StateMachineToUppaalResponse
        rospy.loginfo("Waiting for state machine to uppaal service")
        service.wait_for_service()
        request = StateMachineToUppaalRequest()
        request.test = self.test_tag
        request.stateMachine = state_machine.stateMachine
        return service(request)

    def write(self, service, uppaal):
        # type: (rospy.ServiceProxy, StateMachineToUppaalResponse) -> WriteUppaalModelResponse
        rospy.loginfo("Waiting for write service")
        service.wait_for_service()
        synced_inputs_matrix = self.logger_config['configuration'].get('syncedExploreTopics', [])
        inputs = self.logger_config['configuration']['inputs']
        request = WriteUppaalModelRequest()
        request.test = self.test_tag
        request.inputTypes = list(
            map(lambda synced_inputs: [inputs[i]['identifier'] for i in synced_inputs], synced_inputs_matrix))
        request.model = uppaal.uppaalModel
        return service(request)

    def write_models(self):
        log_to_cluster_service = self.test_config.get('logToClusterService', '/testit/learn/log/cluster')
        cluster_to_state_machine_service = self.test_config.get('clusterToStateMachineService',
                                                                '/testit/learn/cluster/statemachine')
        state_machine_to_uppaal_service = self.test_config.get('stateMachineToUppaalService',
                                                               '/testit/learn/cluster/uppaal')
        write_service = self.test_config.get('writeUppaalService', '/testit/learn/write/uppaal')

        log_to_cluster = rospy.ServiceProxy(log_to_cluster_service, LogToCluster)
        cluster_to_state_machine = rospy.ServiceProxy(cluster_to_state_machine_service, ClusterToStateMachine)
        state_machine_to_uppaal = rospy.ServiceProxy(state_machine_to_uppaal_service, StateMachineToUppaal)
        write = rospy.ServiceProxy(write_service, WriteUppaalModel)

        log = self.get_log()
        cluster = self.log_to_cluster(log_to_cluster, log)
        state_machine = self.cluster_to_state_machine(cluster_to_state_machine, cluster)
        uppaal = self.state_machine_to_uppaal(state_machine_to_uppaal, state_machine)
        writing = self.write(write, uppaal)

        return writing.result


if __name__ == '__main__':
    exit(int(not Launcher().write_models()))
