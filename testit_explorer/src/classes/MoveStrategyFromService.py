from __future__ import print_function

import json

import rospy
from testit_explorer.msg import MoveStrategyInit, Actions, Action as ActionMsg
from testit_explorer.srv import MoveStrategy as MoveStrategySrv, MoveStrategyRequest, MoveStrategyResponse

from util import flatten

try:
    from typing import *
except:
    pass


class MoveStrategyFromService:
    def __init__(self, **kwargs):
        self.service_path = kwargs['service']
        self.state_machine = kwargs['state_machine']
        self.init_publisher = None
        if kwargs['init_topic'] != '':
            self.init_publisher = rospy.Publisher(kwargs['init_topic'], MoveStrategyInit, queue_size=1)
        self.service = rospy.ServiceProxy(self.service_path, MoveStrategySrv)
        rospy.sleep(1)

        self.previous_states = None
        self.actions = None
        self.topics = None
        self.feedback = None

    def set_initial_state(self, initial_state):
        if self.init_publisher is None:
            return

        msg = MoveStrategyInit()
        msg.stateMachine = ""
        if self.state_machine is not None:
            msg.stateMachine = json.dumps(self.state_machine)

        msg.topics = list(self.topics)
        msg.initialState = list(initial_state)
        msg.previousStates = flatten(self.previous_states)

        actions = Actions()
        actions.actions = []
        for act in self.actions:
            action = ActionMsg(act.step, act.index)
            actions.actions.append(action)

        self.init_publisher.publish(msg)

    def set_previous_states(self, states):
        self.previous_states = states

    def give_feedback(self, successes):
        self.feedback = successes

    def add(self, actions, topic):
        self.actions.append(actions)
        self.topics.append(topic)

    def get_next_states(self):
        self.service.wait_for_service()
        request = MoveStrategyRequest()
        request.feedback = list(self.feedback)
        response = self.service(request)  # type: MoveStrategyResponse
        return response.nextStates
