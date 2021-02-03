import re
import threading

import rospy

from ExploreMoveStrategy import ExploreMoveStrategy

from util import lmap, dynamic_import, get_attribute, set_attribute

try:
    from typing import *
except:
    pass


class RobotMover:
    def __init__(self, robot_move_strategy, topics):
        # type: (ExploreMoveStrategy, List[dict]) -> None
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
        if initial_state:
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
                    return
                lock.acquire()
                step()
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
