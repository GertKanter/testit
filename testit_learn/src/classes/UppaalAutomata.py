import json
import xml.etree.cElementTree as xml
from collections import defaultdict
from copy import deepcopy
from itertools import count

import xml.dom.minidom as xmldom

from util import is_numeric, flatten


class UppaalAutomata:
    def __init__(self, state_machine, test_config, input_types, model=None):
        self.edges, self.edge_labels, _, self.centroids_by_state, self.timestamps_by_state, self.initial_state = state_machine
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
        self.is_timed = self.is_timed_enabled()
        self.code_centroids()
        self.convert_timestamps()
        self.add_initial_state_to_map()

    def add_initial_state_to_map(self):
        connection = self.edges[self.initial_state][0]
        identifier = self.edge_labels[(self.initial_state, connection)]
        centroids = self.centroids_by_state[self.initial_state]
        self.get_commands(identifier, centroids)

    def convert_timestamps(self):
        time_factor = self.test_config.get('modelTimeUnitInMicrosec', 1000000) / 1000000
        for state in self.timestamps_by_state:
            timestamps = self.timestamps_by_state[state]
            self.timestamps_by_state[state] = list(map(lambda timestamp: timestamp * time_factor, timestamps))

    def get_topic_model(self, identifier):
        return next(
            iter(filter(lambda input_config: input_config['identifier'] == identifier, self.test_config['inputs']))) \
            .get('model', {'timed': True, 'timeBuffer': 0})

    def is_topic_timed(self, identifier):
        return self.get_topic_model(identifier)['timed']

    def is_timed_enabled(self):
        return any(input_config.get('model', {}).get('timed', True) for input_config in self.test_config['inputs'])

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

        declaration = declaration_format.format(channels=channels, variables=variable_strings)
        if self.is_timed:
            declaration += ' clock time;'
        return declaration

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
                                    *self.get_success_response_sync_labels(identifier, state1, state2))
                self.add_transition(self.template_map, self.get_response_id(state1, state2),
                                    self.get_goal_id(state1),
                                    ('synchronisation', self.get_failure_response_sync(identifier)))
        return self

    def get_success_response_sync_labels(self, identifier, state1, state2):
        labels = [('synchronisation', self.get_success_response_sync(identifier))]
        if self.is_topic_timed(identifier):
            labels.append(('guard', self.get_time_guard(identifier, state1, state2)))
        return labels

    def get_time_guard(self, identifier, state1, state2):
        time_before = max(self.timestamps_by_state[state1])
        time_after = min(self.timestamps_by_state[state2])
        dt = abs(time_after - time_before)
        return str(int(round(dt + self.get_topic_model(identifier)['timeBuffer']))) + ' >= time'

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

            reset_time_transitions = {self.success_response_format}

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

                if response_format in reset_time_transitions and self.is_topic_timed(identifier):
                    label_assignment = xml.SubElement(transition_response, 'label')
                    label_assignment.set('kind', 'assignment')
                    label_assignment.text = 'time = 0'

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
