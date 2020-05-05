import importlib
import subprocess


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
