import subprocess

print "no way"

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
