def get_attribute(value, path):
    for attribute in path.split('.'):
        value = value[attribute]
    return value


def add_to_list_dict(dictionary, key, value):
    if key in dictionary:
        dictionary[key].append(value)
    else:
        dictionary[key] = [value]
