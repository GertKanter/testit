#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018 Gert Kanter.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Gert Kanter

import sys
import rosparam
import yaml
import json

def parse_yaml(filename):
    try:
        with open(filename, 'r') as config:
            data = yaml.load(config)
            return data
    except Exception as e:
        import traceback
        traceback.print_exc()
        sys.exit(-1)

def write_yaml_to_file(data, filename):
    try:
        with open(filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
    except Exception as e:
        import traceback
        traceback.print_exc()
        return False
    return True

def dump_yaml(data):
    try:
        return yaml.dump(data, default_flow_style=False)
    except Exception as e:
        import traceback
        traceback.print_exc()
    return None

def load_config_to_rosparam(config):
    rosparam.upload_params('testit', config)

def parse_json_stream_file(filename):
    try:
        with open(filename, 'r') as data:
            return_value = []
            for line in data:
                return_value.append(json.loads(line))
            return return_value
    except Exception as e:
        import traceback
        traceback.print_exc()
        sys.exit(-1)

def append_to_json_file(data, filename, mode="a+"):
    try:
        with open(filename, mode) as outfile:
            outfile.write(json.dumps(data) + "\n")#json.dump(data, outfile, default_flow_style=False)
    except Exception as e:
        import traceback
        traceback.print_exc()
        return False
    return True
