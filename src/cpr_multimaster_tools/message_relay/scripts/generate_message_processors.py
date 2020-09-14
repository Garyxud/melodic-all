#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Paul Bovbel <pbovbel@clearpath.ai>
# @copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import argparse
import re
import pprint

from Cheetah.Template import Template
from roslib.message import get_message_class, get_service_class
from collections import namedtuple

pp = pprint.PrettyPrinter(indent=1)


# Convert python field notation (poses[].header.frame_id) into C++ container iterations.
def generate_cpp_iteration(field, default_accessor, processor):
    accessor = default_accessor
    prefix = str()
    suffix = str()
    indent_level = 1

    # TODO move these templates out of code and into Cheetah template
    prefix_template = '{0}for(auto {1} : {2}) \n{0}{{\n'
    suffix_template = '{0}}}\n'
    result_template = '{0}->process({1}{2});\n'

    # Iterate over array fields, and unroll into container iterations
    while (field.count('[].') > 0):
        splitfield = field.split('[].', 1)

        container = accessor + splitfield[0]
        accessor = splitfield[0].split('.')[-1].rstrip('s')+get_accessor(indent_level)
        field = ''.join(splitfield[1:])

        prefix = prefix + prefix_template.format(
            format_indent(indent_level), accessor.rstrip(get_accessor(indent_level)), container)
        suffix = suffix_template.format(format_indent(indent_level)) + suffix
        indent_level += 1

    inset = format_indent(indent_level) + result_template.format(processor, accessor, field)

    result = prefix + inset + suffix
    return result


# Provide indentation as required
def format_indent(indent_level):
    return ''.join(['  ' for s in xrange(indent_level)])


# Get C++ style accessor. Pointers only live at the base indent level.
def get_accessor(indent_level):
    if indent_level == 0:
        return '->'
    else:
        return '.'


# Recursively search through the message/service class for fields with a name and/or type
# Match by regex pattern.
def find_fields(msg_srv_class, field_name_pattern=None, field_type_pattern=None):
    fields = []

    if field_name_pattern is None:
        field_name_pattern = ".*"
    if field_type_pattern is None:
        field_type_pattern = ".*"

    name_regex = re.compile(field_name_pattern)
    type_regex = re.compile(field_type_pattern)

    # ROS msg/srv creates python definitions via __slots__
    if hasattr(msg_srv_class, '__slots__'):
        for (field_name, field_type) in zip(msg_srv_class.__slots__, msg_srv_class._slot_types):
            # If this field is a frame ID, add it to the output list
            if name_regex.match(field_name) is not None and type_regex.match(field_type) is not None:
                fields.append(field_name)

            elif not is_primitive_msg(field_type):
                # If this field is another ROS message type, look inside it for more frame IDs
                if is_msg_array(field_type):
                    # If this field is a message array,
                    field_name += "[]"
                    field_type = field_type.rstrip("[]")
                child_fields = find_fields(get_message_class(field_type), field_name_pattern, field_type_pattern)
                for child_field in child_fields:
                    fields.append("{0}.{1}".format(field_name, child_field))

    return fields


def is_primitive_msg(field_type):
    return field_type.find('/') == -1


def is_msg_array(field_type):
    return field_type[-2:] == '[]'


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate message processor headers and sources.')
    parser.add_argument('pkg_name', metavar='package_name', help='Package Name')
    parser.add_argument('--msg-names', metavar='*.msg', nargs='*', help='Message file paths')
    parser.add_argument('--srv-names', metavar='*.srv', nargs='*', help='Service file paths')
    parser.add_argument('--cpp-tmpl', metavar='*.cpp.tmpl', help='Source template file')
    parser.add_argument('--h-tmpl', metavar='*.h.tmpl', help='Header template file')
    parser.add_argument('--cpp-out', metavar='*.cpp', help='Output source file')
    parser.add_argument('--h-out', metavar='*.h', help='Output header file')

    args = parser.parse_args()
    if not args.cpp_out:
        args.cpp_out = args.pkg_name + "_message_processor.cpp"
    if not args.h_out:
        args.h_out = args.pkg_name + "_message_processor.h"

    # Generator configuration
    FIELD_NAME_FILTER = 'field_name_filter'
    FIELD_TYPE_FILTER = 'field_type_filter'
    MESSAGES = 'msgs'
    SERVICES = 'srvs'

    ServiceComponent = namedtuple("ServiceComponent", "name accessor member_class")
    SERVICE_COMPONENTS = [ServiceComponent(name="req", accessor="req.", member_class="_request_class"),
                          ServiceComponent(name="res", accessor="res.", member_class="_response_class")]

    # Define processors to generate. For example:
    # frame_id_processor:
    #   - References C++ class message_relay::FrameIdProcessor
    #   - Captures fields with a name that contains 'frame_id', and are of type 'string'
    # time_processor:
    #   - References C++ class message_relay::TimeProcessor
    #   - Captures fields of type 'time'
    processors = {
        'frame_id_processor': {
            FIELD_NAME_FILTER: '.*frame_id.*',
            FIELD_TYPE_FILTER: 'string',
            },
        'time_processor': {
            FIELD_NAME_FILTER: None,
            FIELD_TYPE_FILTER: 'time',
            },
    }

    for processor_name, processor in processors.iteritems():
        processor[MESSAGES] = {}
        for msg_name in args.msg_names:
            msg_base = msg_name.partition('/')[2]
            msg_class = get_message_class(msg_name)
            fields = find_fields(msg_srv_class=msg_class, field_name_pattern=processor[FIELD_NAME_FILTER],
                                 field_type_pattern=processor[FIELD_TYPE_FILTER])
            field_processors = [generate_cpp_iteration(field, 'msg->', processor_name) for field in fields]
            processor[MESSAGES][msg_base] = field_processors

        # Generate code for processing service request and responses
        processor[SERVICES] = {}
        for srv_name in args.srv_names:
            srv_base = srv_name.partition('/')[2]
            srv_class = get_service_class(srv_name)
            processor[SERVICES][srv_base] = {}

            for component in SERVICE_COMPONENTS:
                if hasattr(srv_class, component.member_class):
                    fields = find_fields(msg_srv_class=getattr(srv_class, component.member_class),
                                         field_name_pattern=processor[FIELD_NAME_FILTER],
                                         field_type_pattern=processor[FIELD_TYPE_FILTER])
                    field_processors = [generate_cpp_iteration(field, component.accessor, processor_name) for
                                        field in fields]
                    processor[SERVICES][srv_base][component.name] = field_processors

    template_namespace = {}
    template_namespace['processors'] = processors
    template_namespace['pkg_name'] = args.pkg_name
    template_namespace[MESSAGES] = [msg_name.partition('/')[2] for msg_name in args.msg_names]
    template_namespace[SERVICES] = [srv_name.partition('/')[2] for srv_name in args.srv_names]

    # Fill .cpp and .h template with generated message processors
    pp.pprint("Generating templates for package " + args.pkg_name)
    with open(args.cpp_tmpl, 'r') as f:
        source_template = Template(f.read(), searchList=[template_namespace])
    with open(args.h_tmpl, 'r') as f:
        header_template = Template(f.read(), searchList=[template_namespace])

    with open(args.cpp_out, 'w') as f:
        f.write(str(source_template))
    with open(args.h_out, 'w') as f:
        f.write(str(header_template))
