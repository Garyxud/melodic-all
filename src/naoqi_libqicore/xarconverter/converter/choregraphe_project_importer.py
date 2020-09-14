#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import os
import codecs

import converter.xar_types as xar_types


def import_animation(filename, working_dir):
    import converter.animation_parser as parser
    filepath = os.path.join(working_dir, filename)
    result = parser.generate_tree_from_filename(filepath)
    if not result:
        print "Loading of %s failed!" % filename
    return result


def import_behavior(filename):
    import converter.behavior_parser as parser
    result = parser.generate_tree_from_filename(filename)
    if not result:
        print "Loading of %s failed!" % filename
        return None

    # then import dependency
    rootBox = result.root_box
    working_dir = os.path.dirname(filename)
    rootBox.interface = import_interface(rootBox.path,
                                         working_dir)

    return result


def import_flow_diagram(filename, working_dir):
    import converter.flow_diagram_parser as parser
    filepath = os.path.join(working_dir, filename)
    result = parser.generate_tree_from_filename(filepath)
    if not result:
        print "Loading of %s failed!" % filename
        return None

    # then import dependency
    for instance in result.box_instances:
        instance.interface = import_interface(instance.path,
                                              working_dir)

    return result


def import_interface(filename, working_dir):
    import converter.box_interface_parser as parser
    filepath = os.path.join(working_dir, filename)
    result = parser.generate_tree_from_filename(filepath)
    if not result:
        print "Loading of %s failed!" % filename
        return None

    # then import dependency
    for content in result.contents:
        if content.content_type == xar_types.ContentType.ANIMATION:
            content.impl = import_animation(content.path,
                                            working_dir)
        elif content.content_type == xar_types.ContentType.FLOW_DIAGRAM:
            content.impl = import_flow_diagram(content.path,
                                               working_dir)
        elif content.content_type == xar_types.ContentType.BEHAVIOR_SEQUENCE:
            content.impl = import_sequence(content.path,
                                           working_dir)
        elif content.content_type == xar_types.ContentType.PYTHON_SCRIPT:
            content.impl = import_script_content(content.path,
                                                 working_dir)
        elif content.content_type == xar_types.ContentType.QICHAT_SCRIPT:
            content.impl = import_script_content(content.path,
                                                 working_dir)
        else:
            raise Exception("Unknown box implementation type: "
                            + str(content.content_type))

    return result


def import_sequence(filename, working_dir):
    import converter.behavior_sequence_parser as parser
    filepath = os.path.join(working_dir, filename)
    result = parser.generate_tree_from_filename(filepath)
    if result is None:
        print "Loading of %s failed!" % filename
        return None

    for layer in result.behavior_layers:
        for keyframe in layer.behavior_keyframes:
            keyframe.diagram = import_flow_diagram(keyframe.path,
                                                   working_dir)

    return result


def import_script_content(filename, working_dir):
    result = ""
    filepath = os.path.join(working_dir, filename)
    with codecs.open(filepath, encoding='utf-8', mode='r') as script_file:
        for line in script_file:
            result += line

    result = result.lstrip(' \r\t\n')
    if result:
        result = result.rstrip(' \r\t\n')

    return result
