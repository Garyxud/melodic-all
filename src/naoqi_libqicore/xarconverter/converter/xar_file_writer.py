#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import os
import xml.sax.saxutils as saxutils

import converter.xar_types as xar_types


ENTITIES = {
    '\x00': "&#x00;", '\x01': "&#x01;",
    '\x02': "&#x02;", '\x03': "&#x03;",
    '\x04': "&#x04;", '\x05': "&#x05;",
    '\x06': "&#x06;", '\x07': "&#x07;",
    '\x08': "&#x08;", '\x09': "&#x09;",
    '\x0a': "&#x0A;", '\x0b': "&#x0B;",
    '\x0c': "&#x0C;", '\x0d': "&#x0D;",
    '\x0e': "&#x0E;", '\x0f': "&#x0F;",
    '\x10': "&#x10;", '\x11': "&#x11;",
    '\x12': "&#x12;", '\x13': "&#x13;",
    '\x14': "&#x14;", '\x15': "&#x15;",
    '\x16': "&#x16;", '\x17': "&#x17;",
    '\x18': "&#x18;", '\x19': "&#x19;",
    '\x1a': "&#x1A;", '\x1b': "&#x1B;",
    '\x1c': "&#x1C;", '\x1d': "&#x1D;",
    '\x1e': "&#x1E;", '\x1f': "&#x1F;",
    '\"': "&quot;", '\'': "&apos;"
}


def write_xar_file(f, project):
    # XML header
    f.write(u"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" + os.linesep)

    # Root tag
    f.write(u"<ChoregrapheProject "
            + u"xmlns=\"http://www.aldebaran-robotics.com/schema/choregraphe/project.xsd\""
            + u" xar_version=\"3\">"
            + os.linesep)

    _write_box_instance(f, project.root_box, "    ")

    f.write(u"</ChoregrapheProject>" + os.linesep)


def _write_box_instance(f, instance, indent):
    # box beacon
    f.write((u"{}<Box name=\"{}\" id=\"{}\"")
            .format(indent,
                    saxutils.escape(instance.name, entities=ENTITIES),
                    instance.id))

    if instance.interface.localization:
        f.write((u" localization=\"{}\"")
                .format(instance.interface.localization))

    f.write((u" tooltip=\"{}\"")
            .format(saxutils.escape(instance.interface.tooltip,
                                    entities=ENTITIES)))

    if instance.interface.plugin:
        f.write((u" plugin=\"{}\"").format(instance.interface.plugin))

    f.write((u" x=\"{}\" y=\"{}\">{}")
            .format(instance.x,
                    instance.y,
                    os.linesep))

    for bitmap in instance.interface.bitmaps:
        _write_bitmap(f, bitmap, indent + '    ')

    _write_box_script(f, instance, indent + '    ')

    if instance.plugin_content and instance.interface.plugin:
        _write_plugin_content(f, instance.plugin_content, indent + '    ')

    for input in instance.interface.inputs:
        _write_input(f, input, indent + '    ')

    for output in instance.interface.outputs:
        _write_output(f, output, indent + '    ')

    for parameter in instance.interface.parameters:
        value = instance.get_parameter_value(parameter.id)
        _write_parameter(f, parameter, value, indent + '    ')

    _write_timeline(f, instance.interface, indent + '    ')

    for resource in instance.interface.resources:
        _write_resource(f, resource, indent + '    ')

    f.write((u"{}</Box>{}").format(indent, os.linesep))


def _write_bitmap(f, bitmap, indent):
    f.write((u"{}<bitmap>{}</bitmap>{}")
            .format(indent,
                    bitmap.path,
                    os.linesep))


def _write_box_script(f, instance, indent):
    script_content = None
    for content in instance.interface.contents:
        if (content.content_type == xar_types.ContentType.PYTHON_SCRIPT
                or content.content_type == xar_types.ContentType.QICHAT_SCRIPT):
            script_content = content

    if script_content:
        if script_content.content_type == xar_types.ContentType.PYTHON_SCRIPT:
            f.write((u"{}<script language=\"{}\">{}")
                    .format(indent,
                            xar_types.ScriptLanguage.PYTHON,
                            os.linesep))
        elif script_content.content_type == xar_types.ContentType.QICHAT_SCRIPT:
            f.write((u"{}<script language=\"{}\">{}")
                    .format(indent,
                            xar_types.ScriptLanguage.QICHAT,
                            os.linesep))
        else:
            raise Exception("Bad script language "
                            + script_content.content_type)

        f.write((u"{}<content>{}"
                + u"{}<![CDATA[{}]]>{}"
                + u"</content>{}"
                + u"{}</script>{}")
                .format(indent + '    ', os.linesep,
                        indent + '        ', script_content.impl, os.linesep,
                        os.linesep,
                        indent, os.linesep))
    else:
        f.write((u"{}<script/>{}").format(indent, os.linesep))


def _write_plugin_content(f, plugin_content, indent):
    """ Write XML part of the <pluginContent> beacon, inside the box
        interface XML

        A recursive write is necessary because the content of the part
        is completely unknown and can contain levels and levels of
        sub-nodes
    """

    f.write((u"{}<pluginContent>{}").format(indent,
                                            os.linesep))
    for subnode in plugin_content.subnodes:
        _write_plugin_subnode(f, subnode, indent + "    ")
    f.write((u"{}</pluginContent>{}").format(indent,
                                             os.linesep))


def _write_plugin_subnode(f, subnode, indent):
    f.write((u"{}<{}").format(indent,
                              subnode.beacon()))
    if subnode.attributes.keys():
        for key in subnode.attributes.keys():
            f.write((u" {}=\"{}\"").format(str(key),
                                           str(subnode.attributes[key])))

    if subnode.subnodes:
        f.write(u">" + os.linesep)
        for subsubnode in subnode.subnodes:
            _write_plugin_subnode(f, subsubnode, indent + "    ")
        f.write((u"{}</{}>{}").format(indent,
                                      subnode.beacon(),
                                      os.linesep))
    elif subnode.content:
        f.write((u">{}</{}>{}").format(saxutils.escape(subnode.content,
                                                       entities=ENTITIES),
                                       subnode.beacon(),
                                       os.linesep))
    else:
        f.write(u"/>" + os.linesep)


def _write_input(f, input, indent):
    f.write((u"{}<Input name=\"{}\" {} nature=\"{}\"")
            .format(indent,
                    input.name,
                    _resolve_io_type_and_size(input),
                    input.nature))

    if input.stm_value_name:
        f.write((u" stm_value_name=\"{}\"")
                .format(input.stm_value_name))

    f.write((u" inner=\"{}\" tooltip=\"{}\" id=\"{}\" />{}")
            .format(input.inner,
                    saxutils.escape(input.tooltip, entities=ENTITIES),
                    input.id,
                    os.linesep))


def _write_output(f, output, indent):
    f.write((u"{}<Output name=\"{}\" {}"
            + u" nature=\"{}\" inner=\"{}\" tooltip=\"{}\" id=\"{}\" />{}")
            .format(indent,
                    output.name,
                    _resolve_io_type_and_size(output),
                    output.nature,
                    output.inner,
                    saxutils.escape(output.tooltip, entities=ENTITIES),
                    output.id,
                    os.linesep))


def _write_parameter(f, parameter, value, indent):
    f.write((u"{}<Parameter name=\"{}\" inherits_from_parent=\"{}\""
            + u" content_type=\"{}\" value=\"{}\" default_value=\"{}\"")
            .format(indent,
                    parameter.name,
                    parameter.inherits_from_parent,
                    _resolve_parameter_type(parameter),
                    value,
                    parameter.default_value))

    if (parameter.type == xar_types.IOSignature.DOUBLE
            or parameter.type == xar_types.IOSignature.INT):
        f.write((u" min=\"{}\" max=\"{}\"")
                .format(parameter.min,
                        parameter.max))

    if parameter.custom_choice:
        f.write((u" custom_choice=\"{}\"").format(parameter.custom_choice))

    if parameter.password == "1":
        f.write((u" password=\"{}\"").format(parameter.password))

    if parameter.choices:
        f.write((u" tooltip=\"{}\" id=\"{}\">{}")
                .format(saxutils.escape(parameter.tooltip, entities=ENTITIES),
                        parameter.id,
                        os.linesep))
        for choice in parameter.choices:
            f.write((u"{}<Choice value=\"{}\" />{}")
                    .format(indent + '    ',
                            saxutils.escape(choice.value, entities=ENTITIES),
                            os.linesep))
        f.write((u"{}</Parameter>{}")
                .format(indent,
                        os.linesep))
    else:
        f.write((u" tooltip=\"{}\" id=\"{}\" />{}")
                .format(saxutils.escape(parameter.tooltip, entities=ENTITIES),
                        parameter.id,
                        os.linesep))


def _resolve_io_type_and_size(io):
    ioType = ""
    ioSize = u"1"

    # allowed because old xar do not manage complete types
    signature = io.signature.strip("()")
    if signature == xar_types.IOSignature.BANG:  # empty
        ioType = xar_types.IOType.BANG
    elif signature == xar_types.IOSignature.BITMAP:
        ioType = xar_types.IOType.BITMAP
    elif signature == xar_types.IOSignature.SOUND:
        ioType = xar_types.IOType.SOUND
    elif signature[0] == xar_types.IOSignature.DYNAMIC:
        ioType = xar_types.IOType.DYNAMIC
    elif signature[0] == xar_types.IOSignature.DOUBLE:
        ioType = xar_types.IOType.NUMBER
        ioSize = str(signature.count(xar_types.IOSignature.DOUBLE))
    elif signature[0] == xar_types.IOSignature.STRING:
        ioType = xar_types.IOType.STRING
        ioSize = str(signature.count(xar_types.IOSignature.STRING))
    else:
        raise Exception("Unknown signature: %s" % io.type)

    return u"type=\"{}\" type_size=\"{}\"".format(ioType, ioSize)


def _resolve_parameter_type(parameter):
    paramType = ""

    if parameter.type == xar_types.IOSignature.BOOL:
        paramType = xar_types.ParameterType.BOOL
    elif parameter.type == xar_types.IOSignature.DOUBLE:
        paramType = xar_types.ParameterType.DOUBLE
    elif parameter.type == xar_types.IOSignature.INT:
        paramType = xar_types.ParameterType.INT
    elif parameter.type == xar_types.IOSignature.RESOURCE:
        paramType = xar_types.ParameterType.RESOURCE
    elif parameter.type == xar_types.IOSignature.STRING:
        paramType = xar_types.ParameterType.STRING
    else:
        raise Exception("Unknown signature: %s" % parameter.type)

    return paramType


def _write_timeline(f, interface, indent):
    # extract interesting contents
    flow_diagram = None
    sequence = None
    animation = None

    for content in interface.contents:
        if content.content_type == xar_types.ContentType.FLOW_DIAGRAM:
            flow_diagram = content.impl
        elif content.content_type == xar_types.ContentType.BEHAVIOR_SEQUENCE:
            sequence = content.impl
        elif content.content_type == xar_types.ContentType.ANIMATION:
            animation = content.impl
        elif content.content_type == xar_types.ContentType.PYTHON_SCRIPT:
            pass
        elif content.content_type == xar_types.ContentType.QICHAT_SCRIPT:
            pass
        else:
            raise Exception("unknown content type "
                            + str(content.content_type))

    if not (flow_diagram or sequence or animation):
        return

    if flow_diagram:
        f.write((u"{}<Timeline enable=\"0\">{}")
                .format(indent,
                        os.linesep))
    elif animation:
        f.write((u"{}<Timeline enable=\"1\" fps=\"{}\" start_frame=\"{}\""
                + u" end_frame=\"{}\" size=\"{}\"")
                .format(indent,
                        animation.fps,
                        animation.start_frame,
                        animation.end_frame,
                        animation.size))
        if animation.resources_acquisition:
            f.write((u" resources_acquisition=\"{}\">{}")
                    .format(animation.resources_acquisition,
                            os.linesep))
        else:
            f.write((u">{}").format(os.linesep))
    else:
        f.write((u"{}<Timeline enable=\"1\" fps=\"{}\" start_frame=\"{}\""
                + u" end_frame=\"{}\" size=\"{}\"")
                .format(indent,
                        sequence.fps,
                        sequence.start_frame,
                        sequence.end_frame,
                        sequence.size))
        if sequence.resources_acquisition:
            f.write((u" resources_acquisition=\"{}\">{}")
                    .format(sequence.resources_acquisition,
                            os.linesep))
        else:
            f.write((u">{}").format(os.linesep))

    if flow_diagram:
        _write_flow_diagram(f, flow_diagram, indent + '    ')

    if sequence:
        _write_behavior_sequence(f, sequence, indent + '    ')

    if animation:
        _write_animation(f, animation, indent + '    ')

    f.write((u"{}</Timeline>{}")
            .format(indent,
                    os.linesep))


def _write_flow_diagram(f, diagram, indent):
    f.write((u"{}<BehaviorLayer name=\"behavior_layer1\">{}"
            + u"{}<BehaviorKeyframe name=\"keyframe1\" index=\"1\">{}")
            .format(indent,
                    os.linesep,
                    indent + '    ',
                    os.linesep))
    _write_diagram(f, diagram, indent + '        ')
    f.write((u"{}</BehaviorKeyframe>{}"
            + u"{}</BehaviorLayer>{}")
            .format(indent + '    ',
                    os.linesep,
                    indent,
                    os.linesep))


def _write_animation(f, animation, indent):
    # there is no animation level in old xar format
    _write_actuator_list(f, animation.actuator_list, indent)


def _write_actuator_list(f, actuator_list, indent):
    f.write((u"{}<ActuatorList model=\"{}\">{}")
            .format(indent,
                    actuator_list.model,
                    os.linesep))

    for curve in actuator_list.curves:
        _write_actuator_curve(f, curve, indent + '    ')

    f.write((u"{}</ActuatorList>{}")
            .format(indent,
                    os.linesep))


def _write_actuator_curve(f, curve, indent):
    f.write((u"{}<ActuatorCurve name=\"{}\" actuator=\"{}\" recordable=\"{}\""
            + u" mute=\"{}\" unit=\"{}\"")
            .format(indent,
                    curve.name,
                    curve.actuator,
                    curve.recordable,
                    curve.mute,
                    curve.unit))

    if curve.keys:
        f.write(u">" + os.linesep)
        for key in curve.keys:
            _write_actuator_key(f, key, indent + '    ')

        f.write((u"{}</ActuatorCurve>{}")
                .format(indent,
                        os.linesep))
    else:
        f.write(u" />" + os.linesep)


def _write_actuator_key(f, key, indent):
    f.write((u"{}<Key frame=\"{}\" value=\"{}\"")
            .format(indent,
                    key.frame,
                    key.value))

    if key.smooth == "1":
        f.write(u" smooth=\"1\"")
    if key.symmetrical == "1":
        f.write(u" symmetrical=\"1\"")

    if key.tangents:
        f.write(u">" + os.linesep)
        for tangent in key.tangents:
            _write_tangent(f, tangent, indent + '    ')
        f.write(indent + u"</Key>" + os.linesep)
    else:
        f.write(u" />" + os.linesep)


def _write_tangent(f, tangent, indent):
    f.write((u"{}<Tangent side=\"{}\" interpType=\"{}\""
            + u" abscissaParam=\"{}\" ordinateParam=\"{}\" />{}")
            .format(indent,
                    tangent.side,
                    tangent.interpType,
                    tangent.abscissaParam,
                    tangent.ordinateParam,
                    os.linesep))


def _write_behavior_sequence(f, sequence, indent):
    for layer in sequence.behavior_layers:
        _write_behavior_layer(f, layer, indent)


def _write_behavior_layer(f, layer, indent):
    f.write((u"{}<BehaviorLayer name=\"{}\"")
            .format(indent,
                    layer.name))
    if layer.mute:
        f.write((u" mute=\"{}\">{}")
                .format(layer.mute,
                        os.linesep))
    else:
        f.write(u">" + os.linesep)

    for keyframe in layer.behavior_keyframes:
        _write_behavior_keyframe(f, keyframe, indent + '    ')

    f.write((u"{}</BehaviorLayer>{}")
            .format(indent,
                    os.linesep))


def _write_behavior_keyframe(f, keyframe, indent):
    f.write((u"{}<BehaviorKeyframe name=\"{}\" index=\"{}\"")
            .format(indent,
                    keyframe.name,
                    keyframe.index))
    if keyframe.bitmap:
        f.write((u" bitmap=\"{}\">{}")
                .format(keyframe.bitmap,
                        os.linesep))
    else:
        f.write(u">" + os.linesep)

    _write_diagram(f, keyframe.diagram, indent + '    ')

    f.write((u"{}</BehaviorKeyframe>{}")
            .format(indent,
                    os.linesep))


def _write_diagram(f, diagram, indent):
    # because a keyframe can embed an empty diagram
    if (not diagram.box_instances) and (not diagram.links):
        f.write(indent + u"<Diagram />" + os.linesep)
        return

    # else
    f.write(indent + u"<Diagram")

    if diagram.scale:
        f.write((u" scale=\"{}\">{}")
                .format(diagram.scale,
                        os.linesep))
    else:
        f.write(u">" + os.linesep)

    for box_instance in diagram.box_instances:
        _write_box_instance(f, box_instance, indent + '    ')

    for link in diagram.links:
        _write_link(f, link, indent + '    ')

    f.write(indent + u"</Diagram>" + os.linesep)


def _write_link(f, link, indent):
    f.write((u"{}<Link inputowner=\"{}\" indexofinput=\"{}\""
            + u" outputowner=\"{}\" indexofoutput=\"{}\" />{}")
            .format(indent,
                    link.emitterID,
                    link.indexofinput,
                    link.receiverID,
                    link.indexofoutput,
                    os.linesep))


def _write_resource(f, resource, indent):
    f.write((u"{}<Resource name=\"{}\" type=\"{}\" timeout=\"{}\" />{}")
            .format(indent,
                    resource.name,
                    resource.lock_type,
                    resource.timeout,
                    os.linesep))
