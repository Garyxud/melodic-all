#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import os
import uuid
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


def write_box_interface(f, node):
    """ Write meta informations about a box

        :param f: open file to write informations
        :param node: the box
    """
    if not node.tooltip:
        node.tooltip = ""

    # XML header
    f.write(u"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" + os.linesep)

    # Root tag and attributes
    f.write((u"<BoxInterface uuid=\"{}\" box_version=\"{}\" name=\"{}\"")
            .format(uuid.uuid4(),
                    "1.0.0.0",
                    node.name))
    if node.localization:
        f.write((u" localization=\"{}\"")
                .format(node.localization))

    f.write((u" tooltip=\"{}\"")
            .format(saxutils.escape(node.tooltip, entities=ENTITIES)))
    if node.plugin:
        f.write((u" plugin=\"{}\"").format(node.plugin))
    f.write((u"  format_version=\"{}\" >{}").format(u"4", os.linesep))

    # bitmap elements
    for bitmap in node.bitmaps:
        f.write((u"    <Bitmap path=\"{}\" />{}")
                .format(bitmap.path,
                        os.linesep))

    for inp in node.inputs:
        if not inp.tooltip:
            inp.tooltip = ""
        if inp.stm_value_name:
            f.write((u"    <Input name=\"{}\" signature=\"{}\""
                     + u" nature=\"{}\" stm_value_name=\"{}\" inner=\"{}\""
                     + u" tooltip=\"{}\" id=\"{}\" />{}")
                    .format(inp.name,
                            saxutils.escape(inp.signature, entities=ENTITIES),
                            inp.nature,
                            inp.stm_value_name,
                            inp.inner,
                            saxutils.escape(inp.tooltip, entities=ENTITIES),
                            inp.id,
                            os.linesep))
        else:
            f.write((u"    <Input name=\"{}\" signature=\"{}\""
                    + u" nature=\"{}\" inner=\"{}\" tooltip=\"{}\""
                    + u" id=\"{}\" />{}")
                    .format(inp.name,
                            saxutils.escape(inp.signature, entities=ENTITIES),
                            inp.nature,
                            inp.inner,
                            saxutils.escape(inp.tooltip, entities=ENTITIES),
                            inp.id,
                            os.linesep))

    for output in node.outputs:
        if not output.tooltip:
            output.tooltip = ""
        f.write((u"    <Output name=\"{}\" signature=\"{}\""
                 + u" nature=\"{}\" inner=\"{}\" tooltip=\"{}\""
                 + u" id=\"{}\" />{}")
                .format(output.name,
                        saxutils.escape(output.signature, entities=ENTITIES),
                        output.nature,
                        output.inner,
                        saxutils.escape(output.tooltip, entities=ENTITIES),
                        output.id,
                        os.linesep))

    for parameter in node.parameters:
        if not parameter.tooltip:
            parameter.tooltip = ""
        f.write((u"    <Parameter name=\"{}\" inherits_from_parent=\"{}\""
                 + u" type=\"{}\" default_value=\"{}\"")
                .format(parameter.name,
                        parameter.inherits_from_parent,
                        saxutils.escape(parameter.type, entities=ENTITIES),
                        parameter.default_value))

        if (parameter.type == xar_types.IOSignature.DOUBLE
                or parameter.type == xar_types.IOSignature.INT):
            f.write((u" min=\"{}\" max=\"{}\"")
                    .format(parameter.min,
                            parameter.max))

        if parameter.custom_choice:
            f.write((u" custom_choice=\"{}\"")
                    .format(parameter.custom_choice))

        if parameter.password:
            f.write((u" password=\"{}\"")
                    .format(parameter.password))

        f.write((u" tooltip=\"{}\" id=\"{}\"")
                .format(saxutils.escape(parameter.tooltip,
                                        entities=ENTITIES),
                        parameter.id))
        if parameter.choices:
            f.write(u">" + os.linesep)
            for choice in parameter.choices:
                f.write((u"        <Choice value=\"{}\" />{}")
                        .format(saxutils.escape(choice.value,
                                                entities=ENTITIES),
                                os.linesep))
            f.write(u"    </Parameter>" + os.linesep)
        else:
            f.write(u" />" + os.linesep)

    for resource in node.resources:
        f.write((u"    <Resource name=\"{}\" lock_type=\"{}\""
                + u" timeout=\"{}\" />{}")
                .format(resource.name,
                        resource.lock_type,
                        resource.timeout,
                        os.linesep))

    # and the link to box implementation
    f.write(u"    <Contents>" + os.linesep)

    # does this box embed a script ?
    if node.script:
        if node.script.language == xar_types.ScriptLanguage.PYTHON:
            f.write((u"        <Content type=\"{}\" path=\"{}\""
                    + u" checksum=\"\" />{}")
                    .format(xar_types.ContentType.PYTHON_SCRIPT,
                            node.script.node_path + ".py",
                            os.linesep))
        else:
            f.write((u"        <Content type=\"{}\" path=\"{}\""
                    + u" checksum=\"\" />{}")
                    .format(xar_types.ContentType.QICHAT_SCRIPT,
                            node.script.node_path + ".top",
                            os.linesep))

    # does this box embed a flow diagram ?
    if node.timeline and node.timeline.enable == "0":
        f.write((u"        <Content type=\"{}\" path=\"{}\""
                + u" checksum=\"\" />{}")
                .format(xar_types.ContentType.FLOW_DIAGRAM,
                        node.timeline.node_path + ".fld",
                        os.linesep))

    # does this box embed a behaviorSequence
    if (node.timeline
            and node.timeline.enable == "1"
            and node.timeline.behavior_layers):
        f.write((u"        <Content type=\"{}\" path=\"{}\""
                + u" checksum=\"\" />{}")
                .format(xar_types.ContentType.BEHAVIOR_SEQUENCE,
                        node.timeline.node_path + ".bhs",
                        os.linesep))

    # does this box embed an animation
    if node.timeline and node.timeline.actuator_list:
        f.write((u"        <Content type=\"{}\" path=\"{}\""
                + u" checksum=\"\" />{}")
                .format(xar_types.ContentType.ANIMATION,
                        node.timeline.actuator_list.node_path + ".anim",
                        os.linesep))

    f.write(u"    </Contents>" + os.linesep)

    f.write(u"</BoxInterface>" + os.linesep)


def write_behavior_sequence(f, timeline):
    """ Write the behavior sequence file

        :param f: open file to write information
        :param timeline: the timeline object containing the sequence
    """

    # XML Header
    f.write(u"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" + os.linesep)

    # BehaviorSequence root tag
    f.write((u"<BehaviorSequence fps=\"{}\" "
            + u"start_frame=\"{}\" end_frame=\"{}\" size=\"{}\"")
            .format(timeline.fps,
                    timeline.start_frame,
                    timeline.end_frame,
                    timeline.size))

    if timeline.resources_acquisition:
        f.write((u" resources_acquisition=\"{}\"")
                .format(timeline.resources_acquisition))

    f.write((u" format_version=\"{}\" >{}")
            .format(u"4",
                    os.linesep))

    for layer in timeline.behavior_layers:
        _write_behavior_layer(f, layer, "    ")

    f.write(u"</BehaviorSequence>" + os.linesep)


def _write_behavior_layer(f, layer, indent):
    f.write((u"{}<BehaviorLayer name=\"{}\"")
            .format(indent,
                    layer.name))
    if layer.mute:
        f.write((u" mute=\"{}\"")
                .format(layer.mute))
    f.write((u" >{}").format(os.linesep))

    for keyframe in layer.behavior_keyframes:
        _write_behavior_keyframe(f, keyframe, indent + "    ")

    f.write((u"{}</BehaviorLayer>{}").format(indent,
                                             os.linesep))


def _write_behavior_keyframe(f, keyframe, indent):
    f.write((u"{}<BehaviorKeyframe name=\"{}\" index=\"{}\"")
            .format(indent,
                    keyframe.name,
                    keyframe.index))

    if keyframe.bitmap:
        f.write((u" bitmap=\"{}\"")
                .format(keyframe.bitmap))

    f.write((u" path=\"{}\" />{}")
            .format(keyframe.node_path + ".fld",
                    os.linesep))


def write_flow_diagram(f, flow_diagram):
    """ Write flow diagram description

        :param f: open file to write information
        :param flow_diagram: diagram node to write
    """
    # XML header
    f.write(u"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" + os.linesep)

    # Root tag
    f.write(u"<FlowDiagram")

    if flow_diagram.scale:
        f.write((u" scale=\"{}\"")
                .format(flow_diagram.scale))

    f.write((u" format_version=\"{}\" >{}")
            .format(u"4",
                    os.linesep))

    # instances of boxes used
    for box in flow_diagram.boxes:
        _write_box_instance(f, box, "    ")

    # links between thos boxes and/or parent box
    for link in flow_diagram.links:
        _write_link(f, link, "    ")

    f.write(u"</FlowDiagram>" + os.linesep)


def _write_box_instance(f, instance, indent):
    f.write((u"{}<BoxInstance name=\"{}\" id=\"{}\""
            + u" x=\"{}\" y=\"{}\" path=\"{}\"")
            .format(indent,
                    instance.name,
                    instance.id,
                    instance.x_pos,
                    instance.y_pos,
                    instance.node_path + ".xml"))

    # if no content, close the beacon
    if not instance.parameters and not instance.plugin_content:
        f.write(u" />" + os.linesep)
        return

    # else
    f.write(u" >" + os.linesep)

    if instance.parameters:
        for parameter in instance.parameters:
            _write_parameter_value(f, parameter, indent + "    ")

    if instance.plugin_content:
        _write_plugin_content(f, instance.plugin_content, indent + "    ")

    f.write((u"{}</BoxInstance>{}").format(indent, os.linesep))


def _write_parameter_value(f, parameter, indent):
    f.write((u"{}<ParameterValue id=\"{}\" value=\"{}\" />{}")
            .format(indent,
                    parameter.id,
                    parameter.value,
                    os.linesep))


def _write_plugin_content(f, pluginContent, indent):
    """ Write XML part of the <pluginContent> beacon, inside the box
        interface XML

        A recursive write is necessary because the content of the part
        is completely unknown and can contain levels and levels of
        sub-nodes
    """

    f.write((u"{}<{}>{}").format(indent,
                                 pluginContent.beacon(),
                                 os.linesep))
    for subnode in pluginContent.subnodes:
        _write_plugin_subnode(f, subnode, indent + "    ")
    f.write((u"{}</{}>{}").format(indent,
                                  pluginContent.beacon(),
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
        f.write((u">{}</{}>{}").format(subnode.content,
                                       subnode.beacon(),
                                       os.linesep))

    else:
        f.write(u"/>" + os.linesep)


def _write_link(f, link, indent):
    f.write((u"{}<Link inputowner=\"{}\" indexofinput=\"{}\""
            + u" outputowner=\"{}\" indexofoutput=\"{}\" />{}")
            .format(indent,
                    link.emitterID,
                    link.indexofinput,
                    link.receiverID,
                    link.indexofoutput,
                    os.linesep))


def write_animation(f, actuator_list):
    """ Write the animation file

        :param f: open file to write information
        :param timeline: the timeline containing the motion layer
    """
    # XML Header
    f.write(u"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" + os.linesep)

    # root tag
    f.write((u"<Animation fps=\"{}\""
            + u" start_frame=\"{}\" end_frame=\"{}\" size=\"{}\"")
            .format(actuator_list.parent_node.fps,
                    actuator_list.parent_node.start_frame,
                    actuator_list.parent_node.end_frame,
                    actuator_list.parent_node.size))

    if actuator_list.parent_node.resources_acquisition:
        f.write((u" resources_acquisition=\"{}\"")
                .format(actuator_list.parent_node.resources_acquisition))

    f.write((u" format_version=\"{}\" >{}")
            .format(u"4",
                    os.linesep))

    f.write((u"    <ActuatorList model=\"{}\" >{}")
            .format(actuator_list.model,
                    os.linesep))

    for curve in actuator_list.curves:
        _write_actuator_curve(f, curve, "        ")

    f.write(u"    </ActuatorList>" + os.linesep)
    f.write(u"</Animation>" + os.linesep)


def _write_actuator_curve(f, curve, indent):
    f.write((u"{}<ActuatorCurve name=\"{}\" actuator=\"{}\""
            + u" recordable=\"{}\" mute=\"{}\" unit=\"{}\" >{}")
            .format(indent,
                    curve.name,
                    curve.actuator,
                    curve.recordable,
                    curve.mute,
                    curve.unit,
                    os.linesep))

    for key in curve.keys:
        _write_actuator_key(f, key, indent + "    ")

    f.write((u"{}</ActuatorCurve>{}")
            .format(indent,
                    os.linesep))


def _write_actuator_key(f, key, indent):
    f.write((u"{}<Key frame=\"{}\" value=\"{}\""
            + u" smooth=\"{}\" symmetrical=\"{}\"")
            .format(indent,
                    key.frame,
                    key.value,
                    key.smooth,
                    key.symmetrical))

    if key.tangents:
        f.write(u" >" + os.linesep)
        for tangent in key.tangents:
            _write_tangent(f, tangent, indent + "    ")
        f.write((u"{}</Key>{}")
                .format(indent,
                        os.linesep))
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


def write_entry_point(f, node, name):
    """ Write the main file of the behavior

        :param f: open file to write information
        :param node: the root node of the behavior
    """

    # XML Header
    f.write(u"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" + os.linesep)

    # root tag
    f.write((u"<ChoregrapheProject name=\"{}\" format_version=\"{}\" >{}")
            .format(name, u"4", os.linesep))

    _write_box_instance(f, node, '    ')

    f.write(u"</ChoregrapheProject>" + os.linesep)
