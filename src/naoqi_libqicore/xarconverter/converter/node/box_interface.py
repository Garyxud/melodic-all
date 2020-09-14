#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds box informations
.. module:: node
"""

import converter.node as node
import converter.xar_types as xar_types


class BoxInterface(node.Node):
    """ Stores informations about box in the xar format
    """

    def __init__(self, attrs):
        super(BoxInterface, self).__init__("BoxInterface")

        # Attributes
        self.uuid = attrs.getValue("uuid")
        self.box_version = attrs.getValue("box_version")
        self.name = attrs.getValue("name")
        self.localization = attrs.getValue("localization")
        self.tooltip = attrs.getValue("tooltip")
        self.plugin = attrs.getValue("plugin")
        self.format_version = attrs.getValue("format_version")

        # Elements
        self.bitmaps = []
        self.inputs = []
        self.outputs = []
        self.parameters = []
        self.resources = []
        self.contents = []

        # Function map to speed up process
        self._function_map = {'Bitmap': BoxInterface.attach_bitmap,
                              'Input': BoxInterface.attach_input,
                              'Output': BoxInterface.attach_output,
                              'Parameter': BoxInterface.attach_parameter,
                              'Resource': BoxInterface.attach_resource,
                              'Content': BoxInterface.attach_content}

    def attach_bitmap(self, bitmap):
        self.bitmaps.append(bitmap)

    def attach_input(self, attrs):
        input = xar_types.IO(attrs.getValue('name'),
                             attrs.getValue('signature'),
                             attrs.getValue('nature'),
                             attrs.getValue('stm_value_name'),
                             attrs.getValue('inner'),
                             attrs.getValue('tooltip'),
                             attrs.getValue('id'))
        self.inputs.append(input)

    def attach_output(self, attrs):
        output = xar_types.IO(attrs.getValue('name'),
                              attrs.getValue('signature'),
                              attrs.getValue('nature'),
                              attrs.getValue('stm_value_name'),
                              attrs.getValue('inner'),
                              attrs.getValue('tooltip'),
                              attrs.getValue('id'))
        self.outputs.append(output)

    def attach_parameter(self, parameter):
        self.parameters.append(parameter)

    def attach_resource(self, attrs):
        resource = xar_types.resource(attrs.getValue('name'),
                                      attrs.getValue('lock_type'),
                                      attrs.getValue('timeout'))
        self.resources.append(resource)

    def attach_content(self, content):
        self.contents.append(content)
