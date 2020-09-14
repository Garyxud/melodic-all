#!/usr/bin/env python

# # Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
# # Use of this source code is governed by a BSD-style license that can be
# # found in the COPYING file.

""" Contains a class that holds box informations
.. module:: node
"""

import converter.node as node


class BoxInstance(node.Node):
    """ Stores informations about flow diagram fld format
    """

    def __init__(self, attrs):
        super(BoxInstance, self).__init__("BoxInstance")

        # Attributes
        self.name = attrs.getValue("name")
        self.id = attrs.getValue("id")
        self.x = attrs.getValue("x")
        self.y = attrs.getValue("y")
        self.path = attrs.getValue("path")

        # Elements
        self.parameter_values = []
        self.plugin_content = None

        self.interface = None

        # Function map to speed up process
        self._function_map = {
            'ParameterValue': BoxInstance.attach_parameter_value,
            'PluginContent': BoxInstance.attach_plugin_content}

    def attach_parameter_value(self, parameter_value):
        self.parameter_values.append(parameter_value)

    def attach_plugin_content(self, plugin_content):
        self.plugin_content = plugin_content

    def get_parameter_value(self, parameter_id):
        for item in self.parameter_values:
            if item.id == parameter_id:
                return item.value

        return ""
