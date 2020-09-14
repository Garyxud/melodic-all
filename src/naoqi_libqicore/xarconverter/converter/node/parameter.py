#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node
import converter.xar_types as xar_types


class Parameter(node.Node):
    """ Stores informations about parameter in the xar format
    """

    def __init__(self, attrs):
        super(Parameter, self).__init__("Parameter")

        # Attributes
        self.name = attrs.getValue("name")
        self.inherits_from_parent = attrs.getValue("inherits_from_parent")
        self.value = attrs.getValue("value")
        self.default_value = attrs.getValue("default_value")
        self.min = attrs.getValue("min")
        self.max = attrs.getValue("max")
        self.custom_choice = attrs.getValue("custom_choice")
        self.password = attrs.getValue("password")
        self.tooltip = attrs.getValue("tooltip")
        self.id = attrs.getValue("id")

        # parameter from the new format
        if attrs.getValue("content_type") is None:
            self.type = attrs.getValue("type")
        else:  # parameter from old xar
            self.type = xar_types.resolve_parameter_signature(attrs.getValue("content_type"))

        # Elements
        self.choices = []

        self._function_map = {'Choice': Parameter.attach_choice}

    def attach_choice(self, attrs):
        choice = xar_types.choice(attrs.getValue('value'))
        self.choices.append(choice)

    def beacon(self):
        return "Parameter"
