#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class ActuatorCurve(node.Node):
    """ Stores informations about actuatorCurves in the xar format
    """

    def __init__(self, attrs):
        super(ActuatorCurve, self).__init__("ActuatorCurve")

        # Attributes
        self.name = attrs.getValue('name')
        self.actuator = attrs.getValue('actuator')
        self.recordable = attrs.getValue('recordable')
        self.mute = attrs.getValue('mute')
        self.unit = attrs.getValue('unit')
        self.alwaysVisible = attrs.getValue('alwaysvisible')

        # Elements
        self.keys = []

        self._function_map = {'Key': ActuatorCurve.attach_key}

    def attach_key(self, key):
        self.keys.append(key)

    def beacon(self):
        return "ActuatorCurve"
