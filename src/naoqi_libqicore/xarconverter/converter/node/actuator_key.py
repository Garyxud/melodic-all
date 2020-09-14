#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node
import converter.xar_types as xar_types


class ActuatorKey(node.Node):
    """ Stores informations about ActuatorKey in the xar format
    """

    def __init__(self, attrs):
        super(ActuatorKey, self).__init__("ActuatorKey")

        # Attributes
        self.frame = attrs.getValue('frame')
        self.value = attrs.getValue('value')
        self.smooth = attrs.getValue('smooth')
        self.symmetrical = attrs.getValue('symmetrical')

        # Elements
        self.tangents = []

        self._function_map = {'Tangent': ActuatorKey.attach_tangent}

    def attach_tangent(self, attrs):
        self.tangents.append(xar_types.tangent(attrs.getValue("side"),
                                               attrs.getValue("interpType"),
                                               attrs.getValue("abscissaParam"),
                                               attrs.getValue("ordinateParam")))

    def beacon(self):
        return "Key"
