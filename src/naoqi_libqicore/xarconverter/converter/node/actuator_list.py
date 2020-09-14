#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class ActuatorList(node.Node):
    """ Stores informations about actuatorList in the xar format
    """

    def __init__(self, attrs):
        super(ActuatorList, self).__init__("ActuatorList")

        # Attributes
        self.model = attrs.getValue('model')

        # Elements
        self.curves = []

        self._function_map = {'ActuatorCurve': ActuatorList.attach_curve}

    def attach_curve(self, curve):
        self.curves.append(curve)

    def beacon(self):
        return "ActuatorList"
