#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class ParameterValue(node.Node):
    """ Stores the curent value of the parameter associated to
        the given ID
    """

    def __init__(self, attrs):
        super(ParameterValue, self).__init__("ParameterValue")

        # Attributes
        self.id = attrs.getValue("id")
        self.value = attrs.getValue("value")

    def beacon(self):
        return "ParameterValue"
