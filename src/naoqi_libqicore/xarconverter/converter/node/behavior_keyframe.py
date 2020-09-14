#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds behaviorKeyFrame informations
.. module:: node
"""

import converter.node as node


class BehaviorKeyframe(node.Node):
    """ Stores informations about BehaviorKeyframe in the xar format
    """

    def __init__(self, attrs):
        super(BehaviorKeyframe, self).__init__("BehaviorKeyframe")

        # Attributes
        self.name = attrs.getValue("name")
        self.index = attrs.getValue("index")
        self.bitmap = attrs.getValue("bitmap")
        self.diagram = None

        self._function_map = {'Diagram': BehaviorKeyframe.attach_diagram}

    def attach_diagram(self, diagram):
        self.diagram = diagram
