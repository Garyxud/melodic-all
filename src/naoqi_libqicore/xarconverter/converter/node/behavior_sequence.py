#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds timeline informations
.. module:: node
"""

import converter.node as node


class BehaviorSequence(node.Node):
    """ Stores informations about BehaviorSequence
    """

    def __init__(self, attrs):
        super(BehaviorSequence, self).__init__("BehaviorSequence")

        # Attributes
        self.fps = attrs.getValue("fps")
        self.size = attrs.getValue("size")
        self.start_frame = attrs.getValue("start_frame")
        self.end_frame = attrs.getValue("end_frame")
        self.format_version = attrs.getValue("format_version")
        self.resources_acquisition = attrs.getValue("resources_acquisition")

        # Elements
        self.behavior_layers = []

        self._function_map = {
            'BehaviorLayer': BehaviorSequence.attach_behavior_layer}

    def attach_behavior_layer(self, behavior_layer):
        behavior_layer.id = len(self.behavior_layers)
        self.behavior_layers.append(behavior_layer)
