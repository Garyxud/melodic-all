#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds behaviorLayer informations
.. module:: node
"""

import converter.node as node


class BehaviorLayer(node.Node):
    """ Stores informations about BehaviorLayer in the xar format
    """

    def __init__(self, attrs):
        super(BehaviorLayer, self).__init__("BehaviorLayer")

        # Attributes
        self.name = attrs.getValue("name")
        self.mute = attrs.getValue("mute")

        # Elements
        self.behavior_keyframes = []

        self._function_map = {
            'BehaviorKeyframe': BehaviorLayer.attach_keyframe}

    def attach_keyframe(self, keyframe):
        keyframe.id = len(self.behavior_keyframes)
        self.behavior_keyframes.append(keyframe)
