#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds timeline informations
.. module:: node
"""

import converter.node as node


class Timeline(node.Node):
    """ Stores informations about Timeline in the xar format
    """

    def __init__(self, attrs):
        super(Timeline, self).__init__("Timeline")

        # Attributes
        self.fps = attrs.getValue("fps")
        self.resources_acquisition = attrs.getValue("resources_acquisition")
        self.size = attrs.getValue("size")
        self.enable = attrs.getValue("enable")
        self.start_frame = attrs.getValue("start_frame")
        self.end_frame = attrs.getValue("end_frame")
        self.scale = attrs.getValue("scale")

        # Elements
        self.behavior_layers = []
        self.actuator_list = None

        self._function_map = {'ActuatorList': Timeline.attach_actuator_list,
                              'BehaviorLayer': Timeline.attach_behavior_layer}

    def __eq__(self, other):
        if not other:
            return False
        if not isinstance(other, Timeline):
            return False

        rdict = self.__dict__
        ldict = other.__dict__
        for key in rdict.keys():
            if (key == "parent_node" or key == "children_node"
                    or key == "_function_map" or key == "behavior_layers"):
                continue
            if rdict[key] != ldict[key]:
                return False

        # for diagrams, we don't need to test layer and keyframe
        if self.enable == "0":
            rfld = self.behavior_layers[0].behavior_keyframes[0].diagram
            lfld = other.behavior_layers[0].behavior_keyframes[0].diagram
            if rfld != lfld:
                return False
        else:
            if len(self.behavior_layers) != len(other.behavior_layers):
                return False
            for i in range(len(self.behavior_layers)):
                if (len(self.behavior_layers[i].behavior_keyframes) !=
                        len(other.behavior_layers[i].behavior_keyframes)):
                    return False
                for j in range(len(self.behavior_layers[i].behavior_keyframes)):
                    if (self.behavior_layers[i].behavior_keyframes[j] !=
                            other.behavior_layers[i].behavior_keyframes[j]):
                        return False
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def attach_behavior_layer(self, behavior_layer):
        behavior_layer.id = len(self.behavior_layers)
        self.behavior_layers.append(behavior_layer)

    def attach_actuator_list(self, actuator_list):
        self.actuator_list = actuator_list
