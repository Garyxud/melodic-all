#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class Animation(node.Node):
    """ Stores informations about an animation file
    """

    def __init__(self, attrs):
        super(Animation, self).__init__("Animation")

        # Attributes
        self.fps = attrs.getValue('fps')
        self.start_frame = attrs.getValue('start_frame')
        self.end_frame = attrs.getValue('end_frame')
        self.size = attrs.getValue('size')
        self.format_version = attrs.getValue("format_version")
        self.resources_acquisition = attrs.getValue("resources_acquisition")

        # Elements
        self.actuator_list = None

        self._function_map = {'ActuatorList': Animation.attach_actuator_list}

    def attach_actuator_list(self, actuator_list):
        self.actuator_list = actuator_list

    def beacon(self):
        return "Animation"
