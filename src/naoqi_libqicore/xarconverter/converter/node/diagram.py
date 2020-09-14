#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node
import converter.xar_types as xar_types


class Diagram(node.Node):
    """ Stores informations about diagram in the xar format
    """

    def __init__(self, attrs):
        super(Diagram, self).__init__("Diagram")

        self.scale = attrs.getValue('scale')

        self.boxes = []
        self.links = []
        self.name = ""

        self._function_map = {'Box': Diagram.attach_box,
                              'Link': Diagram.attach_link}

    def attach_box(self, box):
        self.boxes.append(box)

    def attach_link(self, attrs):
        self.links.append(xar_types.link(attrs.getValue("inputowner"),
                                         attrs.getValue("indexofinput"),
                                         attrs.getValue("outputowner"),
                                         attrs.getValue("indexofoutput")))
