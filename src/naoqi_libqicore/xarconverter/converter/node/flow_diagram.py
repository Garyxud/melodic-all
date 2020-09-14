#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds box informations
.. module:: node
"""

import converter.node as node
import converter.xar_types as xar_types


class FlowDiagram(node.Node):
    """ Stores informations about flow diagram fld format
    """

    def __init__(self, attrs):
        super(FlowDiagram, self).__init__("FlowDiagram")

        # Attributes
        self.scale = attrs.getValue("scale")
        self.format_version = attrs.getValue("format_version")

        # Elements
        self.box_instances = []
        self.links = []

        # Function map to speed up process
        self._function_map = {'BoxInstance': FlowDiagram.attach_box_instance,
                              'Link': FlowDiagram.attach_link}

    def attach_box_instance(self, box_instance):
        self.box_instances.append(box_instance)

    def attach_link(self, attrs):
        link = xar_types.link(attrs.getValue('inputowner'),
                              attrs.getValue('indexofinput'),
                              attrs.getValue('outputowner'),
                              attrs.getValue('indexofoutput'))
        self.links.append(link)
