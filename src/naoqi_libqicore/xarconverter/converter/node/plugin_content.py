#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class PluginContent(node.Node):
    """ Stores informations about pluginContent in the xar format
    """

    def __init__(self, attrs):
        super(PluginContent, self).__init__("PluginContent")

        self.subnodes = []

    def add_subnode(self, subnode):
        self.subnodes.append(subnode)

    def beacon(self):
        return "PluginContent"


class PluginSubNode(node.Node):
    """ Stores informations about sub-nodes in plugin content
    """

    def __init__(self, name, attrs):
        super(PluginSubNode, self).__init__(name)

        self.attributes = {}
        self.subnodes = []
        self.content = None

    def add_attrs(self, attributes):
        attrNames = attributes.getNames()
        for attrName in attrNames:
            self.attributes[attrName] = attributes.getValue(attrName)

    def add_subnode(self, subnode):
        self.subnodes.append(subnode)

    def add_content(self, content):
        self.content = content

    def beacon(self):
        return self.node_type
