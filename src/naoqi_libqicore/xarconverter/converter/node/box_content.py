#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class BoxContent(node.Node):
    """ Stores informations about Box implementation, a content
        of the Box Interface
    """

    def __init__(self, attrs):
        super(BoxContent, self).__init__("BoxContent")

        self.content_type = attrs.getValue("type")
        self.path = attrs.getValue("path")
        self.checksum = attrs.getValue("checksum")

        self.impl = None

    def beacon(self):
        return "Content"
