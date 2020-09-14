#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class RootBox(node.Node):
    """ Stores informations about behavior entry point
    """

    def __init__(self, attrs):
        super(RootBox, self).__init__("RootBox")

        # Attributes
        self.path = attrs.getValue("path")

        self.interface = None

    def beacon(self):
        return "RootBox"
