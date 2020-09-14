#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class ChoregrapheProject(node.Node):
    """ Stores informations about behavior entry point
    """

    def __init__(self, attrs):
        super(ChoregrapheProject, self).__init__("ChoregrapheProject")

        # Attributes
        self.name = attrs.getValue("name")
        self.format_version = attrs.getValue("format_version")

        # Elements
        self.root_box = None

        # Function map to speed up process
        self._function_map = {
            'BoxInstance': ChoregrapheProject.attach_root_box}

    def beacon(self):
        return "ChoregrapheProject"

    def attach_root_box(self, root_box):
        self.root_box = root_box
