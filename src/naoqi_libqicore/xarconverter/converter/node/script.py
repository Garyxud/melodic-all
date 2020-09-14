#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class Script(node.Node):
    """ Stores informations about script in the xar format
    """

    def __init__(self, attrs):
        super(Script, self).__init__("Script")

        # Attributes
        self.language = attrs.getValue('language')

        # Elements
        self.content = ""

        self._function_map = {'content': Script.attach_content}

    def attach_content(self, content):
        self.content = content

    def beacon(self):
        return "script"
