#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class BitmapV2(node.Node):
    """ Stores informations about bitmap in the box interface format
    """

    def __init__(self, attrs):
        super(BitmapV2, self).__init__("BitmapV2")

        # Elements
        self.path = attrs.getValue("path")

    def beacon(self):
        return "Bitmap"
