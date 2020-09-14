#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds diagram informations
.. module:: node
"""

import converter.node as node


class Bitmap(node.Node):
    """ Stores informations about bitmap in the xar format
    """

    def __init__(self, path):
        super(Bitmap, self).__init__("Bitmap")

        # Elements
        self.path = path

    def beacon(self):
        return "bitmap"
