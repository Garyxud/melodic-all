#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import codecs
import os

import converter.xar_file_writer as xar_file_writer


class XarFormatGenerator:
    """ Do a traversal of files' tree and write the xar format
    """

    def __init__(self, root_node):
        self._root_node = root_node

    def export_to_xar(self, dest_dir):
        destination = os.path.join(dest_dir, "behavior.xar")
        with codecs.open(destination, encoding='utf-8', mode='w') as fdest:
            xar_file_writer.write_xar_file(fdest, self._root_node)
