#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import xml.sax

import converter.xar_types as xar_types

import converter.node.box_interface as node_box_interface
import converter.node.bitmap_v2 as node_bitmap_v2
import converter.node.parameter as node_parameter
import converter.node.box_content as node_box_content


def generate_tree_from_filename(filename):
    """ Generates the reprenstation of the file designed by filename
        in memory. Build a tree.

        :param filename: name of the file to parse
        :returns: the tree representing the file
    """
    parser = xml.sax.make_parser()
    handler = BoxInterfaceHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))

    root_node = handler.get_root()
    if root_node.format_version != "4":
        return None

    return root_node


class BoxInterfaceHandler(xml.sax.handler.ContentHandler):
    """ ContentHandler to parse the Box interface XML file
    """

    def __init__(self):
        xml.sax.handler.ContentHandler.__init__(self)
        self._nodes = []
        self._root = None

    def startElement(self, name, attrs):
        new_node = None
        if name == 'BoxInterface':
            new_node = node_box_interface.BoxInterface(
                xar_types.attributes(attrs))
        elif name == 'Bitmap':
            new_node = node_bitmap_v2.BitmapV2(xar_types.attributes(attrs))
        elif name == 'Parameter':
            new_node = node_parameter.Parameter(xar_types.attributes(attrs))
        elif name == 'Content':
            new_node = node_box_content.BoxContent(xar_types.attributes(attrs))

        if new_node:
            if not self._root:
                self._root = new_node
            # Attach to parent here
            if self._nodes:
                parent_node = self._nodes.pop()
                parent_node.attach_attribute(name, new_node)
                parent_node.add_child(new_node)
                self._nodes.append(parent_node)

            self._nodes.append(new_node)
        else:
            parent_node = self._nodes.pop()
            parent_node.attach_attribute(name,
                                         xar_types.attributes(attrs))
            self._nodes.append(parent_node)

    def endElement(self, name):
        if (name == 'BoxInterface'
                or name == 'Parameter'
                or name == 'Content'
                or name == 'Bitmap'):
            self._nodes.pop()

    def get_root(self):
        return self._root
