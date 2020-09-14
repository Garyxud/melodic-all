#!/usr/bin/env python

# # Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
# # Use of this source code is governed by a BSD-style license that can be
# # found in the COPYING file.

""" Contains all modules needed to convert a xar file
.. module:: node
"""


class Node(object):
    """ Parent class of any object in the xar format
    """

    def __init__(self, type):
        self.id = -1
        self.name = ""
        self.node_type = type
        self.node_path = ""
        self.parent_path = ""

        self.children_node = []
        self.parent_node = None
        self._function_map = None

    def diff(self, other):
        result = ""

        if self != other:
            if not other:
                result += str(self.node_type) + ": value of other is None"
            if self.node_type != other.node_type:
                result += "Types different: " + self.node_type + " and " + other.node_type

            rdict = self.__dict__
            ldict = other.__dict__
            for key in rdict.keys():
                ignored_keys = ["parent_node", "children_node", "_function_map", "uuid"]
                if key in ignored_keys:
                    continue
                elif isinstance(rdict[key], list) and isinstance(ldict[key], list):
                    if len(rdict[key]) != len(ldict[key]):
                        result += "%s length is different\n" % str(key)
                        return result

                    for j in range(len(rdict[key])):
                        if rdict[key][j] != ldict[key][j]:
                            result += ("Values different for attrib \"{}\" | row {}\n").format(
                                    str(key), j)

                            if isinstance(rdict[key][j], Node) and isinstance(ldict[key][j], Node):
                                result += rdict[key][j].diff(ldict[key][j])
                            else:
                                result += " value 1: %s\n" % str(rdict[key][j])
                                result += " value 2: %s\n" % str(ldict[key][j])
                            return result
                else:
                    if rdict[key] != ldict[key]:
                        result += "Values different for attrib \"%s\"\n" % str(key)

                        if isinstance(rdict[key], Node) and isinstance(ldict[key], Node):
                            result += rdict[key].diff(ldict[key])
                        else:
                            if key == "timeline":
                                result += " value 1: %s\n" % str(rdict[key].__dict__)
                                result += " value 2: %s\n" % str(ldict[key].__dict__)
                            else:
                                result += " value 1: %s\n" % str(rdict[key])
                                result += " value 2: %s\n" % str(ldict[key])
                        return result

        return result

    def __eq__(self, other):
        if not other:
            return False

        if not isinstance(other, self.__class__):
            return False

        if self.node_type != other.node_type:
            return False

        rdict = self.__dict__
        ldict = other.__dict__
        for key in rdict.keys():
            ignored_keys = ["parent_node", "children_node", "_function_map", "uuid"]
            if key in ignored_keys:
                continue
            elif isinstance(rdict[key], list) and isinstance(ldict[key], list):
                if len(rdict[key]) != len(ldict[key]):
                    return False

                for j in range(len(rdict[key])):
                    if rdict[key][j] != ldict[key][j]:
                        return False
            else:
                if rdict[key] != ldict[key]:
                    return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def add_child(self, child):
        self.children_node.append(child)
        child.parent_node = self

    def attach_attribute(self, name, attrs):
        if (name in self._function_map.keys()):
            self._function_map[name](self, attrs)

    def beacon(self):
        return "Node"
