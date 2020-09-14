#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import os
import codecs

import converter.code_patcher as code_patcher
import converter.file_writer as file_writer
import converter.xar_types as xar_types


class NewFormatGenerator:
    """ Do a traversal of boxes' treeand write the new format
    """

    def __init__(self, boxes):
        self._boxes = boxes
        self._dest_dir = ""

    def generate_entry_point(self, node, name):
        """ Generate the behavior.xar file used as entry point
        """
        filepath = os.path.join(self._dest_dir, "behavior.xar")
        with codecs.open(filepath, encoding='utf-8', mode='w') as fxar:
            file_writer.write_entry_point(fxar, node, name)

    def visit(self, node, dest_dir=None):
        """ Visit a node, and choose the good method to apply
        """
        if not node:
            return
        if dest_dir:
            self._dest_dir = dest_dir
        methname = "_visit_%s" % node.node_type.lower()
        method = getattr(self, methname, self.visit)
        return method(node)

    def _visit_timeline(self, node):
        if node.enable == "1" and len(node.behavior_layers) > 0:
            # bhs file is an XML description of a behavior sequence
            filepath = os.path.join(self._dest_dir, node.node_path + ".bhs")
            with codecs.open(filepath, encoding='utf-8', mode='w') as fbhs:
                file_writer.write_behavior_sequence(fbhs, node)

        """ even if bhs file contains data about BehaviorLayers,
            BehaviorKeyframes, visit sub node tree to respect recursion
        """
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_behaviorlayer(self, node):
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_behaviorkeyframe(self, node):
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_diagram(self, node):
        # diagram files are an XML description of boxes linked together
        filepath = os.path.join(self._dest_dir, node.node_path + ".fld")
        with codecs.open(filepath, encoding='utf-8', mode='w') as fdiag:
            file_writer.write_flow_diagram(fdiag, node)

        """ even if bhs file contain data about BehaviorLayers,
            BehaviorKeyframes, visit sub node tree to respect recursion
        """
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_actuatorlist(self, node):
        # anim file is an XML description of motion layers
        filepath = os.path.join(self._dest_dir, node.node_path + ".anim")
        with codecs.open(filepath, encoding='utf-8', mode='w') as fanim:
            file_writer.write_animation(fanim, node)

        # no need to go deeper in the node tree

    def _visit_box(self, node):
        filepath = os.path.join(self._dest_dir, node.node_path + ".xml")
        with codecs.open(filepath, encoding='utf-8', mode='w') as fxml:
            file_writer.write_box_interface(fxml, node)

        self.visit(node.script)
        self.visit(node.timeline)

    def _visit_script(self, node):
        script_content = code_patcher.patch(node)

        filepath = os.path.join(self._dest_dir, node.node_path)
        if node.language == xar_types.ScriptLanguage.QICHAT:
            with codecs.open(filepath + ".top",
                             encoding='utf-8', mode='w') as ftop:
                ftop.write(script_content)
        elif node.language == xar_types.ScriptLanguage.PYTHON:
            with codecs.open(filepath + ".py",
                             encoding='utf-8', mode='w') as fpy:
                fpy.write(script_content)
        else:
            print("unidentified script "
                  + node.node_path
                  + " : " + node.language)
