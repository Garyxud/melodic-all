## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

"""Automatic testing for qicore behavior's converter"""


import os

import pytest

import converter.name_map_builder as name_map_builder
import converter.new_format_generator as new_format_generator
import converter.xar_format_generator as xar_format_generator
import converter.xar_parser as xar_parser
import converter.choregraphe_project_importer as crg_importer


@pytest.fixture
def oldFormatRootNode(parse_args):
    root_node = xar_parser.generate_tree_from_filename(parse_args[0])
    assert root_node
    return root_node


@pytest.fixture
def newFormatRootNode(parse_args):
    root_node = crg_importer.import_behavior(parse_args[1])
    assert root_node
    return root_node


def test_writeNewFormat(oldFormatRootNode, tmpdir):
    """ code to parse behavior.xar and get root
        looking for no throw and root not None
    """
    namesBuilder = name_map_builder.NameMapBuilder()
    namesBuilder.visit(oldFormatRootNode)
    namesMap = namesBuilder.get_name_map()

    generator = new_format_generator.NewFormatGenerator(namesMap)
    generator.visit(oldFormatRootNode, str(tmpdir))
    generator.generate_entry_point(oldFormatRootNode, "test")


def test_writeOldFormat(newFormatRootNode, tmpdir):
    """ code to parse behavior.xar and get root
        looking for no throw and root not None
    """
    generator = xar_format_generator.XarFormatGenerator(newFormatRootNode)
    generator.export_to_xar(str(tmpdir))
