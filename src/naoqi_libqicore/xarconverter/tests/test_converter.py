## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

"""Automatic testing for qicore behavior's converter"""

import converter.choregraphe_project_importer as crg_importer
import converter.xar_parser as xar_parser


def test_noDataDriftOldFormat(generate_data):
    path_array = generate_data

    root_node1 = xar_parser.generate_tree_from_filename(path_array[0])
    root_node2 = xar_parser.generate_tree_from_filename(path_array[2])

    assert root_node1 == root_node2, root_node1.diff(root_node2)


def test_noDataDriftNewFormat(generate_data):
    path_array = generate_data

    root_node1 = crg_importer.import_behavior(path_array[1])
    root_node2 = crg_importer.import_behavior(path_array[3])

    assert root_node1 == root_node2, root_node1.diff(root_node2)
