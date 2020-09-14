## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

"""Automatic testing for qicore behavior's converter"""

import distutils.dir_util
import os
import py
import pytest

import converter.choregraphe_project_importer as crg_importer
import converter.xar_format_generator as xar_format_generator
import converter.new_format_generator as new_format_generator
import converter.name_map_builder as names_builder
import converter.xar_parser as xar_parser


def pytest_addoption(parser):
    parser.addoption("--xar", action="store", default="",
                     help="specify path of files to use:\n"
                     + "- old format behavior.xar\n")


def _default_args():
    args = []
    currentDir = os.path.dirname(os.path.abspath(__file__))
    args.append(os.path.join(currentDir,
                             "tests",
                             "behavior1",
                             "behavior.xar"))
    return args


@pytest.fixture(scope="module")
def parse_arg(request):
    """ extract args
    """
    str_args = request.config.getoption("--xar")
    if not str_args:
        return _default_args()

    args = []
    raw_args = str_args.split(" ")
    for argument in raw_args:
        args.append(os.path.abspath(argument))
    return args


@pytest.fixture(scope="module")
def mytmpdir(request):
    import tempfile
    temppath = tempfile.mkdtemp()
    print "\nTest working in %s" % str(temppath)
    tempdir = py.path.local(temppath)

    def remove_temp_dir():
        tempdir.remove(rec=1, ignore_errors=True)

    # request.addfinalizer(remove_temp_dir)
    return tempdir


@pytest.fixture(scope="module")
def generate_data(parse_arg, mytmpdir):
    # execute successives conversion and return an array of output path
    result = []

    filename = parse_arg[0]
    behaviorname = os.path.basename(os.path.dirname(filename))
    # input path is the witness
    result.append(filename)

    # first, convert to new format
    root_node1 = xar_parser.generate_tree_from_filename(filename)
    namesBuilder = names_builder.NameMapBuilder()
    namesBuilder.visit(root_node1)
    namesMap = namesBuilder.get_name_map()

    dest_dir = str(mytmpdir.join("test1"))
    distutils.dir_util.mkpath(dest_dir)

    generator = new_format_generator.NewFormatGenerator(namesMap)
    generator.visit(root_node1, dest_dir)
    generator.generate_entry_point(root_node1, behaviorname)
    result.append(os.path.join(dest_dir, "behavior.xar"))

    # second, convert result back to old xar format
    filename = os.path.join(dest_dir, "behavior.xar")
    root_node2 = crg_importer.import_behavior(filename)

    dest_dir2 = str(mytmpdir.join("test2"))
    distutils.dir_util.mkpath(dest_dir2)

    generator = xar_format_generator.XarFormatGenerator(root_node2)
    generator.export_to_xar(dest_dir2)
    result.append(os.path.join(dest_dir2, "behavior.xar"))

    # third, import and convert once again last xar
    filename = os.path.join(dest_dir2, "behavior.xar")
    root_node3 = xar_parser.generate_tree_from_filename(filename)
    namesBuilder = names_builder.NameMapBuilder()
    namesBuilder.visit(root_node3)

    dest_dir3 = str(mytmpdir.join("test3"))
    distutils.dir_util.mkpath(dest_dir3)

    namesMap = namesBuilder.get_name_map()
    generator = new_format_generator.NewFormatGenerator(namesMap)
    generator.visit(root_node3, dest_dir3)
    generator.generate_entry_point(root_node3, behaviorname)
    result.append(os.path.join(dest_dir3, "behavior.xar"))

    return result
