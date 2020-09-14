#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Entry point of the converter
.. module:: converter
"""

import sys
import os
import distutils.dir_util

import converter.xar_parser as xar_parser
import converter.new_format_generator as nfgenerator
import converter.name_map_builder as nmbuilder


def main():
    """ Entry point of the xar converter
    """
    param = []
    if len(sys.argv) not in range(2, 4):
        sys.stderr.write("Incorrect number of arguments" + os.linesep)
        sys.exit(1)
    param.append(sys.argv[1])
    if (len(sys.argv) == 3):
        param.append(sys.argv[2])
    else:
        param.append("objects")

    abspath = os.path.abspath(param[0])
    dest_dir = os.path.abspath(param[1])

    root = xar_parser.generate_tree_from_filename(abspath)
    if not root:
        sys.stderr.write("Incorrect format, file must be in xar_version 3"
                         + os.linesep)
        sys.exit(6)

    nmb = nmbuilder.NameMapBuilder()
    nmb.visit(root)

    name_map = nmb.get_name_map()

    nfg = nfgenerator.NewFormatGenerator(name_map)

    distutils.dir_util.mkpath(dest_dir)
    nfg.visit(root, dest_dir)
    nfg.generate_entry_point(root, os.path.dirname(param[0]))

if __name__ == "__main__":
    main()
