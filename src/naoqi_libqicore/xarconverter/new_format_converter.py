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

import converter.choregraphe_project_importer as crg_importer
import converter.xar_format_generator as xar_format_generator


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

    root = crg_importer.import_behavior(abspath)

    if not root:
        sys.stderr.write("Incorrect format, file must be in format_version 4"
                         + os.linesep)
        sys.exit(6)

    xar_gen = xar_format_generator.XarFormatGenerator(root)

    distutils.dir_util.mkpath(dest_dir)
    xar_gen.export_to_xar(dest_dir)

if __name__ == "__main__":
    main()
