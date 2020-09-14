## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

"""Automatic testing for qicore behavior's converter"""

import os
import pytest


OPTIONS_LIST = [
                    "--old-xar",
                    "--new-xar",
                    "--xml",
                    "--bhs",
                    "--anim",
                    "--fld"
               ]


def pytest_addoption(parser):
    parser.addoption("--old-xar", action="store", default="",
                     help="specify path of an old xar to test:\n"
                     + "- old format behavior.xar\n")
    parser.addoption("--new-xar", action="store", default="",
                     help="specify path of a new xar to test:\n"
                     + "- new format behavior.xar\n")
    parser.addoption("--xml", action="store", default="",
                     help="specify path of a behavior interface to test:\n"
                     + "- new format <box interface>.xml\n")
    parser.addoption("--bhs", action="store", default="",
                     help="specify path of a behavior sequence to test:\n"
                     + "- new format <behavior sequence>.bhs\n")
    parser.addoption("--anim", action="store", default="",
                     help="specify path of a animation to test:\n"
                     + "- new format <animation>.anim\n")
    parser.addoption("--fld", action="store", default="",
                     help="specify path of a flow diagram to test:\n"
                     + "- new format <flow diagram>.fld\n")


def _default_args():
    args = []
    currentDir = os.path.dirname(os.path.abspath(__file__))
    currentDir = os.path.realpath(os.path.join(currentDir,
                                               "..",
                                               "..",
                                               ".."))

    args.append(os.path.join(currentDir,
                             "xarconverter",
                             "tests",
                             "behavior1",
                             "behavior.xar"))
    args.append(os.path.join(currentDir,
                             "xarconverter",
                             "tests",
                             "behavior2",
                             "behavior.xar"))
    args.append(os.path.join(currentDir,
                             "xarconverter",
                             "tests",
                             "behavior2",
                             "B_0_0_Hello1.xml"))
    args.append(os.path.join(currentDir,
                             "xarconverter",
                             "tests",
                             "behavior2",
                             "B_0_0_Hello1.bhs"))
    args.append(os.path.join(currentDir,
                             "xarconverter",
                             "tests",
                             "behavior2",
                             "B_0_0_Hello1.anim"))
    args.append(os.path.join(currentDir,
                             "xarconverter",
                             "tests",
                             "behavior2",
                             "B_0_0_B1_0_FaceLeds0.fld"))
    return args


@pytest.fixture
def parse_args(request):
    """ extract args
    """
    args = []
    str_arg = ""
    fallback_args = _default_args()

    for i in range(len(OPTIONS_LIST)):
        str_arg = request.config.getoption(OPTIONS_LIST[i])
        if not str_arg:
            args.append(fallback_args[i])
        else:
            args.append(os.path.abspath(str_arg))

    return args
