#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" This modules provides functions useful to patch code contained in a box
.. module:: converter
"""

import converter.xar_types as xar_types


def patch(script):
    """ Clean the code contained in script boxes
        For now, script boxes can contain Python or QiChat script

        :param box: the box that holds the code
        :returns: the patched code as a string
    """
    code = script.content

    # remove all unwanted characters at the beginning of the code
    code = code.lstrip(' \t\r\n')
    code = code.rstrip(' \t\r\n')
    code += '\n'

    if script.language == xar_types.ScriptLanguage.QICHAT:
        code = _patch_qichat_script(code)
    elif script.language == xar_types.ScriptLanguage.PYTHON:
        code = _patch_python_script(code)

    return code


def _patch_qichat_script(code):
    """ Apply modifications to old version qichat script
        to make it correspond to new behavior format files
    """

    # for now, nothing to do more than the lstrip() made upper
    return code


def _patch_python_script(code):
    """ Apply modifications to old version python script
        to make it correspond to new behavior format files
    """

    # for now, nothing to do more than the lstrip() made upper
    return code
