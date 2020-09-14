#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" QiCore Python Module """
from __future__ import absolute_import
from __future__ import unicode_literals
from __future__ import print_function

import os
import sys

def register():
    """ Register QiCore Python Module """
    # Get the Absolute Path if the Package
    path_package = os.path.dirname(os.path.realpath(__file__))
    # Get the Root Path of QiCore
    path_qicore = os.path.join(path_package, "linux")
    if "darwin" in sys.platform:
        path_qicore = os.path.join(path_package, "mac")
    elif "win" in sys.platform:
        path_qicore = os.path.join(path_package, "win")
    # Update QiAdditionalSdkPrefixes if needed
    qisdk_prefixes = os.environ.get("QI_ADDITIONAL_SDK_PREFIXES", "")
    if path_qicore not in qisdk_prefixes:
        if qisdk_prefixes:
            qisdk_prefixes += os.path.pathsep
        os.environ["QI_ADDITIONAL_SDK_PREFIXES"] = qisdk_prefixes + path_qicore
