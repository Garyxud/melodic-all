#!/usr/bin/env python
#
# @brief Visual Studio Project GUID pickup script
# @date $Date: 2007-07-20 15:37:16 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2007
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

import sys
import re

file = open(sys.argv[1], "r")
lines = file.readlines()

name = ""
guid = ""

re_guid = re.compile('^.*?ProjectGUID=\"{(.*)}\"')
re_name = re.compile('^.*?Name=\"(.*)\"')

for l in lines:
    n = re_name.match(l)
    g = re_guid.match(l)

    if name == "" and n:
        name = n.group(1)
    if g:
        guid = g.group(1)

    if name != "" and guid != "":
        break

print name + "GUID:", guid
