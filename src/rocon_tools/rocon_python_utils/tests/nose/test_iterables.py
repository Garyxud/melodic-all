#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function
import os
import rocon_console.console as console
from rocon_python_utils.iterables import looktotheend, lookahead

##############################################################################
# Tests
##############################################################################

def test_looktotheend():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Look To The End" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    last_results = []
    for i, last in looktotheend(range(3)):
        print("%s, %s" % (i, last))
        last_results.append(last)
    assert(not last_results[0])
    assert(not last_results[1])
    assert(last_results[2])

def test_lookahead():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Look Ahead" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    next_results = []
    for current, next in lookahead(range(3)):
        print("%s, %s" % (current, next))
        next_results.append(next)
    assert(next_results[0] == 1)
    assert(next_results[1] == 2)
    assert(next_results[2] is None)
