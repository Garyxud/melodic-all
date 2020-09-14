#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_console.console as console

import rospy

##############################################################################
# Tests
##############################################################################
    
def test_master_available():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Master Available" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    result = rocon_python_comms.master.check()
    assert(not result)
