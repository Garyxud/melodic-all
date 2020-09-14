#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_python_comms


##############################################################################
# Main
##############################################################################


if __name__ == '__main__':
    rospy.init_node('connection_cache')
    # print(rocon_python_comms.__file__)  # to make sure we get the module from the right place
    conn_cache = rocon_python_comms.ConnectionCacheNode()
    conn_cache.spin()


