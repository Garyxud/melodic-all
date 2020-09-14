#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_master_info

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node('master')
    rocon_master = rocon_master_info.RoconMaster()
    rocon_master.spin()
