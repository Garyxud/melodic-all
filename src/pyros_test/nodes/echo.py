#!/usr/bin/env python
import sys

import rospy
import pyros_test

##############################################################################
# Main
##############################################################################


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    node_name = args[1] if len(args) > 1 else 'pyros_test_echo'

    rospy.init_node(node_name)
    node = pyros_test.EchoNode()
    node.spin()

