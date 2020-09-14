#!/usr/bin/env python
import sys

import rospy
from std_srvs.srv import Empty, EmptyResponse


def handle_msg(rq):
    print("Empty Node got request")
    # TODO: assert rq is Empty Request (needed or done by rospy already ?)
    return EmptyResponse()


def empty_server():

    args = rospy.myargv(argv=sys.argv)
    node_name = args[1] if len(args) > 1 else 'empty_node'

    rospy.init_node(node_name)

    # TODO : seems not possible with ROS ?
    #   File "/usr/lib/python2.7/xmlrpclib.py", line 659, in dump_nil
    # raise TypeError, "cannot marshal None unless allow_none is enabled"
    # TypeError: cannot marshal None unless allow_none is enabled
    #rospy.set_param('~empty_param', None)

    srv = rospy.Service('/test/empsrv', Empty, handle_msg)
    rospy.spin()
    if rospy.has_param('empty_param'):
        rospy.delete_param('~empty_param')

if __name__ == '__main__':
    empty_server()
