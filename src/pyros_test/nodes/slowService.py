#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse

delay = 30


def handle_msg(rq):
    print("Slow Node got request")
    rospy.rostime.wallsleep(delay)
    return EmptyResponse()


def empty_server():
    args = rospy.myargv(argv=sys.argv)
    node_name = args[1] if len(args) > 1 else 'slow_node'

    rospy.init_node(node_name)
    rospy.set_param('~slow_param', delay)
    srv = rospy.Service('/test/slowsrv', Empty, handle_msg)
    rospy.spin()
    if rospy.has_param('slow_param'):
        rospy.delete_param('~slow_param')

if __name__ == '__main__':
    empty_server()
