#!/usr/bin/env python
import sys

import rospy
from std_srvs.srv import Trigger, TriggerResponse

confirm_msg = "trigger received"


def handle_msg(rq):
    print("Trigger Node got request")
    return TriggerResponse(success=True, message=confirm_msg)


def trigger_server():
    args = rospy.myargv(argv=sys.argv)
    node_name = args[1] if len(args) > 1 else 'trigger_node'

    rospy.init_node(node_name)
    rospy.set_param('/test/confirm_param', confirm_msg)
    srv = rospy.Service('/test/trgsrv', Trigger, handle_msg)
    rospy.spin()
    if rospy.has_param('/test/confirm_param'):
        rospy.delete_param('/test/confirm_param')


if __name__ == '__main__':
    trigger_server()
