import sys
import os
#from .ice.motorsIceClient import MotorsIceClient
from .tools import server2int

rosversion = os.environ["ROS_VERSION"]
server = int(rosversion)

if ( server == 2):
    import rclpy
    from .ros2.listenerCameraros2 import ListenerCameraros2

if ( server == 1):
    import rospy
    from .ros.publisherMotors import PublisherMotors

def __getPublisherMotors(jdrc, prefix):
    '''
    Returns a Motors ROS Publisher. This function should never be used. Use getMotorsClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return Motors ROS Publisher

    '''
    if (sys.version_info[0] == 2):
        print("Publishing "+  prefix + " with ROS messages")
        topic = jdrc.getConfig().getProperty(prefix+".Topic")

        maxW = jdrc.getConfig().getPropertyWithDefault(prefix+".maxW", 0.5)
        if not maxW:
            maxW = 0.5
            print (prefix+".maxW not provided, the default value is used: "+ repr(maxW))


        maxV = jdrc.getConfig().getPropertyWithDefault(prefix+".maxV", 5)
        if not maxV:
            maxV = 5
            print (prefix+".maxV not provided, the default value is used: "+ repr(maxV))


        client = PublisherMotors(topic, maxV, maxW)
        return client
    else:
        print(prefix + ": ROS msg are diabled for python "+ sys.version_info[0])
        return None

def __Motorsdisabled(jdrc, prefix):
    '''
    Prints a warning that the client is disabled. This function should never be used. Use getMotorsClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return None

    '''
    print(prefix + " Disabled")
    return None

def getMotorsClient (jdrc, prefix):
    '''
    Returns a Motors Client.

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file
    @param node: ROS node

    @type jdrc: Comm Communicator
    @type prefix: String
    @type node: ROS node

    @return None if Motors is disabled

    '''
    server = jdrc.getConfig().getProperty(prefix+".Server")
    server = server2int(server)

    cons = [__Motorsdisabled, __getMotorsIceClient, __getPublisherMotors]

    return cons[server](jdrc, prefix)
