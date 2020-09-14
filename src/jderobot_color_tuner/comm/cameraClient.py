import sys
import os
from .tools import server2int

rosversion = os.environ["ROS_VERSION"]
ser = int(rosversion)

if ( ser == 2):
    import rclpy
    from .ros2.listenerCameraros2 import ListenerCameraros2

if ( ser == 1):
    import rospy
    from .ros.listenerCamera import ListenerCamera


def __getCameraROS2Client(jdrc, prefix):
    '''
    Returns a Camera ROS2 Subscriber

    '''    
    if (ser == 2):
        print("Receiving " + prefix + " Image from ROS2 messages")
        topic = jdrc.getConfig().getProperty(prefix+".Topic")
        client = ListenerCameraros2(topic, jdrc.noderos2)
        return client
    else:
        print(prefix + ": ROS2 msg are diabled for python "+ sys.version_info[0])
        return None

def __getListenerCamera(jdrc, prefix):
    '''
    @return Camera ROS Subscriber

    '''
    if (ser == 1):
        print("Receiving " + prefix + " Image from ROS messages")
        topic = jdrc.getConfig().getProperty(prefix+".Topic")
        client = ListenerCamera(topic)
        return client
    else:
        print(prefix + ": ROS msg are diabled for python "+ sys.version_info[0])
        return None

def __Cameradisabled(jdrc, prefix):
    '''
    Prints a warning that the client is disabled. This function should never be used. Use getCameraClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @return None

    '''
    print( prefix + " Disabled")
    return None

def getCameraClient (jdrc, prefix):
    '''
    Returns a Camera Client.

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type name: String

    @return None if Camera is disabled

    '''
    server = jdrc.getConfig().getProperty(prefix+".Server")
    server = server2int(server)

    cons = [__Cameradisabled, __getCameraROS2Client, __getListenerCamera]

    return cons[server](jdrc, prefix)
