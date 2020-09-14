#! /usr/bin/env python
import roslib; roslib.load_manifest('test_ros') # FIXME
import rospy
import std_msgs.msg
import rosgraph.masterapi

try:
    import rospy.masterslave as masterslave
except:
    import rospy.impl.masterslave as masterslave
try:
    import rospy.registration as registration
except:
    import rospy.impl.registration as registration

master_check_interval = 2

def new_publish(*args, **nargs):
    check_master()
    original_Publisher_publish(*args, **nargs)

def new_init_node(*args, **nargs):
    global check_interval
    global our_uri
    original_rospy_init_node(*args, **nargs)
    our_uri =  rospy.get_node_uri()
    check_interval = rospy.Duration(master_check_interval)
    check_master()

next_check_time = None
master_pid = None
pub_list = {}
def check_master():
    global next_check_time
    global master
    global our_uri
    global master_pid
    now = rospy.Time.now()
    if next_check_time == None or now > next_check_time:
        next_check_time = now + check_interval
        try:
            new_pid = master.getPid()
            if new_pid != master_pid:
                raise Exception
        except:
            try:
                master = rosgraph.masterapi.Master(rospy.get_name())
                for (name, type) in pub_list.iteritems():
                    master.registerPublisher(name, type, our_uri)
                master_pid = master.getPid()
                #print "Registered with master."
            except Exception, e:
                master = None
                master_pid = None

def new_registration_reg_added(self, resolved_name, data_type_or_uri, reg_type):
    if reg_type == registration.Registration.PUB:
        pub_list[resolved_name] = data_type_or_uri
        try:
            master.registerPublisher(resolved_name, data_type_or_uri, our_uri)
        except:
            pass
    return True

# Store original functions
original_Publisher_publish = rospy.Publisher.publish
original_rospy_init_node = rospy.init_node

# hot patch
masterslave.ROSHandler._is_registered = lambda x: True
registration.RegManager.reg_added = new_registration_reg_added
rospy.Publisher.publish = new_publish
rospy.init_node = new_init_node

## init with default topics disabled
#import rosmasterless
#
#rospy.init_node('ronin', disable_rosout=True, disable_rostime=True)
#
#pub = rospy.Publisher('chatter', std_msgs.msg.String)
#
#while not rospy.is_shutdown():
#    pub.publish('testing')
#    rospy.sleep(rospy.Duration(1))
