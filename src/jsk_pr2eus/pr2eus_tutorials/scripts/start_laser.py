#!/usr/bin/env python
### copy from pr2_move_base/scripts/pr2_move_base.py
import roslib; roslib.load_manifest('pr2_move_base')
import rospy

from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import SetLaserTrajCmd
import dynamic_reconfigure.client

def set_tilt_profile(position, time_from_start):
    cmd = LaserTrajCmd()
    cmd.profile = 'blended_linear'
    cmd.position = position
    cmd.time_from_start = [rospy.Time.from_sec(x) for x in time_from_start]
    cmd.max_velocity = 10
    cmd.max_acceleration = 30
    try:
        tilt_profile_client.call(cmd)
    except rospy.ServiceException, e:
        rospy.logerr("Couldn't set the profile on the laser. Exception %s" % e)
        return False
    return True

def configure_laser():
    #TODO: remove hack to get things working in gazebo
    try:
        rospy.wait_for_service('tilt_hokuyo_node/set_parameters', 10.0)
    except rospy.exceptions.ROSException, e:
        rospy.logerr("Couldn't set parameters %s" % e)
        return
    #end TODO

    client = dynamic_reconfigure.client.Client('tilt_hokuyo_node')
    global old_config
    old_config = client.get_configuration(2.0)
    new_config = {'skip': 0, 'intensity': 0, 'min_ang': -1.57, 'max_ang': 1.57, 'calibrate_time': 1, 'cluster': 1, 'time_offset': 0.0}
    rospy.loginfo('Setting laser to the navigation configuration: %s' % new_config)
    client.update_configuration(new_config)

if __name__ == '__main__':
    name = 'setting_laser'
    rospy.init_node(name)

    #create a client to the tilt laser
    rospy.wait_for_service('laser_tilt_controller/set_traj_cmd')
    tilt_profile_client = rospy.ServiceProxy('laser_tilt_controller/set_traj_cmd', SetLaserTrajCmd)

    ## set_tilt_profile([1.05,  -.7, 1.05], [0.0, 1.8, 2.0125 + .3]) ## original
    set_tilt_profile([1.05,  -.7, 1.05], [0.0, 2.4, 2.4 + .2125 + .3])
    configure_laser()
