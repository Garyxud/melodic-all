#! /bin/sh

import rospy

class NoDefault:
    pass

def get_interface_parameter(interface, name, default = NoDefault):
    try:
        return rospy.get_param("~interfaces/%s/%s"%(interface, name))
    except KeyError:
        pass    
    
    try:
        return rospy.get_param("~default/%s"%name)
    except KeyError:
        pass

    if default is not NoDefault:
        return default

    raise KeyError("Could not find parameter %s for interface %s."%(interface, parameter))
    
def get_parameter(name, default = NoDefault):
    try:
        return rospy.get_param("~%s"%name)
    except:
        if default is not NoDefault:
            return default
    
    raise KeyError("Could not find parameter %s."%name)
