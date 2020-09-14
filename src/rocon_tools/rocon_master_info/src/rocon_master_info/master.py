#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: master
   :platform: Unix
   :synopsis: Advertising info about a ros/rocon master.


This module contains the machinery for advertising basic information about a
ros master.

----

"""
##############################################################################
# Imports
##############################################################################

import rospkg
import rospy
import rocon_console.console as console
import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri

##############################################################################
# Parameters
##############################################################################


class Parameters:
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar robot_type: used for `rocon_uri`_ rapp compatibility checks *['rapp_manager_script']*
    :vartype robot_type: str
    :ivar robot_name: also used for `rocon_uri`_ rapp compatibility checks *['robot']*
    :vartype robot_name: str
    :ivar robot_icon: a `resource name` pointing to a representative icon for this platform. *['rocon_bubble_icons/rocon.png']*
    :vartype robot_icon: str

    .. _rocon_uri: http://wiki.ros.org/rocon_uri
    .. _resource name: http://wiki.ros.org/Names#Package_Resource_Names
    """
    def __init__(self):
        # see sphinx docs above for more detailed explanations of each parameter
        self.type = rospy.get_param('~type', 'robot')
        self.name = rospy.get_param('~name', 'cybernetic_pirate')
        self.icon = rospy.get_param('~icon', 'rocon_icons/cybernetic_pirate.png')
        self.description = rospy.get_param('~description', 'A rocon system.')
        self.version = rocon_std_msgs.Strings.ROCON_VERSION
        # and set a global version parameter (useful as a ping to check for a rocon master (e.g. by androids)
        rospy.set_param('version', self.version)

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s


class RoconMaster(object):
    """
    This class accepts a few parameters describing the ros master and then
    publishes the ros master info on a latched publisher. Publishing is necessary
    because an icon can only be represented by it's location as a parameter. It
    is easier directly publishing the icon rather than having clients go do
    the lookup themselves.
    """
    __slots__ = [
        'publishers',
        'parameters',
        'spin',
    ]

    def __init__(self):
        '''
        Retrieves ``name``, ``description`` and ``icon`` parameters from the
        parameter server and publishes them on a latched ``info`` topic.
        The icon parameter must be a ros resource name (pkg/filename).
        '''
        ##################################
        # Pubs, Subs and Services
        ##################################
        self.publishers = {}
        # efficient latched publisher, put in the public concert namespace.
        self.parameters = Parameters()
        self.publishers["info"] = rospy.Publisher("~info", rocon_std_msgs.MasterInfo, latch=True, queue_size=1)
        master_info = rocon_std_msgs.MasterInfo()
        master_info.name = self.parameters.name
        master_info.description = self.parameters.description
        master_info.rocon_uri = rocon_uri.generate_platform_rocon_uri(self.parameters.type, self.parameters.name)
        try:
            master_info.icon = rocon_python_utils.ros.icon_resource_to_msg(self.parameters.icon)
        except rospkg.ResourceNotFound as e:
            rospy.logwarn("Master Info : no icon found, using a default [%s][%s]" % (self.parameters.icon, e))
            master_info.icon = rocon_python_utils.ros.icon_resource_to_msg("rocon_bubble_icons/rocon_logo.png")
        master_info.version = rocon_std_msgs.Strings.ROCON_VERSION
        self.publishers['info'].publish(master_info)
        # Aliases
        self.spin = rospy.spin
        """Spin function, currently this just replicates the rospy spin function since everything is done in the constructor."""

