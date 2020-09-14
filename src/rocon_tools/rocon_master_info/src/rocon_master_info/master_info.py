#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: master_info
   :platform: Unix
   :synopsis: Client utilities for retrieving rocon master information.

Some methods for retrieving the rocon master information. Also provides the
main method for scripts/tools that do this.
----

"""
##############################################################################
# Imports
##############################################################################

import os
import sys
import rospy
import rocon_python_comms
import rocon_python_utils
import rocon_console.console as rocon_console
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Methods
##############################################################################


def get_master_info(timeout=1.0):
    '''
      Tries to gather the rocon master info but if not available, return
      with a rocon_std_msgs.MasterInfo_ object filled with appropriate information
      ("Unknown Master" ...).

      :param double timeout: how long to blather around looking for the master info topic.

      :returns: the master information
      :rtype: rocon_std_msgs.MasterInfo_

      .. include:: weblinks.rst
    '''
    # default values
    master_info = rocon_std_msgs.MasterInfo()
    master_info.name = "Unknown Master"
    master_info.description = "Unknown"
    master_info.version = rocon_std_msgs.Strings.ROCON_VERSION
    master_info.icon = rocon_python_utils.ros.icon_resource_to_msg('rocon_icons/unknown.png')

    try:
        topic_name = rocon_python_comms.find_topic('rocon_std_msgs/MasterInfo', timeout=rospy.rostime.Duration(timeout), unique=True)
    except rocon_python_comms.NotFoundException as e:
        print(rocon_console.red + "failed to find unique topic of type 'rocon_std_msgs/MasterInfo' [%s]" % str(e) + rocon_console.reset)
        master_info.description = "Is it rocon enabled? See http://wiki.ros.org/rocon_master_info"
        return master_info

    master_info_proxy = rocon_python_comms.SubscriberProxy(topic_name, rocon_std_msgs.MasterInfo)
    try:
        master_info_proxy.wait_for_publishers()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("Concert Info : ros shut down before rocon master info could be retrieved.")
        master_info.description = "Unkonwn"
        return master_info

    result = master_info_proxy(rospy.Duration(0.2))
    if result:
        master_info = result  # rocon_std_msgs.MasterInfo
    return master_info


##############################################################################
# Main
##############################################################################

def console_only_main(node_name='master_info', title='Master Information'):
    # Establishes a connection and prints master information to the console.
    rospy.init_node(node_name)
    master_info = get_master_info()
#     display_available = True if 'DISPLAY' in os.environ.keys() else False

    rocon_console.pretty_println(title, rocon_console.bold)
    print(rocon_console.cyan + "  Name       : " + rocon_console.yellow + master_info.name + rocon_console.reset)
    print(rocon_console.cyan + "  Rocon Uri  : " + rocon_console.yellow + master_info.rocon_uri + rocon_console.reset)
    print(rocon_console.cyan + "  Description: " + rocon_console.yellow + master_info.description + rocon_console.reset)
    print(rocon_console.cyan + "  Icon       : " + rocon_console.yellow + master_info.icon.resource_name + rocon_console.reset)
    print(rocon_console.cyan + "  Version    : " + rocon_console.yellow + master_info.version + rocon_console.reset)


def main(node_name='master_info', title='Master Information', console=True):
    display_available = True if 'DISPLAY' in os.environ.keys() else False
    try:
        import rocon_qt_master_info
        from rqt_gui.main import Main
        qt_available = True
    except ImportError:
        qt_available = False
        if display_available and not console:
            print(rocon_console.red + "WARNING: rqt plugin not found, console output only (hint: install rocon_qt_master_info)." + rocon_console.reset)

    if console or not display_available or not qt_available:
        console_only_main(node_name, title)
    else:
        main = Main()
        sys.exit(main.main(argv=sys.argv, standalone='rocon_qt_master_info'))
