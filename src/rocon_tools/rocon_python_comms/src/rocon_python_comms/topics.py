#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: topics
   :platform: Unix
   :synopsis: Useful methods relating to ros topics.


This module contains anything relating to introspection or manipulation
of ros topics.

----

"""
##############################################################################
# Imports
##############################################################################

import rospy
import time
import rostopic

from .exceptions import NotFoundException

##############################################################################
# Find topics/services
##############################################################################


def find_topic(topic_type, timeout=rospy.rostime.Duration(5.0), unique=False):
    '''
    Do a lookup to find topics of the type specified. It can apply the
    additional logic of whether this should return a single unique result,
    or a list.  Under the hood this calls out to the ros master for a list
    of registered topics and it parses that to determine the result. If nothing
    is found, it loops around internally on a 100ms loop until the result is
    found or the specified timeout is reached.

    This will raise exceptions if it times out or returns multiple values.

    Usage:

    .. code-block:: python

        from rocon_python_comms import find_topic

        try:
            pairing_topic_name = find_topic('rocon_interaction_msgs/Pair', timeout=rospy.rostime.Duration(0.5), unique=True)
        except rocon_python_comms.NotFoundException as e:
            rospy.logwarn("support for interactions disabled")

    :param str topic_type: topic type specification, e.g. rocon_std_msgs/MasterInfo
    :param rospy.rostime.Duration timeout: raise an exception if nothing is found before this timeout occurs.
    :param bool unique: flag to select the lookup behaviour (single/multiple results)

    :returns: the fully resolved name of the topic (unique) or list of names (non-unique)
    :rtype: str

    :raises: :exc:`.NotFoundException` if no topic is found within the timeout
    '''
    topic_name = None
    topic_names = []
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time and not topic_names:
        try:
            topic_names = rostopic.find_by_type(topic_type)
        except rostopic.ROSTopicException:
            raise NotFoundException("ros shutdown")
        if unique:
            if len(topic_names) > 1:
                raise NotFoundException("multiple topics found %s." % topic_names)
            elif len(topic_names) == 1:
                topic_name = topic_names[0]
        if not topic_names:
            rospy.rostime.wallsleep(0.1)
    if not topic_names:
        raise NotFoundException("timed out")
    return topic_name if topic_name else topic_names
