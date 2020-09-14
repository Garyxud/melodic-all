#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: publisher
   :platform: Unix
   :synopsis: Extending the ros publisher.


----

"""
##############################################################################
# Imports
##############################################################################

import rospy

##############################################################################
# Publisher
##############################################################################


class Publisher(rospy.Publisher):
    """
    If you publish too soon after creating the publisher, the message gets
    lost in the void. We have no way of discerning when this happens.
    Services are sometimes undesirable because of the blocking or the extra
    effort needed to make a small service unblocking so sometimes a
    poor, but practical option is just to wait a bit.
    """
    def __init__(self, *args, **kwargs):
        """
        Matches the same signature as the rospy.Publisher class.
        """
        super(Publisher, self).__init__(*args, **kwargs)
        self._timer = rospy.Timer(rospy.Duration(0.5), self._mark_is_ready, oneshot=True)
        self._is_ready = False

    def _mark_is_ready(self, unused_event):
        self._is_ready = True

    def is_ready(self):
        return self._is_ready
