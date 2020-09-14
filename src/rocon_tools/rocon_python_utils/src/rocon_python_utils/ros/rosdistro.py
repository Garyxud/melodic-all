#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.rosdistro
   :platform: Unix
   :synopsis: Lookup the rosdistro version in runtime.

----

"""
##############################################################################
# Imports
##############################################################################

import rospy

##############################################################################
# Methods
##############################################################################


def get_rosdistro():
    '''
      Abstraction for pulling the rosdistro version (e.g. groovy/hydro). We
      are using the configured parameter here, but you could equally use one
      of the non-runtime techniques in rospkg elucidated on ros answers by
      Jack:

      http://answers.ros.org/question/36485/how-do-i-test-the-ros-version-in-python-code/

      :returns: the human readable string version for the ros distro
      :rtype: str
    '''
    return rospy.get_param("/rosdistro").rstrip()
