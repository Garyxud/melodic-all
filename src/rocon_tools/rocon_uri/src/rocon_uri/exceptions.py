#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: exceptions
   :platform: Unix
   :synopsis: Exceptions thrown parsing/formatting rocon uri's.

This module defines exceptions raised by the rocon_uri package.
These exception names are all included in the main
rocon_uri namespace.  To catch one, import it this
way::

    from rocon_uri import RoconURIValueError

----

"""

##############################################################################
# Exceptions
##############################################################################


class RoconURIValueError(Exception):
    """ Raised when an error occurs in parsing or manipulating rocon uri strings """
    pass
