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
   :synopsis: Exceptions thrown parsing rocon launchers.

This module defines exceptions raised by the rocon_launch package.

----

"""

##############################################################################
# Exceptions
##############################################################################


class InvalidRoconLauncher(Exception):
    """ Raised when an error occurs in parsing a rocon launcher """
    pass


class UnsupportedTerminal(Exception):
    """ Raised when an unsupported terminal is requested or auto-detected."""
    pass
