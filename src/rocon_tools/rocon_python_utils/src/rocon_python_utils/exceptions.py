#!/usr/bin/env python
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
   :synopsis: General purpose exceptions for the rocon framework.

This module is a library of general purpose exceptions that fill general needs
not catered for by the `python exception heirarchy`_.
These exception names are all included in the main
rocon_python_utils namespace.  To catch one, import it this
way::

    from rocon_python_comms import TimeoutExpiredError

.. _`python exception heirarchy` : https://docs.python.org/2/library/exceptions.html#exception-hierarchy

----

"""
##############################################################################
# Exceptions
##############################################################################


class TimeoutExpiredError(Exception):
    """General purpose exception usable in any timeout situation."""
    pass
