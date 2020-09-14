#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: system.command_line_wrappers
   :platform: Unix
   :synopsis: Call command line executables with convenience wrappers.

This module wraps a few command line executables with a convenient python api.

----

"""

##############################################################################
# Imports
##############################################################################

import os

##############################################################################
# Methods
##############################################################################


def which(program):
    '''
      Wrapper around the command line 'which' program.

      :returns: path to the program or None if it doesnt exist.
      :rtype: str or None
    '''
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, unused_fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None
