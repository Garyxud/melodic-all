#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: launch
   :platform: Unix
   :synopsis: Machinery for spawning multiple roslaunchers.


This module contains the machinery for spawning and managing multiple terminals
that execute a pre-configured roslaunch inside each.

----

"""
##############################################################################
# Imports
##############################################################################

import os
import argparse
import signal
import sys
import roslaunch
import rocon_python_comms
import rosgraph
import rocon_console.console as console

from .exceptions import UnsupportedTerminal
from . import terminals
from . import utils

##############################################################################
# Methods
##############################################################################


def parse_arguments():
    """
    Argument parser for the rocon_launch script.
    """
    parser = argparse.ArgumentParser(description="Rocon's multi-roslauncher.")
    terminal_group = parser.add_mutually_exclusive_group()
    terminal_group.add_argument('-k', '--konsole', default=False, action='store_true', help='spawn individual ros systems via multiple konsole terminals')
    terminal_group.add_argument('-g', '--gnome', default=False, action='store_true', help='spawn individual ros systems via multiple gnome terminals')
    parser.add_argument('--screen', action='store_true', help='run each roslaunch with the --screen option')
    parser.add_argument('--no-terminals', action='store_true', help='do not spawn terminals for each roslaunch')
    parser.add_argument('--hold', action='store_true', help='hold terminals open after upon completion (incompatible with --no-terminals)')
    # Force package, launcher pairs, I like this better than roslaunch style which is a bit vague
    parser.add_argument('package', nargs='?', default='', help='name of the package in which to find the concert launcher')
    parser.add_argument('launcher', nargs=1, help='name of the concert launch configuration (xml) file')
    #parser.add_argument('launchers', nargs='+', help='package and concert launch configuration (xml) file configurations, roslaunch style')
    mappings = rosgraph.names.load_mappings(sys.argv)  # gets the arg mappings, e.g. scheduler_type:=simple
    argv = rosgraph.myargv(sys.argv[1:])  # strips the mappings
    args = parser.parse_args(argv)
    if args.no_terminals:
        args.terminal_name = terminals.active
    elif args.konsole:
        args.terminal_name = terminals.konsole
    elif args.gnome:
        args.terminal_name = terminals.gnome_terminal
    else:
        args.terminal_name = None
    return (args, mappings)


class RoconLaunch(object):
    """
    The rocon launcher. Responsible for all the pieces of the puzzle
    that go into spawning roslaunches inside terminals (startup, shutdown
    and signal handling).
    """
    __slots__ = [
                 'terminal',
                 'processes',
                 'temporary_files',  # store temporary files that need to unlinked on shutdown
                 'hold'  # keep terminals open when sighandling them
                ]

    def __init__(self, terminal_name, hold=False):
        """
        Initialise empty of processes, but configure a terminal
        so we're ready to go when we start spawning.

        :param str terminal_name: string name (or None) for the terminal to use.
        :param bool hold: whether or not to hold windows open or not.
        """
        self.processes = []
        """List of spawned :class:`subprocess.Popen` terminal processes."""
        self.temporary_files = []
        """List of temporary files used to construct launches that we must unlink before finishing."""
        self.hold = hold
        try:
            self.terminal = terminals.create_terminal(terminal_name)
        except (UnsupportedTerminal, rocon_python_comms.NotFoundException) as e:
            console.error("Cannot find a suitable terminal [%s]" % str(e))
            sys.exit(1)

    def signal_handler(self, sig, frame):
        '''
          Special handler that gets triggered if someone hits CTRL-C in the original terminal that executed
          'rocon_launch'. We catch the interrupt here, search and eliminate all child roslaunch processes
          first (give them time to gracefully quit) and then finally close the terminals themselves.
          closing down the terminals themselves.

          :param str sig: signal id (usually looking for SIGINT - 2 here)
          :param frame: frame
        '''
        self.terminal.shutdown_roslaunch_windows(self.processes, self.hold)
        # Have to unlink them here rather than in the actual spawning stage,
        # because terminal subprocesses take a while to kick in (in the background)
        # and the unlinking may occur before it actually runs the roslaunch that
        # needs the file.
        for f in self.temporary_files:
            os.unlink(f.name)

    def spawn_roslaunch_window(self, launch_configuration):
        """
        :param launch_configuration:
        :type launch_configuration :class:`.RosLaunchConfiguration`
        """
        process, meta_roslauncher = self.terminal.spawn_roslaunch_window(launch_configuration)
        self.processes.append(process)
        self.temporary_files.append(meta_roslauncher)


def main():
    """
    Main function for the rocon_launch script.
    """
    (args, mappings) = parse_arguments()
    rocon_launch = RoconLaunch(args.terminal_name, args.hold)
    signal.signal(signal.SIGINT, rocon_launch.signal_handler)

    if args.package == '':
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments(args.launcher)[0]
    else:
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments([args.package] + args.launcher)[0]
    if args.screen:
        roslaunch_options = "--screen"
    else:
        roslaunch_options = ""
    launchers = utils.parse_rocon_launcher(rocon_launcher, roslaunch_options, mappings)
    for launcher in launchers:
        console.pretty_println("Launching %s on port %s" % (launcher.path, launcher.port), console.bold)
        rocon_launch.spawn_roslaunch_window(launcher)
    signal.pause()
