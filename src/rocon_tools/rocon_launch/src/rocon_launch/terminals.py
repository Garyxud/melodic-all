#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: terminals
   :platform: Unix
   :synopsis: Terminal specific support.


This module provides a list of supported terminals and methods to handle them.

----

"""
##############################################################################
# Imports
##############################################################################

import os
import rocon_console.console as console
import rocon_python_comms
import rocon_python_utils
import signal
import sys
import subprocess
import tempfile
import time
from urlparse import urlparse

from .exceptions import UnsupportedTerminal
from . import utils

##############################################################################
# Supported Terminals
##############################################################################

active = "active"  # the currently open terminal
"""String identifier for the currently open (active) terminal"""
konsole = "konsole"
"""String identifier for KDE's konsole terminal"""
gnome_terminal = "gnome-terminal"
"""String identifier for Gnome's terminal."""
gnome_terminal_wrapper = "gnome-terminal.wrapper"  # some systems use this for gnome-terminal
"""String identifier for an oft used representation of gnome's terminal on desktops like KDE."""

##############################################################################
# Terminal
##############################################################################


class Terminal(object):
    __slots__ = ['name']

    def __init__(self, name):
        """
        Creates a manager for the terminal with supporting methods and variables.
        :param str name: name of this terminal.
        """
        self.name = name

    def shutdown_roslaunch_windows(self, processes, hold):
        """
        Shuts down a roslaunch window cleanly, i.e. it first kills the roslaunch
        processes, then kills the terminal itself.
        """
        roslaunch_pids = []
        for process in processes:
            roslaunch_pids.extend(utils.get_roslaunch_pids(process.pid))
        # kill roslaunch's
        for pid in roslaunch_pids:
            try:
                os.kill(pid, signal.SIGHUP)
            except OSError:
                continue
        for pid in roslaunch_pids:
            console.pretty_println("Terminating roslaunch [pid: %d]" % pid, console.bold)
            rocon_python_utils.system.wait_pid(pid)
            # console.pretty_println("Terminated roslaunch [pid: %d]" % pid, console.bold)
        time.sleep(1)
        if hold:
            try:
                raw_input("Press <Enter> to close terminals...")
            except RuntimeError:
                pass  # this happens when you ctrl-c again instead of enter
        # now kill the terminal itself
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except OSError:
                console.warning("Kill signal failed to reach the terminal - typically this means the terminal has already shut down.")
            except TypeError as e:
                console.error("Invalid pid value [%s][%s]" % (str(process.pid), str(e)))
            # process.terminate()

    def _prepare_meta_roslauncher(self, roslaunch_configuration):
        """
        Generate a meta roslauncher which calls our real roslaunch provided in the
        launch configuration. This applies the more esoteric options specified
        in the launch configuration, e.g. screen, args.

        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :returns: handle to the temporary meta roslaunch file
        :rtype: :class:`tempfile.NamedTemporaryFile`
        """
        ros_launch_file = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        # print("Launching %s" % temp.name)
        launch_text = '<launch>\n'
        if roslaunch_configuration.screen():
            launch_text += '  <param name="rocon/screen" value="true"/>\n'
        else:
            launch_text += '  <param name="rocon/screen" value="false"/>\n'
        if roslaunch_configuration.namespace:
            launch_text += '  <group ns="%s">\n' % roslaunch_configuration.namespace
        launch_text += '  <include file="%s">\n' % roslaunch_configuration.path
        for (arg_name, arg_value) in roslaunch_configuration.args:
            launch_text += '    <arg name="%s" value="%s"/>\n' % (arg_name, arg_value)
        launch_text += '  </include>\n'
        if roslaunch_configuration.namespace:
            launch_text += '  </group>\n'
        launch_text += '</launch>\n'
        # print launch_text
        ros_launch_file.write(launch_text)
        ros_launch_file.close()  # unlink it later
        return ros_launch_file

    def spawn_roslaunch_window(self, roslaunch_configuration, postexec_fn=None, env={}):
        """
        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param func postexec_fn: run this after the subprocess finishes
        :param dict env: a additional customised environment to run ros launcher, {key : value}
        :returns: the subprocess and temp roslaunch file handles
        :rtype: (:class:`subprocess.Popen`, :class:`tempfile.NamedTemporaryFile`
        """
        if self.__class__ is Terminal:
            console.logerror("Do not use 'Terminal' directly, it is an abstract base class")
            sys.exit(1)
        if 'prepare_roslaunch_command' not in vars(self.__class__):
            console.logerror("The method _prepare_roslaunch_command must be implemented in children of rocon_launch.terminals.Terminal")
            sys.exit(1)
        meta_roslauncher = self._prepare_meta_roslauncher(roslaunch_configuration)
        cmd = self.prepare_roslaunch_command(roslaunch_configuration, meta_roslauncher.name)  # must be implemented in children
        # ROS_NAMESPACE is typically set since we often call this from inside a node
        # itself. Got to get rid of this otherwise it pushes things down
        roslaunch_env = os.environ.copy()
        if len(env) != 0:
            for key in env.keys():
                roslaunch_env[key] = env[key]
        try:
            roslaunch_env['ROS_MASTER_URI'] = roslaunch_env['ROS_MASTER_URI'].replace(str(urlparse(roslaunch_env['ROS_MASTER_URI']).port), str(roslaunch_configuration.port))
            del roslaunch_env['ROS_NAMESPACE']
        except KeyError:
            pass
        return (rocon_python_utils.system.Popen(cmd, postexec_fn=postexec_fn, env=roslaunch_env), meta_roslauncher)

    def spawn_executable_window(self, title, command, postexec_fn=None, env={}):
        if self.__class__ is Terminal:
            console.logerror("Do not use 'Terminal' directly, it is an abstract base class")
            sys.exit(1)
        if 'prepare_roslaunch_command' not in vars(self.__class__):
            console.logerror("The method _prepare_roslaunch_command must be implemented in children of rocon_launch.terminals.Terminal")
            sys.exit(1)
        cmd = self.prepare_command(title, command)  # must be implemented in children
        our_env = os.environ.copy()
        if len(env) != 0:
            for key in env.keys():
                our_env[key] = env[key]
        # ROS_NAMESPACE is typically set since we often call this from inside a node
        # itself. Got to get rid of this otherwise it pushes things down
        try:
            del our_env['ROS_NAMESPACE']
        except KeyError:
            pass
        return rocon_python_utils.system.Popen(cmd, postexec_fn=postexec_fn, env=our_env)

##############################################################################
# Active
##############################################################################


class Active(Terminal):
    """
    A pseudo representation of the currently open terminal.
    """

    def __init__(self):
        """Dude"""
        super(Active, self).__init__(active)

    def prepare_command(self, title, cmd):
        """
        Prepare a regular command to run inside the terminal window.

        :param str title: label to put above on the terminal window
        :param str[] cmd: command to run
        """
        return cmd

    def prepare_roslaunch_command(self, roslaunch_configuration, meta_roslauncher_filename):
        """
        Prepare the custom command for a roslaunch window.

        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param str meta_roslauncher_filename: temporary roslauncher file
        """
        cmd = ["roslaunch"]
        if roslaunch_configuration.options:
            cmd.append(roslaunch_configuration.options)
        cmd.append(meta_roslauncher_filename)
        return cmd

##############################################################################
# Konsole
##############################################################################


class Konsole(Terminal):
    """
    Responsible for handling of kde konsole terminals.
    """

    def __init__(self):
        super(Konsole, self).__init__(konsole)

    def prepare_command(self, title, cmd):
        """
        Prepare a regular command to run inside the terminal window.

        :param str title: label to put above on the terminal window
        :param str[] cmd: command to run
        """
        cmd = [self.name,
               '-p',
               'tabtitle=%s' % title,
               '--nofork',
               '--hold',
               '-e',
               "/bin/bash",
               "-c",
               " ".join(cmd)]
        return cmd

    def prepare_roslaunch_command(self, roslaunch_configuration, meta_roslauncher_filename):
        """
        Prepare the custom command for a roslaunch window.

        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param str meta_roslauncher_filename: temporary roslauncher file
        """
        cmd = [self.name,
               '-p',
               'tabtitle=%s' % roslaunch_configuration.title,
               '--nofork',
               '--hold',
               '-e',
               "/bin/bash",
               "-c",
               "roslaunch %s --disable-title %s" %
                   (roslaunch_configuration.options,
                    meta_roslauncher_filename)]
        return cmd

##############################################################################
# Gnome Terminal
##############################################################################


class GnomeTerminal(Terminal):
    """
    Responsible for handling of gnome-terminal terminals.
    """

    def __init__(self):
        super(GnomeTerminal, self).__init__(gnome_terminal)

    def prepare_command(self, title, cmd):
        """
        Prepare a regular command to run inside the terminal window.

        :param str title: label to put above on the terminal window
        :param str[] cmd: command to run
        """
        cmd = [self.name,
               '--title=%s' % title,
               '--disable-factory',
               "-e",
               "/bin/bash -c '%s --disable-title';/bin/bash" % " ".join(cmd)
               ]
        return cmd

    def prepare_roslaunch_command(self, roslaunch_configuration, meta_roslauncher_filename):
        """
        Prepare the custom command for a roslaunch window.

        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param str meta_roslauncher_filename: temporary roslauncher file
        """
        cmd = [self.name,
               '--title=%s' % roslaunch_configuration.title,
               '--disable-factory',
               "-e",
               "/bin/bash -c 'roslaunch %s --disable-title %s';/bin/bash" %
                    (roslaunch_configuration.options, meta_roslauncher_filename)
               ]
        return cmd

##############################################################################
# Factory
##############################################################################

supported_terminals = {active: Active,
                       konsole: Konsole,
                       gnome_terminal: GnomeTerminal,
                       gnome_terminal_wrapper: GnomeTerminal
                       }


def create_terminal(name=None):
    """
    Creates a manager for the terminal with supporting methods and variables.

    If name is None, it will try to auto-detect the user's terminal. We're currently
    using ubuntu's x-terminal-emulator to choose the shell.

    :param str name: name of the terminal manager to create (None to auto-detect).
    :returns: one of the suported terminal classes
    :rtype: one of the children of :class:.`.Terminal`

    :raises :exc:`.UnsupportedTerminal` if the name is not in the supported terminals list.
    :raises :exc:`rocon_python_comms.NotFoundException` if the specified/auto-detected terminal is not found on the system.
    """
    if name is not None and name not in supported_terminals.keys():
        raise UnsupportedTerminal("%s is not a supported terminal type [%s]" %
                         (name, supported_terminals.keys()))
    if name == konsole:
        if not rocon_python_utils.system.which('konsole'):
            msg = "cannot find 'konsole' (hint: try --gnome for gnome-terminal instead)"
            raise rocon_python_comms.NotFoundException(msg)
    elif name == gnome_terminal or name == gnome_terminal_wrapper:
        if not rocon_python_utils.system.which('konsole'):
            msg = "cannot find 'gnome' (hint: try --konsole for konsole instead)"
            raise rocon_python_comms.NotFoundException(msg)
    # elif name is active:  # nothing to do
    elif name is None:
        # auto-detect
        if not rocon_python_utils.system.which('x-terminal-emulator'):
            msg = "tried to auto-detect, but cannot find 'x-terminal-emulator' (hint: try --gnome or --konsole instead)"
            raise rocon_python_comms.NotFoundException(msg)
        p = subprocess.Popen([rocon_python_utils.system.which('update-alternatives'), '--query', 'x-terminal-emulator'], stdout=subprocess.PIPE)
        for line in p.stdout:
            if line.startswith("Value:"):
                auto_detected_name = os.path.basename(line.split()[1])
                break
        if auto_detected_name not in supported_terminals.keys():
            msg = "you are %s, an esoteric and unsupported terminal" % (auto_detected_name)
            console.warning(msg.capitalize())
            fallbacks = [konsole, gnome_terminal]
            for terminal_name in fallbacks:
                if rocon_python_utils.system.which(terminal_name):
                    name = terminal_name
                    console.warning(" --> falling back to '%s'" % terminal_name)
            if name is None:
                raise UnsupportedTerminal(msg + " (hint: try --gnome or --konsole instead)[%s]" % supported_terminals.keys())
        else:
            name = auto_detected_name
    return supported_terminals[name]()
