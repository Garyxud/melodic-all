#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: utils
   :platform: Unix
   :synopsis: Utilities supporting rocon_launch.


This module provides some supporting utilities for rocon launches.

----

"""
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import roslaunch
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

from .roslaunch_configuration import RosLaunchConfiguration

##############################################################################
# Public Methods
##############################################################################


def parse_rocon_launcher(rocon_launcher, default_roslaunch_options, args_mappings={}):
    '''
      Parses an rocon multi-launcher (xml file).

      :param str rocon_launcher: xml string in rocon_launch format
      :param default_roslaunch_options: options to pass to roslaunch (usually "--screen")
      :param dict args_mappings: command line mapping overrides, { arg_name : arg_value }
      :returns: list of launch configurations
      :rtype: :class:`.RosLaunchConfiguration`[]

      :raises :exc:`.InvalidRoconLauncher` : if any roslaunch configuration failed
    '''
    tree = ElementTree.parse(rocon_launcher)
    root = tree.getroot()
    # should check for root concert tag
    launchers = []
    ports = []

    # These are intended for re-use in launcher args via $(arg ...) like regular roslaunch
    vars_dict = {}
    # We do this the roslaunch way since we use their resolvers, even if we only do it for args.
    vars_dict['arg'] = {}
    args_dict = vars_dict['arg']  # convenience ref to the vars_dict['args'] variable
    for tag in root.findall('arg'):
        name, value = _process_arg_tag(tag, args_dict)
        args_dict[name] = value
    args_dict.update(args_mappings)  # bring in command line overrides
    for launch in root.findall('launch'):
        port = launch.get('port', RosLaunchConfiguration.default_port)
        args = []
        for tag in launch.findall('arg'):
            name, value = _process_arg_tag(tag, vars_dict)
            args.append((name, value))
        launch_configuration = RosLaunchConfiguration(
            name=launch.get('name'),
            package=launch.get('package'),
            port=port,
            title=launch.get('title', 'rocon_launch:%s' % str(port)),
            args=args,
            options=default_roslaunch_options
            )
        if port in ports:
            launch_configuration.append_option("--wait")
        else:
            ports.append(port)
        launchers.append(launch_configuration)
    return launchers


def get_roslaunch_pids(parent_pid):
    '''
      Search the pstree of the specified pid for roslaunch processes. We use this to
      aid in gracefully terminating any roslaunch processes running in terminals before
      closing down the terminals themselves.

      :param str parent_pid: the pid of the parent process.
      :returns: list of pids
      :rtype: str[]

    '''
    if parent_pid is None:
        console.warning("aborting call to find child roslaunches of a non-existant parent pid (can happen if cancelling spawned processes while they are still establishing).")
        return []
    ps_command = subprocess.Popen("ps -o pid -o comm --ppid %d --noheaders" % parent_pid, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    ps_output = ps_command.stdout.read()

    retcode = ps_command.wait()
    pids = []
    if retcode == 0:
        for pair in ps_output.split("\n")[:-1]:
            try:
                [pid, command] = pair.lstrip(' ').split(" ")
            except ValueError:  # when we can't unpack the output into two pieces
                # ignore, it's not a roslaunch
                console.warning("Rocon Launch : bad pair while scanning for roslaunch pids [%s]" % pair)
                continue
            if command == 'roslaunch':
                pids.append(int(pid))
            else:
                pids.extend(get_roslaunch_pids(int(pid)))
    else:
        # Presume this roslaunch was killed by ctrl-c or terminated already.
        # Am not worrying about classifying between the above presumption and real errors for now
        pass
    return pids


##############################################################################
# Internal Methods
##############################################################################


def _process_arg_tag(tag, args_dict=None):
    '''
    Process the arg tag. Kind of hate replicating what roslaunch does with
    arg tags, but there's no easy way to pull roslaunch code.

    :param args_dict: dictionary of args previously discovered
    :returns: name, value pairs for the args
    :rtype: (str, str)

    :todo: get rid of the sys.exits and replace with exceptions
    '''
    name = tag.get('name')  # returns None if not found.
    if name is None:
        console.error("<arg> tag must have a name attribute.")
        sys.exit(1)
    value = tag.get('value')
    default = tag.get('default')
    #print("Arg tag processing: (%s, %s, %s)" % (name, value, default))
    if value is not None and default is not None:
        console.error("<arg> tag must have one and only one of value/default attributes specified.")
        sys.exit(1)
    if value is None and default is None:
        console.error("<arg> tag must have one of value/default attributes specified.")
        sys.exit(1)
    if value is None:
        value = default
    if value and '$' in value:
        value = roslaunch.substitution_args.resolve_args(value, args_dict)
    return (name, value)
