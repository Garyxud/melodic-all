# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtctree

Copyright (C) 2009-2014
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

Functions for parsing paths specifying name servers, directories, components,
etc.

'''


from rtctree.exceptions import BadPathError


##############################################################################
## API functions

def parse_path(path):
    '''Parses an address into directory and port parts.

    The last segment of the address will be checked to see if it matches a port
    specification (i.e. contains a colon followed by text). This will be
    returned separately from the directory parts.

    If a leading / is given, that will be returned as the first directory
    component. All other / characters are removed.

    All leading / characters are condensed into a single leading /.

    Any path components that are . will be removed, as they just point to the
    previous path component. For example, '/localhost/.' will become
    '/localhost'. Any path components that are .. will be removed, along with
    the previous path component. If this renders the path empty, it will be
    replaced with '/'.

    Examples:

    'localhost:30000/manager/comp0.rtc' ->
        (['localhost:30000', 'manager', 'comp0.rtc'], None)
    'localhost/manager/comp0.rtc:in' ->
        (['localhost', 'manager', 'comp0.rtc'], 'in')
    '/localhost/manager/comp0.rtc' ->
        (['/', 'localhost', 'manager', 'comp0.rtc'], None)
    '/localhost/manager/comp0.rtc:in' ->
        (['/', 'localhost', 'manager', 'comp0.rtc'], 'in')
    'manager/comp0.rtc' ->
            (['manager', 'comp0.rtc'], None)
    'comp0.rtc' ->
            (['comp0.rtc'], None)

    '''
    bits = path.lstrip('/').split('/')
    if not bits:
        raise BadPathError(path)

    if bits[-1]:
        bits[-1], port = get_port(bits[-1])
    else:
        port = None
    if path[0] == '/':
        bits = ['/'] + bits
    condensed_bits = []
    for bit in bits:
        if bit == '.':
            continue
        if bit == '..':
            condensed_bits = condensed_bits[:-1]
            continue
        condensed_bits.append(bit)
    if not condensed_bits:
        condensed_bits = ['/']
    return condensed_bits, port


def get_port(path):
    split_path = path.split(':')
    if len(split_path) == 1:
        return split_path[0], None
    elif len(split_path) == 2:
        return split_path[0], split_path[1]
    else:
        raise BadPathError(path)


def format_path(path):
    '''Formats a path as a string, placing / between each component.

    @param path A path in rtctree format, as a tuple with the port name as the
                second component.

    '''
    if path[1]:
        port = ':' + path[1]
    else:
        port = ''
    if type(path[0]) is str:
        # Don't add slashes if the path is singular
        return path[0] + port
    if path[0][0] == '/':
        starter = '/'
        path = path[0][1:]
    else:
        starter = ''
        path = path[0]
    return starter + '/'.join(path) + port


# vim: tw=79

