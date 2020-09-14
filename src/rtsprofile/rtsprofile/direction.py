# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtsprofile

Copyright (C) 2009-2010
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

File: direction.py

Enumeration for directions.

This module stores the possible types of component direction.

Valid types are UP, DOWN, LEFT, RIGHT.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile.exceptions import InvalidDirectionError


UP = 'UP'
DOWN = 'DOWN'
LEFT = 'LEFT'
RIGHT = 'RIGHT'


# Use this to match the correct type when specifying data correctness
const_type = str


def from_string(dir_string):
    '''Returns the correct constant for a given string.

    @raises InvalidDirectionError

    '''
    dir_string = dir_string.upper()
    if dir_string == UP:
        return UP
    elif dir_string == DOWN:
        return DOWN
    elif dir_string == LEFT:
        return LEFT
    elif dir_string == RIGHT:
        return RIGHT
    else:
        raise InvalidDirectionError(dir_string)


def to_string(direction):
    '''Returns the correct string for a given direction.

    @raises InvalidDirectionError

    '''
    if direction == UP:
        return UP
    elif direction == DOWN:
        return DOWN
    elif direction == LEFT:
        return LEFT
    elif direction == RIGHT:
        return RIGHT
    else:
        raise InvalidDirectionError(type_string)


# vim: tw=79

