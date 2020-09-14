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

File: exceptions.py

Exceptions that may occur.

'''

__version__ = '$Revision: $'
# $Source$


##############################################################################
## Exception types

class RtsProfileError(Exception):
    '''Base exception for all RtsProfile errors.'''
    pass


class MultipleSourcesError(Exception):
    '''Multiple XML sources were given.'''
    pass


class InvalidTypeError(RtsProfileError):
    '''Tried to set an attribute using an invalid type.'''
    pass


class RequiredAttributeError(RtsProfileError):
    '''Tried to set a required attribute as empty.'''
    pass


class InvalidCompositeTypeError(RtsProfileError):
    '''Tried to convert an invalid string to a CompositeType constant.'''
    pass


class InvalidDirectionError(RtsProfileError):
    '''Tried to convert an invalid string to a Direction constant.'''
    pass


class InvalidDataPortConnectorNodeError(RtsProfileError):
    '''A data port connector node's XML is invalid (e.g. too many children).'''
    pass


class InvalidParticipantNodeError(RtsProfileError):
    '''A participant node's XML is invalid (e.g. too many children).'''
    pass


class InvalidRtsProfileNodeError(RtsProfileError):
    '''The RtsProfile node's XML is invalid (e.g. too many children of a
    certain type).'''
    pass


class InvalidServicePortConnectorNodeError(RtsProfileError):
    '''A service port connector node's XML is invalid (e.g. too many
    children).

    '''
    pass

class MissingComponentError(RtsProfileError):
    '''A component expected to be present (e.g. by a TargetPort) is not.'''
    pass


# vim: tw=79

