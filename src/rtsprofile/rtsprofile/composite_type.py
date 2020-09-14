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

File: composite_type.py

Enumeration for composite types.

This module stores the possible types of component composition.

Valid types are:
NONE: The component is not in a composite component.
ALLSHARED: Composition sharing all attributes. All attributes (execution
    context and status) are shared amongst all components in the composition.
    This is the strongest type of composition.
ECSHARED: Only the execution context is shared amongst components in the
    composition.
NONSHARED: Nothing is shared amongst components in the composition. This is the
    weakest type of composition.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile.exceptions import InvalidCompositeTypeError

NONE = 'None'
PERIODIC_EC_SHARED = 'PeriodicECShared'
PERIODIC_STATE_SHARED = 'PeriodicStateShared'
GROUPING = 'Grouping'
FSM_EC_SHARED = 'FsmECSHared'
FSM_STATE_SHARED = 'FsmStateShared'


# Use this to match the correct type when specifying data correctness
const_type = str


def from_string(type_string):
    '''Returns the correct constant for a given string.

    @raises InvalidCompositeTypeError

    '''
    if type_string == NONE:
        return NONE
    elif type_string == PERIODIC_EC_SHARED:
        return PERIODC_EC_SHARED
    elif type_string == PERIODIC_STATE_SHARED:
        return PERIODIC_STATE_SHARED
    elif type_string == GROUPING:
        return GROUPING
    elif type_string == FSM_EC_SHARED:
        return FSM_EC_SHARED
    elif type_string == FSM_STATE_SHARED:
        return FSM_STATE_SHARED
    else:
        raise InvalidCompositeTypeError(type_string)


def to_string(comp_type):
    '''Returns the correct string for a given composite type.

    @raises InvalidCompositeTypeError

    '''
    if comp_type == NONE:
        return NONE
    elif comp_type== PERIODIC_EC_SHARED:
        return PERIODC_EC_SHARED
    elif comp_type == PERIODIC_STATE_SHARED:
        return PERIODIC_STATE_SHARED
    elif comp_type == GROUPING:
        return GROUPING
    elif comp_type == FSM_EC_SHARED:
        return FSM_EC_SHARED
    elif comp_type == FSM_STATE_SHARED:
        return FSM_STATE_SHARED
    else:
        raise InvalidCompositeTypeError(type_string)


# vim: tw=79

