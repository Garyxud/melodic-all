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

File: participant.py

Object representing a participating component.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S
from rtsprofile.exceptions import InvalidParticipantNodeError
from rtsprofile.targets import TargetComponent


##############################################################################
## Participant object

class Participant(object):
    '''This object contains a reference to a component object that is part of a
    composite component.

    '''

    def __init__(self, target_component=None):
        '''Constructor.

        @param target_component The target component of this participant.
        @type target_component TargetComponent

        '''
        validate_attribute(target_component, 'participant.target_component',
                           expected_type=TargetComponent, required=False)
        self._target_component = target_component

    def __str__(self):
        return str(self.target_component)

    @property
    def target_component(self):
        '''The target component of this participant.'''
        return self._target_component

    @target_component.setter
    def target_component(self, target_component):
        validate_attribute(target_component, 'participant.target_component',
                           expected_type=TargetComponent, required=True)
        self._target_component = target_component

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a participant into this
        object.

        '''
        if node.getElementsByTagNameNS(RTS_NS, 'Participant').length != 1:
            raise InvalidParticipantNodeError
        self.target_component = TargetComponent().parse_xml_node(\
                node.getElementsByTagNameNS(RTS_NS, 'Participant')[0])
        return self

    def parse_yaml_node(self, y):
        '''Parse a YAML specification of a participant into this object.'''
        if 'participant' not in y:
            raise InvalidParticipantNodeError
        self.target_component = TargetComponent().parse_yaml_node(y['participant'])
        return self

    def save_xml(self, doc, element):
        '''Save this participant into an xml.dom.Element object.'''
        new_element = doc.createElementNS(RTS_NS, RTS_NS_S + 'Participant')
        self.target_component.save_xml(doc, new_element)
        element.appendChild(new_element)

    def to_dict(self):
        '''Save this participant into a dictionary.'''
        return {'participant': self.target_component.to_dict()}


# vim: tw=79

