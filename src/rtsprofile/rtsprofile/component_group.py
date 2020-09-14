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

File: component_group.py

Object representing a component group.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S
from rtsprofile.utils import validate_attribute


##############################################################################
## ComponentGroup object

class ComponentGroup(object):
    '''A group of components in the RT system.'''

    def __init__(self, group_id='', members=[]):
        '''Constructor.

        @param group_id ID of the group.
        @type group_id str
        @param members Members of the group. At least one must be present.
        @type members list

        '''
        validate_attribute(group_id, 'component_group.groupID',
                           expected_type=[str, unicode], required=False)
        self._group_id = group_id
        validate_attribute(members, 'component_group.Members',
                           expected_type=list, required=False)
        self._members = members

    def __str__(self):
        result = 'Group ID: {0}\n'.format(self.group_id)
        if self.members:
            result += 'Members:\n'
            for m in self.members:
                result += '  {0}\n'.format(m)
        return result[:-1] # Lop off the last new line

    @property
    def group_id(self):
        '''The ID used to distinguish this group in the RT system.'''
        return self._group_id

    @group_id.setter
    def group_id(self, group_id):
        validate_attribute(group_id, 'component_group.groupID',
                           expected_type=[str, unicode], required=True)
        self._group_id = group_id

    @property
    def members(self):
        '''A list of the components in the group.

        At least one must be present.

        '''
        return self._members

    @members.setter
    def members(self, members):
        validate_attribute(members, 'component_group.Members',
                           expected_type=list, required=True)
        self._members = members

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a component group into
        this object.

        '''
        self.group_id = node.getAttributeNS(RTS_NS, 'groupId')
        self._members = []
        for c in node.getElementsByTagNameNS(RTS_NS, 'Members'):
            self._members.append(TargetComponent().parse_xml_node(c))
        return self

    def parse_yaml(self, node):
        '''Parse a YAML specification of a component group into this
        object.

        '''
        self.group_id = y['groupId']
        self._members = []
        if 'members' in y:
            for m in y.get('members'):
                self._members.append(TargetComponent().parse_yaml(m))
        return self

    def save_xml(self, doc, element):
        '''Save this component group into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'groupID', self.group_id)
        for m in self.members:
            new_element = doc.createElementNS(RTS_NS, RTS_NS_S + 'Members')
            m.save_xml(doc, new_element)
            element.appendChild(new_element)

    def to_dict(self):
        '''Save this component group to a dictionary.'''
        d = {'groupId': self.group_id}
        members = []
        for m in self.members:
            members.append(m.to_dict())
        if members:
            d['members'] = members
        return d



# vim: tw=79

