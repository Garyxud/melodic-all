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

File: exec_context.py

Object representing a component's execution context.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S, RTS_EXT_NS, RTS_EXT_NS_S, \
                       RTS_EXT_NS_YAML
from rtsprofile.participant import Participant
from rtsprofile.utils import get_direct_child_elements_xml, \
                             indent_string, parse_properties_xml, \
                             properties_to_xml, validate_attribute


##############################################################################
## ExecutionContext object

class ExecutionContext(object):
    '''Represents an execution context being used in the RT system.'''

    def __init__(self, id='', kind='', rate=0.0):
        '''Constructor.

        @param id The ID of this execution context.
        @type id str
        @param kind The action execution type used by this context.
        @type kind str
        @param rate The execution rate of this context, if it is periodic.
        @type float

        '''
        validate_attribute(id, 'execution_context.id',
                           expected_type=[str, unicode], required=False)
        self._id = id
        validate_attribute(kind, 'execution_context.kind',
                           expected_type=[str, unicode], required=False)
        self._kind = kind
        validate_attribute(rate, 'execution_context.rate',
                           expected_type=[int, float], required=False)
        self._rate = rate
        self._participants = []
        self._properties = {}

    def __str__(self):
        result = 'ID: {0}\nKind: {1}\nRate: {2}\n'.format(self.id, self.kind,
                                                          self.rate)
        if self.participants:
            result += 'Participants:\n'
            for p in self.participants:
                result += '{0}\n'.format(indent_str(str(p)))
        if self.properties:
            result += 'Properties:\n'
            for p in self.properties:
                result += '  {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def id(self):
        '''The ID used to identify this execution context.'''
        return self._id

    @id.setter
    def id(self, id):
        validate_attribute(id, 'execution_context.id',
                           expected_type=[str, unicode], required=True)
        self._id = id

    @property
    def kind(self):
        '''The action execution type used by this context.

        Valid values are supposed to be in the specification appendix, but they
        aren't. The best way to find them is to create a system with
        RTSystemEditor and look at the XML. A common valid value is
        PeriodicExecutionContext.

        '''
        return self._kind

    @kind.setter
    def kind(self, kind):
        validate_attribute(kind, 'execution_context.kind',
                           expected_type=[str, unicode], required=True)
        self._kind = kind

    @property
    def participants(self):
        '''The components participating in this execution context.

        An ordered list. May be an empty list if no components are
        participating in this context.

        '''
        return self._participants

    @participants.setter
    def participants(self, participants):
        validate_attribute(participants, 'execution_context.participants',
                           expected_type = list)
        self._participants = participants

    @property
    def rate(self):
        '''The execution rate of this context, if it has one, in Hertz.

        This value is only used if the execution context is periodic for a data
        flow component.

        '''
        return self._rate

    @rate.setter
    def rate(self, rate):
        validate_attribute(rate, 'execution_context.rate',
                           expected_type=[int, float], required=False)
        self._rate = rate

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties, 'execution_context.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing an execution context into
        this object.

        '''
        self.id = node.getAttributeNS(RTS_NS, 'id')
        self.kind = node.getAttributeNS(RTS_NS, 'kind')
        if node.hasAttributeNS(RTS_NS, 'rate'):
            self.rate = float(node.getAttributeNS(RTS_NS, 'rate'))
        else:
            self.rate = 0.0
        self._participants = []
        for c in node.getElementsByTagNameNS(RTS_NS, 'Participants'):
            self._participants.append(TargetComponent().parse_xml_node(c))
        for c in get_direct_child_elements_xml(node, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value
        return self

    def parse_yaml(self, y):
        '''Parse a YAML spefication of an execution context into this
        object.

        '''
        self.id = y['id']
        self.kind = y['kind']
        if 'rate' in y:
            self.rate = float(y['rate'])
        else:
            self.rate = 0.0
        self._participants = []
        if 'participants' in y:
            for p in y.get('participants'):
                self._participants.append(TargetComponent().parse_yaml(p))
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y.get(RTS_EXT_NS_YAML + 'properties'):
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value
        return self

    def save_xml(self, doc, element):
        '''Save this execution context into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'id', self.id)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'kind', self.kind)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'rate', str(self.rate))
        for p in self.participants:
            new_element = doc.createElementNS(RTS_NS,
                                              RTS_NS_S + 'Participants')
            p.save_xml(doc, new_element)
            element.appendChild(new_element)
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    def to_dict(self):
        '''Save this execution context into a dictionary.'''
        d = {'id': self.id,
                'kind': self.kind}
        if self.rate != 0.0:
            d['rate'] = self.rate
        participants = []
        for p in self.participants:
            participants.append(p.to_dict())
        if participants:
            d['participants'] = participants
        props = []
        for name in self.properties:
            p = {'name': name}
            if self.properties[name]:
                p['value'] = str(self.properties[name])
            props.append(p)
        if props:
            d[RTS_EXT_NS_YAML + 'properties'] = props
        return d


# vim: tw=79

