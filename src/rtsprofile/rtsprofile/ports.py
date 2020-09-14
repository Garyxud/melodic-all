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

File: ports.py

Objects representing ports in a component.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S, RTS_EXT_NS, RTS_EXT_NS_S, \
                       RTS_EXT_NS_YAML
from rtsprofile.utils import get_direct_child_elements_xml, \
                             parse_properties_xml, validate_attribute


##############################################################################
## DataPort object

class DataPort(object):
    '''Represents a data port of a component, as specified in a
    ConnectorProfile.

    '''

    def __init__(self, name='', comment='', visible=True):
        '''Constructor.

        @param name Name of the port.
        @type name str
        @param comment A comment about the port.
        @type comment str
        @param visible If this port is visible in graphical displays.
        @type visible bool

        '''
        validate_attribute(name, 'dataPort.name',
                           expected_type=[str, unicode], required=False)
        self._name = name
        validate_attribute(comment, 'component.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment
        validate_attribute(visible, 'component.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible
        self._properties = {}

    def __str__(self):
        result = 'Name: {0}\n'.format(self.name)
        if self.comment:
            result += '  Comment: {0}\n'.format(self.comment)
        result += '  Visible: {0}\n'.format(self.visible)
        if self.properties:
            result += '  Properties:\n'
            for p in self.properties:
                result += '    {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def name(self):
        '''The name of this data port.

        This name is used in connector profiles to identify the port.

        '''
        return self._name

    @name.setter
    def name(self, name):
        validate_attribute(name, 'dataPort.name',
                           expected_type=[str, unicode], required=True)
        self._name = name

    @property
    def comment(self):
        '''Comment about the data port.

        A brief comment about the data port. May or may not be displayed in
        other tools. May be empty.

        Part of the extended profile.

        '''
        return self._comment

    @comment.setter
    def comment(self, comment):
        validate_attribute(comment, 'dataPort.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment

    @property
    def visible(self):
        '''Display the port in graphical tools.

        This value controls whether graphical tools will display this port or
        not.

        Part of the extended profile.

        '''
        return self._visible

    @visible.setter
    def visible(self, visible):
        validate_attribute(visible, 'dataPort.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties, 'dataPort.ext.Properties',
                           expected_type=list, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a data port into this
        object.

        '''

        self.name = node.getAttributeNS(RTS_NS, 'name')
        self.comment = node.getAttributeNS(RTS_EXT_NS, 'comment')
        if node.hasAttributeNS(RTS_EXT_NS, 'visible'):
            visible = node.getAttributeNS(RTS_EXT_NS, 'visible')
            if visible.lower() == 'true' or visible == '1':
                self.visible = True
            else:
                self.visible = False
        for c in get_direct_child_elements_xml(node, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a data port into this object.'''
        self.name = y['name']
        if RTS_EXT_NS_YAML + 'comment' in y:
            self.comment = y[RTS_EXT_NS_YAML + 'comment']
        self.visible = False
        if RTS_EXT_NS_YAML + 'visible' in y:
            visible = y.get(RTS_EXT_NS_YAML + 'visible')
            if visible == True or visible == 'true' or visible == 'True':
                self.visible = True
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y.get(RTS_EXT_NS_YAML + 'properties'):
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value
        return self

    def save_xml(self, doc, element):
        '''Save this data port into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'name', self.name)
        if self.comment:
            element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'comment',
                                   self.comment)
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'visible',
                               str(self.visible).lower())
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    def to_dict(self):
        '''Save this data port into a dictionary.'''
        d = {'name': self.name,
                RTS_EXT_NS_YAML + 'visible': str(self.visible).lower()}
        if self.comment:
            d[RTS_EXT_NS_YAML + 'comment'] = self.comment
        props = []
        for name in self.properties:
            p = {'name': name}
            if self.properties[name]:
                p['value'] = str(self.properties[name])
            props.append(p)
        if props:
            d[RTS_EXT_NS_YAML + 'properties'] = props
        return d


##############################################################################
## ServicePort object

class ServicePort(object):
    '''Represents a service port of a component, as specified in a
    ConnectorProfile.

    '''

    def __init__(self, name='', comment='', visible=True):
        '''Constructor.

        @param name Name of the port.
        @type name str
        @param comment A comment about the port.
        @type comment str
        @param visible If this port is visible in graphical displays.
        @type visible bool

        '''
        validate_attribute(name, 'serviceport.name',
                           expected_type=[str, unicode], required=False)
        self._name = name
        validate_attribute(comment, 'component.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment
        validate_attribute(visible, 'component.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible
        self._properties = {}

    def __str__(self):
        result = 'Name: {0}\n'.format(self.name)
        if self.comment:
            result += '  Comment: {0}\n'.format(self.comment)
        result += '  Visible: {0}\n'.format(self.visible)
        if self.properties:
            result += '  Properties:\n'
            for p in self.properties:
                result += '    {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def name(self):
        '''The name of this service port.

        This name is used in connector profiles to identify the port.

        '''
        return self._name

    @name.setter
    def name(self, name):
        validate_attribute(name, 'serviceport.name',
                           expected_type=[str, unicode], required=True)
        self._name = name

    @property
    def comment(self):
        '''Comment about the service port.

        A brief comment about the service port. May or may not be displayed in
        other tools. May be empty.

        Part of the extended profile.

        '''
        return self._comment

    @comment.setter
    def comment(self, comment):
        validate_attribute(comment, 'serviceport.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment

    @property
    def visible(self):
        '''Display the port in graphical tools.

        This value controls whether graphical tools will display this port or
        not.

        Part of the extended profile.

        '''
        return self._visible

    @visible.setter
    def visible(self, visible):
        validate_attribute(visible, 'serviceport.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties, 'serviceport.ext.Properties',
                           expected_type=list, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a service port into this
        object.

        '''

        self.name = node.getAttributeNS(RTS_NS, 'name')
        self.comment = node.getAttributeNS(RTS_EXT_NS, 'comment')
        if node.hasAttributeNS(RTS_EXT_NS, 'visible'):
            visible = node.getAttributeNS(RTS_EXT_NS, 'visible')
            if visible.lower() == 'true' or visible == '1':
                self.visible = True
            else:
                self.visible = False
        for c in get_direct_child_elements_xml(node, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a service port into this object.'''
        self.name = y['name']
        if RTS_EXT_NS_YAML + 'comment' in y:
            self.comment = y[RTS_EXT_NS_YAML + 'comment']
        self.visible = False
        if RTS_EXT_NS_YAML + 'visible' in y:
            visible = y.get(RTS_EXT_NS_YAML + 'visible')
            if visible == True or visible == 'true' or visible == 'True':
                self.visible = True
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y.get(RTS_EXT_NS_YAML + 'properties'):
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value
        return self

    def save_xml(self, doc, element):
        '''Save this service port into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'name', self.name)
        if self.comment:
            element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'comment',
                                   self.comment)
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'visible',
                               str(self.visible).lower())
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    def to_dict(self):
        '''Save this service port into a dictionary.'''
        d = {'name': self.name,
                RTS_EXT_NS_YAML + 'visible': str(self.visible).lower()}
        if self.comment:
            d[RTS_EXT_NS_YAML + 'comment'] = self.comment
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

