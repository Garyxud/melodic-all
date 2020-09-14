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

File: config_set.py

Objects representing a component's configuration set.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S
from rtsprofile.exec_context import ExecutionContext
from rtsprofile.utils import indent_string, validate_attribute


##############################################################################
## ConfigurationSet object

class ConfigurationSet(object):
    '''Represents a configuration set.

    A configuration set is a collection of configuration parameters. An RT
    Component can have multiple configuration sets.

    '''

    def __init__(self, id=''):
        '''Constructor.

        @param id The configuration set ID.
        @type id str

        '''
        validate_attribute(id, 'configuration_set.id',
                           expected_type=[str, unicode], required=False)
        self._id = id
        self._config_data = []

    def __str__(self):
        result = 'ID: {0}\n'.format(self.id)
        if self.configuration_data:
            result += 'Configuration data:\n'
            for d in self.configuration_data:
                result += '{0}\n'.format(indent_string(str(d)))
        return result[:-1] # Lop off the last new line

    @property
    def configuration_data(self):
        '''The configuration parameters contained in this set.

        May be an empty list if this set has no parameters.

        '''
        return self._config_data

    @configuration_data.setter
    def configuration_data(self, configuration_data):
        validate_attribute(configuration_data,
                           'configuration_set.ConfigurationData',
                           expected_type=list)
        self._config_data = configuration_data

    @property
    def id(self):
        '''The configuration set ID.

        Used to distinguish this configuration set from others in the RT
        Component.

        '''
        return self._id

    @id.setter
    def id(self, id):
        validate_attribute(id, 'configuration_set.id',
                           expected_type=[str, unicode], required=True)
        self._id = id

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a configuration set into
        this object.

        '''
        self.id = node.getAttributeNS(RTS_NS, 'id')
        self._config_data = []
        for d in node.getElementsByTagNameNS(RTS_NS, 'ConfigurationData'):
            self._config_data.append(ConfigurationData().parse_xml_node(d))
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a configuration set into this
        object.

        '''
        self.id = y['id']
        self._config_data = []
        if 'configurationData' in y:
            for d in y.get('configurationData'):
                self._config_data.append(ConfigurationData().parse_yaml(d))
        return self

    def save_xml(self, doc, element):
        '''Save this configuration set into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'id', self.id)
        for c in self._config_data:
            new_element = doc.createElementNS(RTS_NS,
                                              RTS_NS_S + 'ConfigurationData')
            c.save_xml(doc, new_element)
            element.appendChild(new_element)

    def to_dict(self):
        '''Save this configuration set into a dictionary.'''
        d = {'id': self.id}
        data = []
        for c in self._config_data:
            data.append(c.to_dict())
        if data:
            d['configurationData'] = data
        return d


##############################################################################
## ConfigurationData object

class ConfigurationData(object):
    '''Represents an individual configuration parameter and its value.'''

    def __init__(self, name='', data=''):
        '''Constructor.

        @param name The name of the parameter.
        @type name str
        @param data The parameter's value, if any.
        @type data str

        '''
        validate_attribute(name, 'configuration_set.name',
                           expected_type=[str, unicode], required=False)
        self._name = name
        validate_attribute(data, 'configuration_set.data',
                           expected_type=[str, unicode], required=False)
        self._data = data

    def __str__(self):
        return '{0}: {1}'.format(self.name, self.data)

    @property
    def data(self):
        '''The value of this configuration parameter.

        May be an empty string if the parameter has no value.

        '''
        return self._data

    @data.setter
    def data(self, data):
        validate_attribute(data, 'configuration_set.data',
                           expected_type=[str, unicode], required=False)
        self._data = data

    @property
    def name(self):
        '''The name of this configuration parameter.

        Used as the parameter's key in the configuration set object.

        '''
        return self._name

    @name.setter
    def name(self, name):
        validate_attribute(name, 'configuration_set.name',
                           expected_type=[str, unicode], required=True)
        self._name = name

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a configuration data into
        this object.

        '''
        self.name = node.getAttributeNS(RTS_NS, 'name')
        if node.hasAttributeNS(RTS_NS, 'data'):
            self.data = node.getAttributeNS(RTS_NS, 'data')
        else:
            self.data = ''
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a configuration data into this
        object.

        '''
        self.name = y['name']
        if 'data' in y:
            self.data = y['data']
        else:
            self.data = ''
        return self

    def save_xml(self, doc, element):
        '''Save this configuration data into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'name', self.name)
        if self.data:
            element.setAttributeNS(RTS_NS, RTS_NS_S + 'data', self.data)

    def to_dict(self):
        '''Save this configuration data into a dictionary.'''
        d = {'name': self.name}
        if self.data:
            d['data'] = self.data
        return d


# vim: tw=79

