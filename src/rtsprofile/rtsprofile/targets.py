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

File: targets.py

Objects representing targets. These can be target components, execution
contexts or ports.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S, RTS_EXT_NS, RTS_EXT_NS_S, \
                       RTS_EXT_NS_YAML
from rtsprofile.utils import get_direct_child_elements_xml, \
                             parse_properties_xml, properties_to_xml, \
                             validate_attribute


##############################################################################
## TargetComponent object

class TargetComponent(object):
    '''Stores enough information to uniquely identify a component in the RT
    system. Used to specify target components, for example the components
    participating in a group or running in an execution context, or the
    execution order of components.

    '''

    def __init__(self, component_id='', instance_name=''):
        '''Constructor.

        @param component_id The ID of the target component.
        @type component_id str
        @param instance_name The instance name of the target component.
        @type instance_name str

        '''
        validate_attribute(component_id, 'target_component.componentID',
                           expected_type=[str, unicode], required=False)
        self._component_id = component_id
        validate_attribute(instance_name, 'target_component.instanceName',
                           expected_type=[str, unicode], required=False)
        self._instance_name = instance_name
        self._properties = {}

    def __str__(self):
        result = 'Component ID: {0}\nInstance name: {1}\n'.format(\
                self.component_id, self.instance_name)
        if self.properties:
            result += 'Properties:\n'
            for p in self.properties:
                result += '  {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def component_id(self):
        '''The ID of the target component.

        RT components can be uniquely identified using the component ID and the
        instance name.

        '''
        return self._component_id

    @component_id.setter
    def component_id(self, component_id):
        validate_attribute(component_id, 'target_component.componentID',
                           expected_type=[str, unicode], required=True)
        self._component_id = component_id

    @property
    def instance_name(self):
        '''The instance name of the target component.

        RT components can be uniquely identified using the component ID and the
        instance name.

        '''
        return self._instance_name

    @instance_name.setter
    def instance_name(self, instance_name):
        validate_attribute(instance_name, 'target_component.instanceName',
                           expected_type=[str, unicode], required=True)
        self._instance_name = instance_name

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties, 'target_component.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a target component into
        this object.

        '''
        self.component_id = node.getAttributeNS(RTS_NS, 'componentId')
        self.instance_name = node.getAttributeNS(RTS_NS, 'instanceName')
        for c in node.getElementsByTagNameNS(RTS_EXT_NS, 'Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a target component into this
        object.

        '''
        self.component_id = y['componentId']
        self.instance_name = y['instanceName']
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y.get(RTS_EXT_NS_YAML + 'properties'):
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value
        return self

    def save_xml(self, doc, element):
        '''Save this target component into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'componentId',
                               self.component_id)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'instanceName',
                               self.instance_name)
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    def to_dict(self):
        '''Save this target component into a dictionary.'''
        d = {'componentId': self.component_id,
                'instanceName': self.instance_name}
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
## TargetPort object

class TargetPort(TargetComponent):
    '''Stores enough information to uniquely identify a port on a component in
    the RT system. Used to specify target ports in connections.

    '''

    def __init__(self, component_id='', instance_name='', port_name=''):
        '''Constructor. See also the @ref TargetComponent constructor.

        @param port_name The name of the target port.
        @type port_name str

        '''
        super(TargetPort, self).__init__(component_id, instance_name)
        validate_attribute(port_name, 'target_port.portName',
                           expected_type=[str, unicode], required=False)
        self._port_name = port_name

    def __str__(self):
        return TargetComponent.__str__(self) + '\nPort name: {0}'.format(\
                self.port_name)

    @property
    def port_name(self):
        '''The ID of the target port.'''
        return self._port_name

    @port_name.setter
    def port_name(self, port_name):
        validate_attribute(port_name, 'target_port.portName',
                           expected_type=[str, unicode], required=True)
        self._port_name = port_name

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a target port into this
        object.

        '''
        super(TargetPort, self).parse_xml_node(node)
        self.port_name = node.getAttributeNS(RTS_NS, 'portName')
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a target port into this object.'''
        super(TargetPort, self).parse_yaml(y)
        self.port_name = y['portName']
        return self

    def save_xml(self, doc, element):
        '''Save this target port into an xml.dom.Element object.'''
        super(TargetPort, self).save_xml(doc, element)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'portName', self.port_name)

    def to_dict(self):
        '''Save this target port into a dictionary.'''
        d = super(TargetPort, self).to_dict()
        d['portName'] = self.port_name
        return d


##############################################################################
## TargetExecutionContext object

class TargetExecutionContext(TargetComponent):
    '''Stores enough information to uniquely identify an execution context in
    the RT system. Used to specify target execution contexts of components that
    are used to manage execution order, execution conditions, and so on. An
    RT component may hold multiple execution contexts. The execution context
    concerned must be specified when activating, shutting down, etc an RT
    component.

    '''

    def __init__(self, component_id='', instance_name='', id=''):
        '''Constructor. See also the @ref TargetComponent constructor.

        @param id The ID of the target execution context.
        @type id str

        '''
        super(TargetExecutionContext, self).__init__(component_id,
                                                     instance_name)
        validate_attribute(id, 'target_executioncontext.id',
                           expected_type=[str, unicode], required=False)
        self._id = id
        self._properties = {}

    def __str__(self):
        result = TargetComponent.__str__(self) + '\nID: {0}\n'.format(self.id)
        if self.properties:
            result += 'Properties:\n'
            for p in self.properties:
                result += '  {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def id(self):
        '''The ID of the target execution context.'''
        return self._id

    @id.setter
    def id(self, id):
        validate_attribute(id, 'target_executioncontext.id',
                           expected_type=[str, unicode], required=True)
        self._id = id

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties,
                           'target_executioncontext.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a target execution context
        into this object.

        '''
        super(TargetExecutionContext, self).parse_xml_node(node)
        if node.hasAttributeNS(RTS_NS, 'id'):
            self.id = node.getAttributeNS(RTS_NS, 'id')
        else:
            self.id = ''
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a target execution context into this
        object.

        '''
        super(TargetExecutionContext, self).parse_yaml(y)
        if 'id' in y:
            self.id = y['id']
        else:
            self.id = ''
        return self

    def save_xml(self, doc, element):
        '''Save this target execution context into an xml.dom.Element
        object.

        '''
        super(TargetExecutionContext, self).save_xml(doc, element)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'id', self.id)

    def to_dict(self):
        '''Save this target execution context into a dictionary.'''
        d = super(TargetExecutionContext, self).to_dict()
        d['id'] = self.id
        return d


# vim: tw=79

