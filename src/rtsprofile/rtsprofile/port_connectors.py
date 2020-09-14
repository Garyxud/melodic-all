# -*- Python -*- # -*- coding: utf-8 -*-

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

File: data_port_connector.py

Objects representing connections between data and service ports.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S, RTS_EXT_NS, RTS_EXT_NS_S, \
                       RTS_EXT_NS_YAML
from rtsprofile.exceptions import InvalidDataPortConnectorNodeError, \
                                  InvalidServicePortConnectorNodeError
from rtsprofile.targets import TargetPort
from rtsprofile.utils import get_direct_child_elements_xml, \
                             indent_string, parse_properties_xml, \
                             properties_to_xml, validate_attribute


##############################################################################
## DataPortConnector object

class DataPortConnector(object):
    '''Represents a connection between data ports.'''

    def __init__(self, connector_id='', name='', data_type='',
            interface_type='', data_flow_type='', subscription_type='',
            push_interval=0.0, source_data_port=TargetPort(),
            target_data_port=TargetPort(), comment='', visible=True):
        '''Constructor.

        @param connector_id ID of the connector.
        @type connector_id str
        @param name Name of the connector.
        @type name str
        @param data_type Data type that this connector transports.
        @type data_type str
        @param interface_type Interface type of the connected ports.
        @type interface_type str
        @param data_flow_type Type of data flow between the ports.
        @type data_flow_type str
        @param subscription_type Type of subscription between the ports.
        @type subscription_type str
        @param push_interval Rate at which data is sent between the ports.
        @type push_interval float
        @param source_data_port The source port in the connection.
        @type source_data_port TargetPort
        @param target_data_port The target port in the connection.
        @type target_data_port TargetPort
        @param comment A comment about the port connector.
        @type comment str
        @param visible If this connector is visible in graphical displays.
        @type visible bool

        '''
        validate_attribute(connector_id, 'dataport_connector.connectorID',
                           expected_type=[str, unicode], required=False)
        self._connector_id = connector_id
        validate_attribute(name, 'dataport_connector.name',
                           expected_type=[str, unicode], required=False)
        self._name = name
        validate_attribute(data_type, 'dataport_connector.dataType',
                           expected_type=[str, unicode], required=False)
        self._data_type = data_type
        validate_attribute(interface_type, 'dataport_connector.interfaceType',
                           expected_type=[str, unicode], required=False)
        self._interface_type = interface_type
        validate_attribute(data_flow_type, 'dataport_connector.dataflowType',
                           expected_type=[str, unicode], required=False)
        self._data_flow_type = data_flow_type
        validate_attribute(subscription_type,
                           'dataport_connector.subscriptionType',
                           expected_type=[str, unicode], required=False)
        self._subscription_type = subscription_type
        validate_attribute(push_interval, 'dataport_connector.pushInterval',
                           expected_type=[int, float], required=False)
        self._push_interval = push_interval
        validate_attribute(source_data_port,
                           'dataport_connector.sourceDataPort',
                           expected_type=TargetPort, required=False)
        self._source_data_port = source_data_port
        validate_attribute(target_data_port,
                           'dataport_connector.targetDataPort',
                           expected_type=TargetPort, required=False)
        self._target_data_port = target_data_port
        validate_attribute(comment, 'component.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment
        validate_attribute(visible, 'component.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible
        self._properties = {}

    def __str__(self):
        result = 'Name: {1}\n  Connector ID: {0}\n  Data type: {2}\n  \
Interface type: {3}\n  Data flow type: {4}\n  Subscription type: {5}\n  Push \
interval: {6}\n  Source data port:\n{7}\n  Target data port:\n{8}\n'.format(\
            self.connector_id, self.name, self.data_type, self.interface_type,
            self.data_flow_type, self.subscription_type, self.push_interval,
            indent_string(str(self.source_data_port), num_spaces=4),
            indent_string(str(self.target_data_port), num_spaces=4))
        if self.comment:
            result += 'Comment: {0}\n'.format(self.comment)
        result += 'Visible: {0}\n'.format(self.visible)
        if self.properties:
            result += 'Properties:\n'
            for p in self.properties:
                result += '  {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def connector_id(self):
        '''The ID of the connector used to distinguish it in the RT system.'''
        return self._connector_id

    @connector_id.setter
    def connector_id(self, connector_id):
        validate_attribute(connector_id, 'dataport_connector.connectorID',
                           expected_type=[str, unicode], required=True)
        self._connector_id = connector_id

    @property
    def name(self):
        '''The name of the connector.'''
        return self._name

    @name.setter
    def name(self, name):
        validate_attribute(name, 'dataport_connector.name',
                           expected_type=[str, unicode], required=True)
        self._name = name

    @property
    def data_type(self):
        '''Data type that this connector transports.'''
        return self._data_type

    @data_type.setter
    def data_type(self, data_type):
        validate_attribute(data_type, 'dataport_connector.dataType',
                           expected_type=[str, unicode], required=True)
        self._data_type = data_type

    @property
    def interface_type(self):
        '''Interface type of the connection.

        As specified when the RT system is created. Dependent on what the RT
        Middleware used to execute the RT system supports.

        '''
        return self._interface_type

    @interface_type.setter
    def interface_type(self, interface_type):
        validate_attribute(interface_type, 'dataport_connector.interfaceType',
                           expected_type=[str, unicode], required=True)
        self._interface_type = interface_type

    @property
    def data_flow_type(self):
        '''Type of data flow between the ports.

        As specified when the RT system is created. Dependent on what the RT
        Middleware used to execute the RT system supports.

        '''
        return self._data_flow_type

    @data_flow_type.setter
    def data_flow_type(self, data_flow_type):
        validate_attribute(data_flow_type, 'dataport_connector.dataflowType',
                           expected_type=[str, unicode], required=True)
        self._data_flow_type = data_flow_type

    @property
    def subscription_type(self):
        '''Type of the subscription between the ports.

        As specified when the RT system is created. Only used when @ref
        data_flow_type is set to PUSH. Dependent on what the RT Middleware used
        to execute the RT system supports.

        '''
        return self._subscription_type

    @subscription_type.setter
    def subscription_type(self, subscription_type):
        validate_attribute(subscription_type,
                           'dataport_connector.subscriptionType',
                           expected_type=[str, unicode], required=False)
        self._subscription_type = subscription_type

    @property
    def push_interval(self):
        '''Rate at which data is sent between ports.

        As specified when the RT system is created.

        '''
        return self._push_interval

    @push_interval.setter
    def push_interval(self, push_interval):
        validate_attribute(push_interval, 'dataport_connector.pushInterval',
                           expected_type=[int, float], required=False)
        self._push_interval = push_interval

    @property
    def source_data_port(self):
        '''The source port in the connection.'''
        return self._source_data_port

    @source_data_port.setter
    def source_data_port(self, source_data_port):
        validate_attribute(source_data_port,
                           'dataport_connector.sourceDataPort',
                           expected_type=TargetPort, required=True)
        self._source_data_port = source_data_port

    @property
    def target_data_port(self):
        '''The target port in the connection.'''
        return self._target_data_port

    @target_data_port.setter
    def target_data_port(self, target_data_port):
        validate_attribute(target_data_port,
                           'dataport_connector.targetDataPort',
                           expected_type=TargetPort, required=True)
        self._target_data_port = target_data_port

    @property
    def comment(self):
        '''Comment about the connector.

        A brief comment about the connector. May or may not be displayed in
        other tools. May be empty.

        Part of the extended profile.

        '''
        return self._comment

    @comment.setter
    def comment(self, comment):
        validate_attribute(comment, 'dataport_connector.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment

    @property
    def visible(self):
        '''Display the connector in graphical tools.

        This value controls whether graphical tools will display this connector
        or not.

        Part of the extended profile.

        '''
        return self._visible

    @visible.setter
    def visible(self, visible):
        validate_attribute(visible, 'dataport_connector.ext.visible',
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
        validate_attribute(properties, 'dataport_connector.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a data connector into this
        object.

        '''
        self.connector_id = node.getAttributeNS(RTS_NS, 'connectorId')
        self.name = node.getAttributeNS(RTS_NS, 'name')
        self.data_type = node.getAttributeNS(RTS_NS, 'dataType')
        self.interface_type = node.getAttributeNS(RTS_NS, 'interfaceType')
        self.data_flow_type = node.getAttributeNS(RTS_NS, 'dataflowType')
        if node.hasAttributeNS(RTS_NS, 'subscriptionType'):
            self.subscription_type = node.getAttributeNS(RTS_NS,
                                                         'subscriptionType')
        else:
            self.subscription_type = ''
        if node.hasAttributeNS(RTS_NS, 'pushInterval'):
            self.push_interval = float(node.getAttributeNS(RTS_NS,
                                                           'pushInterval'))
        else:
            self.push_interval = 0.0
        self.comment = node.getAttributeNS(RTS_EXT_NS, 'comment')
        if node.hasAttributeNS(RTS_EXT_NS, 'visible'):
            visible = node.getAttributeNS(RTS_EXT_NS, 'visible')
            if visible == 'true' or visible == '1':
                self.visible = True
            else:
                self.visible = False

        if node.getElementsByTagNameNS(RTS_NS, 'sourceDataPort').length != 1:
            raise InvalidDataPortConnectorNodeError
        self.source_data_port = TargetPort().parse_xml_node(\
                node.getElementsByTagNameNS(RTS_NS, 'sourceDataPort')[0])
        if node.getElementsByTagNameNS(RTS_NS, 'targetDataPort').length != 1:
            raise InvalidDataPortConnectorNodeError
        self.target_data_port = TargetPort().parse_xml_node(\
                node.getElementsByTagNameNS(RTS_NS, 'targetDataPort')[0])
        for c in get_direct_child_elements_xml(node, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a data port connector into this
        object.

        '''
        self.connector_id = y['connectorId']
        self.name = y['name']
        self.data_type = y['dataType']
        self.interface_type = y['interfaceType']
        self.data_flow_type = y['dataflowType']
        if 'subscriptionType' in y:
            self.subscription_type = y['subscriptionType']
        else:
            self.subscription_type = ''
        if 'pushInterval' in y:
            self.push_interval = float(y['pushInterval'])
        else:
            self.push_interval = 0.0
        if RTS_EXT_NS_YAML + 'comment' in y:
            self.comment = y[RTS_EXT_NS_YAML + 'comment']
        else:
            self.comment = ''
        self.visible = False
        if RTS_EXT_NS_YAML + 'visible' in y:
            visible = y[RTS_EXT_NS_YAML + 'visible']
            if visible == 'true' or visible == '1':
                self.visible = True
        if not 'sourceDataPort' in y:
            raise InvalidDataPortConnectorNodeError
        self.source_data_port = \
                TargetPort().parse_yaml(y['sourceDataPort'])
        if not 'targetDataPort' in y:
            raise InvalidDataPortConnectorNodeError
        self.target_data_port = \
                TargetPort().parse_yaml(y['targetDataPort'])
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y[RTS_EXT_NS_YAML + 'properties']:
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value
        return self

    def save_xml(self, doc, element):
        '''Save this data port into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'connectorId',
                               self.connector_id)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'name', self.name)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'dataType', self.data_type)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'interfaceType',
                               self.interface_type)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'dataflowType',
                               self.data_flow_type)
        if self.subscription_type:
            element.setAttributeNS(RTS_NS, RTS_NS_S + 'subscriptionType',
                                   self.subscription_type)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'pushInterval',
                               str(self.push_interval))
        if self.comment:
            element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'comment',
                                   self.comment)
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'visible',
                               str(self.visible).lower())
        new_element = doc.createElementNS(RTS_NS, RTS_NS_S + 'sourceDataPort')
        self.source_data_port.save_xml(doc, new_element)
        element.appendChild(new_element)
        new_element = doc.createElementNS(RTS_NS, RTS_NS_S + 'targetDataPort')
        self.target_data_port.save_xml(doc, new_element)
        element.appendChild(new_element)
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    def to_dict(self):
        '''Save this data port connector into a dictionary.'''
        d = {'connectorId': self.connector_id,
                'name': self.name,
                'dataType': self.data_type,
                'interfaceType': self.interface_type,
                'dataflowType': self.data_flow_type,
                RTS_EXT_NS_YAML + 'visible': str(self.visible).lower(),
                'sourceDataPort': self.source_data_port.to_dict(),
                'targetDataPort': self.target_data_port.to_dict()}
        if self.subscription_type:
            d['subscriptionType'] = self.subscription_type
        if self.push_interval:
            d['pushInterval'] = self.push_interval
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
## ServicePortConnector object

class ServicePortConnector(object):
    '''Represents a connection between service ports.'''

    def __init__(self, connector_id='', name='', trans_method='',
            source_service_port=TargetPort(),
            target_service_port=TargetPort(), comment='', visible=True):
        '''Constructor.

        @param connector_id ID of the connector.
        @type connector_id str
        @param name Name of the connector.
        @type name str
        @param trans_method Transport method used by the ports.
        @type trans_method str
        @param source_service_port The source port in the connection.
        @type source_service_port TargetPort
        @param target_service_port The target port in the connection.
        @type target_service_port TargetPort
        @param comment A comment about the port connector.
        @type comment str
        @param visible If this connector is visible in graphical displays.
        @type visible bool

        '''
        validate_attribute(connector_id, 'serviceport_connector.connectorID',
                           expected_type=[str, unicode], required=False)
        self._connector_id = connector_id
        validate_attribute(name, 'serviceport_connector.name',
                           expected_type=[str, unicode], required=False)
        self._name = name
        validate_attribute(trans_method, 'serviceport_connector.transMethod',
                           expected_type=[str, unicode], required=False)
        self._trans_method = trans_method
        validate_attribute(source_service_port,
                           'serviceport_connector.sourceServicePort',
                           expected_type=TargetPort, required=True)
        self._source_service_port = source_service_port
        validate_attribute(target_service_port,
                           'serviceport_connector.targetServicePort',
                           expected_type=TargetPort, required=True)
        self._target_service_port = target_service_port
        validate_attribute(comment, 'component.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment
        validate_attribute(visible, 'component.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible
        self._properties = {}


    def __str__(self):
        result = 'Name: {1}\n  Connector ID: {0}\n  Trans method: {2}\n  \
Source data port:\n{3}\n  Target data port:\n{4}'.format(self.connector_id,
            self.name, self.trans_method,
            indent_string(str(self.source_service_port), num_spaces=4),
            indent_string(str(self.target_service_port), num_spaces=4))
        if self.comment:
            result += 'Comment: {0}\n'.format(self.comment)
        result += 'Visible: {0}\n'.format(self.visible)
        if self.properties:
            result += 'Properties:\n'
            for p in self.properties:
                result += '  {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    @property
    def connector_id(self):
        '''The ID of the connector used to distinguish it in the RT system.'''
        return self._connector_id

    @connector_id.setter
    def connector_id(self, connector_id):
        validate_attribute(connector_id, 'serviceport_connector.connectorID',
                           expected_type=[str, unicode], required=True)
        self._connector_id = connector_id

    @property
    def name(self):
        '''The name of the connector.'''
        return self._name

    @name.setter
    def name(self, name):
        validate_attribute(name, 'serviceport_connector.name',
                           expected_type=[str, unicode], required=True)
        self._name = name

    @property
    def trans_method(self):
        '''Transport method used by the ports.

        As specified when the RT system is created. Dependent on what the RT
        Middleware used to execute the RT system supports.

        '''
        return self._trans_method

    @trans_method.setter
    def trans_method(self, trans_method):
        validate_attribute(trans_method, 'serviceport_connector.transMethod',
                           expected_type=[str, unicode], required=False)
        self._trans_method = trans_method

    @property
    def source_service_port(self):
        '''The source port in the connection.'''
        return self._source_service_port

    @source_service_port.setter
    def source_service_port(self, source_service_port):
        validate_attribute(source_service_port,
                           'serviceport_connector.sourceServicePort',
                           expected_type=TargetPort, required=True)
        self._source_service_port = source_service_port

    @property
    def target_service_port(self):
        '''The target port in the connection.'''
        return self._target_service_port

    @target_service_port.setter
    def target_service_port(self, target_service_port):
        validate_attribute(target_service_port,
                           'serviceport_connector.targetServicePort',
                           expected_type=TargetPort, required=True)
        self._target_service_port = target_service_port

    @property
    def comment(self):
        '''Comment about the connector.

        A brief comment about the connector. May or may not be displayed in
        other tools. May be empty.

        Part of the extended profile.

        '''
        return self._comment

    @comment.setter
    def comment(self, comment):
        validate_attribute(comment, 'serviceport_connector.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment

    @property
    def visible(self):
        '''Display the connector in graphical tools.

        This value controls whether graphical tools will display this connector
        or not.

        Part of the extended profile.

        '''
        return self._visible

    @visible.setter
    def visible(self, visible):
        validate_attribute(visible, 'serviceport_connector.ext.visible',
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
        validate_attribute(properties, 'serviceport_connector.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a service port connector into
        this object.

        '''
        self.connector_id = node.getAttributeNS(RTS_NS, 'connectorId')
        self.name = node.getAttributeNS(RTS_NS, 'name')
        if node.hasAttributeNS(RTS_NS, 'transMethod'):
            self.trans_method = node.getAttributeNS(RTS_NS,
                                                         'transMethod')
        else:
            self.trans_method = ''
        self.comment = node.getAttributeNS(RTS_EXT_NS, 'comment')
        if node.hasAttributeNS(RTS_EXT_NS, 'visible'):
            visible = node.getAttributeNS(RTS_EXT_NS, 'visible')
            if visible == 'true' or visible == '1':
                self.visible = True
            else:
                self.visible = False

        if node.getElementsByTagNameNS(RTS_NS, 'sourceServicePort').length != 1:
            raise InvalidServicePortConnectorNodeError
        self.source_service_port = TargetPort().parse_xml_node(\
                node.getElementsByTagNameNS(RTS_NS, 'sourceServicePort')[0])
        if node.getElementsByTagNameNS(RTS_NS, 'targetServicePort').length != 1:
            raise InvalidServicePortConnectorNodeError
        self.target_service_port = TargetPort().parse_xml_node(\
                node.getElementsByTagNameNS(RTS_NS, 'targetServicePort')[0])
        for c in get_direct_child_elements_xml(node, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value
        return self

    def parse_yaml(self, y):
        '''Parse a YAML specification of a service port connector into this
        object.

        '''
        self.connector_id = y['connectorId']
        self.name = y['name']
        if 'transMethod' in y:
            self.trans_method = y['transMethod']
        else:
            self.trans_method = ''
        if RTS_EXT_NS_YAML + 'comment' in y:
            self.comment = y[RTS_EXT_NS_YAML + 'comment']
        else:
            self.comment = ''
        self.visible = False
        if RTS_EXT_NS_YAML + 'visible' in y:
            visible = y[RTS_EXT_NS_YAML + 'visible']
            if visible == 'true' or visible == '1':
                self.visible = True
        if 'sourceServicePort' not in y:
            raise InvalidServicePortConnectorNodeError
        self.source_service_port = \
                TargetPort().parse_yaml(y['sourceServicePort'])
        if 'targetServicePort' not in y:
            raise InvalidServicePortConnectorNodeError
        self.target_service_port = \
                TargetPort().parse_yaml(y['targetServicePort'])
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y[RTS_EXT_NS_YAML + 'properties']:
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value
        return self

    def save_xml(self, doc, element):
        '''Save this service port into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'connectorId',
                               self.connector_id)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'name', self.name)
        if self.trans_method:
            element.setAttributeNS(RTS_NS, RTS_NS_S + 'transMethod',
                                   self.trans_method)
        if self.comment:
            element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'comment',
                                   self.comment)
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'visible',
                               str(self.visible).lower())
        new_element = doc.createElementNS(RTS_NS,
                                          RTS_NS_S + 'sourceServicePort')
        self.source_service_port.save_xml(doc, new_element)
        element.appendChild(new_element)
        new_element = doc.createElementNS(RTS_NS,
                                          RTS_NS_S + 'targetServicePort')
        self.target_service_port.save_xml(doc, new_element)
        element.appendChild(new_element)
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    def to_dict(self):
        '''Save this service port connector into a dictionary.'''
        d = {'connectorId': self.connector_id,
                'name': self.name,
                RTS_EXT_NS_YAML + 'visible': str(self.visible).lower(),
                'sourceServicePort': self.source_service_port.to_dict(),
                'targetServicePort': self.target_service_port.to_dict()}
        if self.trans_method:
            d['transMethod'] = self.trans_method
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

