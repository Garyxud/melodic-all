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

File: component.py

Object representing a component in an RT system.

'''

__version__ = '$Revision: $'
# $Source$


from rtsprofile import RTS_NS, RTS_NS_S, RTS_EXT_NS, RTS_EXT_NS_S, \
                       RTS_EXT_NS_YAML
from rtsprofile import composite_type as comp_type
from rtsprofile.config_set import ConfigurationSet
from rtsprofile.exceptions import InvalidCompositeTypeError
from rtsprofile.exec_context import ExecutionContext
from rtsprofile.location import Location
from rtsprofile.ports import DataPort, ServicePort
from rtsprofile.utils import get_direct_child_elements_xml, \
                             indent_string, parse_properties_xml, \
                             properties_to_xml, validate_attribute


##############################################################################
## Component object

class Component(object):
    '''Information about a component contained in an RT system.'''

    def __init__(self, id='', path_uri='', active_configuration_set='',
                 instance_name='', composite_type=comp_type.NONE,
                 is_required=False, comment='', visible=True,
                 location=Location()):
        '''@param id Component ID.
        @type id str
        @param path_uri Path to the component.
        @type path_uri str
        @param active_configuration_set Name of the active configuration set.
        @type active_configuration_set str
        @param instance_name Component's instance name.
        @type instance_name str
        @param composite_type Type of composition the component is in.
        @type composite_type CompositeType
        @param is_required If the component is optional in the system.
        @type is_required bool
        @param comment A comment about the component.
        @type comment str
        @param visible If this component is visible in graphical displays.
        @type visible bool
        @param location The location of this component in graphical displays.
        @type location Location

        '''
        self._reset()
        validate_attribute(id, 'component.id',
                           expected_type=[str, unicode], required=False)
        self._id = id
        validate_attribute(path_uri, 'component.pathUri',
                           expected_type=[str, unicode], required=False)
        self._path_uri = path_uri
        validate_attribute(active_configuration_set,
                           'component.activeConfigurationSet',
                           expected_type=[str, unicode], required=False)
        self._active_config_set = active_configuration_set
        validate_attribute(instance_name, 'component.instanceName',
                           expected_type=[str, unicode], required=False)
        self._instance_name = instance_name
        validate_attribute(composite_type, 'component.compositeType',
                           expected_type=comp_type.const_type, required=False)
        self._composite_type = composite_type
        validate_attribute(is_required, 'component.isRequired',
                           expected_type=bool)
        self._is_required = is_required
        validate_attribute(comment, 'component.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment
        validate_attribute(visible, 'component.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible
        validate_attribute(location, 'component.ext.Location',
                           expected_type=Location, required=True)
        self._location = location

    def __str__(self):
        result = 'Instance name: {3}\n  ID: {0}\n  Path URI: {1}\n  Active \
configuration set: {2}\n  Composite type: {4}\n  Is required: {5}\n'.format(\
                self.id, self.path_uri, self.active_configuration_set,
                self.instance_name, comp_type.to_string(self.composite_type),
                self.is_required)
        if self.comment:
            result += '  Comment: {0}\n'.format(self.comment)
        result += '  Visible: {0}\n'.format(self.visible)
        if self.data_ports:
            result += '  Data ports:\n'
            for p in self.data_ports:
                result += '{0}\n'.format(indent_string(str(p), num_spaces=4))
        if self.service_ports:
            result += '  Service ports:\n'
            for p in self.service_ports:
                result += '{0}\n'.format(indent_string(str(p), num_spaces=4))
        if self.configuration_sets:
            result += '  Configuration sets:\n'
            for c in self.configuration_sets:
                result += '{0}\n'.format(indent_string(str(c), num_spaces=4))
        if self.execution_contexts:
            result += '  Execution contexts:\n'
            for e in self.execution_contexts:
                result += '{0}\n'.format(indent_string(str(e), num_spaces=4))
        if self.participants:
            result += '  Participants:\n'
            for p in self.participants:
                result += '{0}\n'.format(indent_string(str(p)))
        result += '  Location:\n{0}\n'.format(indent_string(str(self.location),
                                                           num_spaces=4))
        if self.properties:
            result += '  Properties:\n'
            for p in self.properties:
                result += '    {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    ###########################################################################
    # Properties

    @property
    def id(self):
        '''ID of this component in the RT system.

        In case of the same RT Component specification being used to create
        multiple RT Components within a single RT system, this ID is prepended
        to the instance name attribute to distinguish individual components.

        '''
        return self._id

    @id.setter
    def id(self, id):
        validate_attribute(id, 'component.id',
                           expected_type=[str, unicode], required=True)
        self._id = id

    @property
    def path_uri(self):
        '''Path to where this component is registered in URI format.'''
        return self._path_uri

    @path_uri.setter
    def path_uri(self, path_uri):
        validate_attribute(path_uri, 'component.pathUri',
                           expected_type=[str, unicode], required=True)
        self._path_uri = path_uri

    @property
    def active_configuration_set(self):
        '''The ID of the active configuration set of the component.

        If no configuration set is active, this may be empty.

        '''
        return self._active_config_set

    @active_configuration_set.setter
    def active_configuration_set(self, active_config_set):
        validate_attribute(active_config_set,
                           'component.activeConfigurationSet',
                           expected_type=[str, unicode], required=False)
        self._active_config_set = active_config_set

    @property
    def instance_name(self):
        '''Instance name of the component in the RT system.

        In case of the same RT Component specification being used to create
        multiple RT Components within a single RT system, this instance name is
        appended to the ID attribute to distinguish individual components.

        '''
        return self._instance_name

    @instance_name.setter
    def instance_name(self, instance_name):
        validate_attribute(instance_name, 'component.instanceName',
                           expected_type=[str, unicode], required=True)
        self._instance_name = instance_name

    @property
    def composite_type(self):
        '''The type of composite component this component is involved in.

        If this component is involved in a composite component, this attribute
        specifies the type of composition. See @ref CompositeType for valid
        values.

        '''
        return self._composite_type

    @composite_type.setter
    def composite_type(self, composite_type):
        validate_attribute(composite_type, 'component.compositeType',
                           expected_type=comp_type.const_type, required=True)
        self._composite_type = composite_type

    @property
    def is_required(self):
        '''Specifies if this component is optional in the RT system.

        Sometimes a component does not need to be present for an RT system to
        function. If this component must be present for the RT system to
        function, this attribute will be True.

        '''
        return self._is_required

    @is_required.setter
    def is_required(self, is_required):
        validate_attribute(is_required, 'component.isRequired',
                           expected_type=bool)
        self._is_required = is_required

    @property
    def data_ports(self):
        '''Data ports owned by this component.

        May be an empty list if this component has no data ports. Members are
        of type @ref DataPort.

        '''
        return self._data_ports

    @data_ports.setter
    def data_ports(self, data_ports):
        validate_attribute(data_ports, 'component.DataPorts',
                           expected_type=list, required=False)
        self._data_ports = data_ports

    @property
    def service_ports(self):
        '''Service ports owned by this component.

        May be an empty list if this component has no service ports. Members
        are of type @ref ServicePort.

        '''
        return self._service_ports

    @service_ports.setter
    def service_ports(self, service_ports):
        validate_attribute(service_ports, 'component.ServicePorts',
                           expected_type=list, required=False)
        self._service_ports = service_ports

    @property
    def configuration_sets(self):
        '''The configuration sets in this component.

        May be an empty list if this component has no configuration sets.
        Members are of type @ref ConfigurationSet.

        '''
        return self._config_sets

    @configuration_sets.setter
    def configuration_sets(self, configuration_sets):
        validate_attribute(configuration_sets, 'component.ConfigurationSets',
                           expected_type=list, required=False)
        self._config_sets = configuration_sets

    @property
    def execution_contexts(self):
        '''The execution contexts owned by this component.

        May be an empty list if this component does not own any contexts.
        Members are of type @ref ExecutionContext.

        '''
        return self._exec_contexts

    @execution_contexts.setter
    def execution_contexts(self, execution_contexts):
        validate_attribute(execution_contexts, 'component.ExecutionContexts',
                           expected_type=list, required=False)
        self._exec_contexts = execution_contexts

    @property
    def participants(self):
        '''The list of participating components, if this component is a
        composite component.

        Members are of type @ref Participant.

        '''
        return self._participants

    @participants.setter
    def participants(self, participants):
        validate_attribute(participants, 'component.Participants',
                           expected_type=list, required=False)
        self._participants = participants

    @property
    def comment(self):
        '''Comment about the component.

        A brief comment about the component. May or may not be displayed in
        other tools. May be empty.

        Part of the extended profile.

        '''
        return self._comment

    @comment.setter
    def comment(self, comment):
        validate_attribute(comment, 'component.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment

    @property
    def visible(self):
        '''Display the component in graphical tools.

        This value controls whether graphical tools will display this component
        or not.

        Part of the extended profile.

        '''
        return self._visible

    @visible.setter
    def visible(self, visible):
        validate_attribute(visible, 'component.ext.visible',
                           expected_type=bool, required=False)
        self._visible = visible

    @property
    def location(self):
        '''Specifies the position of the component in graphical tools.

        Part of the extended profile.

        '''
        return self._location

    @location.setter
    def location(self, location):
        validate_attribute(location, 'component.ext.Location',
                           expected_type=Location, required=True)
        self._location = location

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties, 'component.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    ###########################################################################
    # API functions

    def get_configuration_set_by_id(self, id):
        '''Finds a configuration set in the component by its ID.

        @param id The ID of the configuration set to search for.
        @return The ConfigurationSet object for the set, or None if it was not
        found.

        '''
        for cs in self.configuration_sets:
            if cs.id == id:
                return cs
        return None

    ###########################################################################
    # XML

    def parse_xml_node(self, node):
        '''Parse an xml.dom Node object representing a component into this
        object.

        '''
        self._reset()
        # Get the attributes
        self.id = node.getAttributeNS(RTS_NS, 'id')
        self.path_uri = node.getAttributeNS(RTS_NS, 'pathUri')
        if node.hasAttributeNS(RTS_NS, 'activeConfigurationSet'):
            self.active_configuration_set = node.getAttributeNS(RTS_NS,
                    'activeConfigurationSet')
        else:
            self.active_configuration_set = ''
        self.instance_name = node.getAttributeNS(RTS_NS, 'instanceName')
        self.compositeType = comp_type.from_string(node.getAttributeNS(RTS_NS,
                'compositeType'))
        required = node.getAttributeNS(RTS_NS, 'isRequired')
        if required == 'true' or required == '1':
            self.is_required = True
        else:
            self.is_required = False
        self.comment = node.getAttributeNS(RTS_EXT_NS, 'comment')
        if node.hasAttributeNS(RTS_EXT_NS, 'visible'):
            visible = node.getAttributeNS(RTS_EXT_NS, 'visible')
            if visible.lower() == 'true' or visible == '1':
                self.visible = True
            else:
                self.visible = False

        # Get the children
        for c in node.getElementsByTagNameNS(RTS_NS, 'DataPorts'):
            self._data_ports.append(DataPort().parse_xml_node(c))
        for c in node.getElementsByTagNameNS(RTS_NS, 'ServicePorts'):
            self._service_ports.append(ServicePort().parse_xml_node(c))
        for c in node.getElementsByTagNameNS(RTS_NS, 'ConfigurationSets'):
            self._config_sets.append(ConfigurationSet().parse_xml_node(c))
        for c in node.getElementsByTagNameNS(RTS_NS, 'ExecutionContexts'):
            self._exec_contexts.append(ExecutionContext().parse_xml_node(c))
        for c in node.getElementsByTagNameNS(RTS_NS, 'Participants'):
            self._participants.append(Participant().parse_xml_node(c))
        # Extended profile children
        c = node.getElementsByTagNameNS(RTS_EXT_NS, 'Location')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('Location')
            self._location = Location().parse_xml_node(c[0])
        for c in get_direct_child_elements_xml(node, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value

        return self

    def save_xml(self, doc, element):
        '''Save this component into an xml.dom.Element object.'''
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'id', self.id)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'pathUri', self.path_uri)
        if self.active_configuration_set:
            element.setAttributeNS(RTS_NS, RTS_NS_S + 'activeConfigurationSet',
                                   self.active_configuration_set)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'instanceName',
                               self.instance_name)
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'compositeType',
                               comp_type.to_string(self.composite_type))
        element.setAttributeNS(RTS_NS, RTS_NS_S + 'isRequired',
                               str(self.is_required).lower())
        if self.comment:
            element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'comment',
                                   self.comment)
        element.setAttributeNS(RTS_EXT_NS, RTS_EXT_NS_S + 'visible',
                               str(self.visible).lower())
        for port in self.data_ports:
            new_element = doc.createElementNS(RTS_NS, RTS_NS_S + 'DataPorts')
            port.save_xml(doc, new_element)
            element.appendChild(new_element)
        for port in self.service_ports:
            new_element = doc.createElementNS(RTS_NS,
                                              RTS_NS_S + 'ServicePorts')
            port.save_xml(doc, new_element)
            element.appendChild(new_element)
        for cs in self.configuration_sets:
            new_element = doc.createElementNS(RTS_NS,
                                              RTS_NS_S + 'ConfigurationSets')
            cs.save_xml(doc, new_element)
            element.appendChild(new_element)
        for ec in self.execution_contexts:
            new_element = doc.createElementNS(RTS_NS,
                                              RTS_NS_S + 'ExecutionContexts')
            ec.save_xml(doc, new_element)
            element.appendChild(new_element)
        for p in self.participants:
            new_element = doc.createElementNS(RTS_NS,
                                              RTS_NS_S + 'Participants')
            p.save_xml(doc, new_element)
            element.appendChild(new_element)
        new_element = doc.createElementNS(RTS_EXT_NS,
                                          RTS_EXT_NS_S + 'Location')
        self._location.save_xml(doc, new_element)
        element.appendChild(new_element)
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            element.appendChild(new_prop_element)

    ###########################################################################
    # YAML

    def parse_yaml(self, y):
        '''Parse a YAML specification of a component into this object.'''
        self._reset()
        self.id = y['id']
        self.path_uri = y['pathUri']
        if 'activeConfigurationSet' in y:
            self.active_configuration_set = y['activeConfigurationSet']
        else:
            self.active_configuration_set = ''
        self.instance_name = y['instanceName']
        self.compositeType = comp_type.from_string(y['compositeType'])
        required = y['isRequired']
        if required == 'true' or required == '1':
            self.is_required = True
        else:
            self.is_required = False
        if RTS_EXT_NS_YAML + 'comment' in y:
            self.comment = y[RTS_EXT_NS_YAML + 'comment']
        self.visible = False
        if RTS_EXT_NS_YAML + 'visible' in y:
            visible = y.get(RTS_EXT_NS_YAML + 'visible')
            if visible == True or visible == 'true' or visible == 'True':
                self.visible = True

        # Get the children
        if 'dataPorts' in y:
            for p in y.get('dataPorts'):
                self._data_ports.append(DataPort().parse_yaml(p))
        if 'servicePorts' in y:
            for p in y.get('servicePorts'):
                self._service_ports.append(ServicePort().parse_yaml(p))
        if 'configurationSets' in y:
            for p in y.get('configurationSets'):
                self._config_sets.append(ConfigurationSet().parse_yaml(p))
        if 'executionContexts' in y:
            for p in y.get('executionContexts'):
                self._exec_contexts.append(ExecutionContext().parse_yaml(p))
        if 'participants' in y:
            for p in y.get('participants'):
                self._participants.append(Participant().parse_yaml(p))

        # Extended profile children
        if RTS_EXT_NS_YAML + 'location' in y:
            l = y[RTS_EXT_NS_YAML + 'location']
            self._location = Location().parse_yaml(l)
        if RTS_EXT_NS_YAML + 'properties' in y:
            for p in y.get(RTS_EXT_NS_YAML + 'properties'):
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value

        return self

    def to_dict(self):
        d = {'id': self.id,
                'pathUri': self.path_uri,
                'instanceName': self.instance_name,
                'compositeType': comp_type.to_string(self.composite_type),
                'isRequired': str(self.is_required).lower(),
                RTS_EXT_NS_YAML + 'visible': str(self.visible).lower()}
        if self.active_configuration_set:
            d['activeConfigurationSet'] = self.active_configuration_set
        if self.comment:
            d[RTS_EXT_NS_YAML + 'comment'] = self.comment

        ports = []
        for p in self.data_ports:
            ports.append(p.to_dict())
        if ports:
            d['dataPorts'] = ports
        ports = []
        for p in self.service_ports:
            ports.append(p.to_dict())
        if ports:
            d['servicePorts'] = ports
        sets = []
        for cs in self.configuration_sets:
            sets.append(cs.to_dict())
        if sets:
            d['configurationSets'] = sets
        ecs = []
        for ec in self.execution_contexts:
            ecs.append(ec.to_dict())
        if ecs:
            d['executionContexts'] = ecs
        participants = []
        for p in self.participants:
            participants.append(p.to_dict())
        if participants:
            d['participants'] = participants

        d[RTS_EXT_NS_YAML + 'location'] = self._location.to_dict()
        props = []
        for name in self.properties:
            p = {'name': name}
            if self.properties[name]:
                p['value'] = str(self.properties[name])
            props.append(p)
        if props:
            d[RTS_EXT_NS_YAML + 'properties'] = props

        return d

    ###########################################################################
    # Internal functions

    def _reset(self):
        # Clears all values in the class in preparation for parsing an XML
        # file.
        # Attributes
        self._id = ''
        self._path_uri = ''
        self._active_config_set = ''
        self._instance_name = ''
        self._composite_type = comp_type.NONE
        self._is_required = False
        # Children
        self._data_ports = []
        self._service_ports = []
        self._config_sets = []
        self._exec_contexts = []
        self._participants = []
        # Extended spec
        self._comment = ''
        self._visible = True
        self._location = Location()
        self._properties = {}


# vim: tw=79

