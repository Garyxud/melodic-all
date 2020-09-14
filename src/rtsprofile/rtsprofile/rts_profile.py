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

File: rtsystem.py

Object representing an RT system. The object constructs itself using an XML
parser and an XML specification meeting the RtsProfile schema.

'''

__version__ = '$Revision: $'
# $Source$


from datetime import datetime, MINYEAR
import xml.dom
import xml.dom.minidom
import yaml

from rtsprofile import RTS_NS, RTS_NS_S, RTS_EXT_NS, RTS_EXT_NS_S, \
                       RTS_EXT_NS_YAML
from rtsprofile.component import Component
from rtsprofile.component_group import ComponentGroup
from rtsprofile.exceptions import MultipleSourcesError, \
                                  InvalidRtsProfileNodeError, RtsProfileError
from rtsprofile.message_sending import StartUp, ShutDown, Activation, \
                                       Deactivation, Resetting, Initialize, \
                                       Finalize
from rtsprofile.port_connectors import DataPortConnector, ServicePortConnector
from rtsprofile.utils import date_to_dict, get_direct_child_elements_xml, \
                             indent_string, parse_properties_xml, \
                             properties_to_xml, validate_attribute


##############################################################################
## RtsProfile object

class RtsProfile:
    def __init__(self, xml_spec=None, yaml_spec=None):
        '''Constructor.

        Pass in an RTSProfile specification either in a string or a file
        object, and the RtsProfile will be loaded from that.

        If no specification is provided, the RtsProfile will be constructed
        from the default values. Use the properties to set the values you need.

        @param xml_spec A string or open file object containing the XML
        specification of the RTS Profile. If present, the other arguments must
        be None.

        @param yaml_spec A string or open file object containing the YAML
        specification of the RTS Profile. If present, the other arguments must
        be None.

        '''
        if xml_spec:
            if yaml_spec:
                raise MultipleSourcesError('XML and YAML specifications both \
given.')
            self.parse_from_xml(xml_spec)
        elif yaml_spec:
            if xml_spec:
                raise MultipleSourcesError('XML and YAML specifications both \
given.')
            self.parse_from_yaml(yaml_spec)
        else:
            self._reset()

    def __str__(self):
        result = 'ID: {0}\nAbstract: {1}\nCreation date: {2}\n\
Update date: {3}\nVersion: {4}\n'.format(self.id, self.abstract,
                                         self.creation_date, self.update_date,
                                         self.version)
        if self.components:
            result += 'Components:\n'
            for c in self.components:
                result += '{0}\n'.format(indent_string(str(c)))
        if self.groups:
            result += 'Groups:\n'
            for g in self.groups:
                result += '{0}\n'.format(indent_string(str(g)))
        if self.data_port_connectors:
            result += 'Data port connectors:\n'
            for c in self.data_port_connectors:
                result += '{0}\n'.format(indent_string(str(c)))
        if self.service_port_connectors:
            result += 'Service port connectors:\n'
            for c in self.service_port_connectors:
                result += '{0}\n'.format(indent_string(str(c)))
        if self.startup:
            result += 'Startup: {0}\n'.format(self.startup)
        if self.shutdown:
            result += 'Shutdown: {0}\n'.format(self.shutdown)
        if self.activation:
            result += 'Activation: {0}\n'.format(self.activation)
        if self.deactivation:
            result += 'Deactivation: {0}\n'.format(self.deactivation)
        if self.resetting:
            result += 'Resetting: {0}\n'.format(self.resetting)
        if self.initializing:
            result += 'Initializing: {0}\n'.format(self.initializing)
        if self.finalizing:
            result += 'Finalizing: {0}\n'.format(self.finalizing)
        if self.comment:
            result += 'Comment: {0}\n'.format(self.comment)
        if self.version_up_log:
            result += 'Version up logs:\n'
            for vl in self.version_up_log:
                result += '{0}\n'.format(indent_string(vl))
        if self.properties:
            result += 'Properties:\n'
            for p in self.properties:
                result += '  {0}: {1}\n'.format(p, self.properties[p])
        return result[:-1] # Lop off the last new line

    ###########################################################################
    # Properties

    @property
    def id(self):
        '''ID used to distinguish the RT system.

        Typically in the format '[vendor name].[system name].[version]'.

        '''
        return self._id

    @id.setter
    def id(self, id):
        print 'setting id to ', id
        validate_attribute(id, 'rts_profile.id',
                           expected_type=[str, unicode], required=True)
        self._id = id

    @property
    def abstract(self):
        '''Description of this RT system.

        May be empty.

        '''
        return self._abstract

    @abstract.setter
    def abstract(self, abstract):
        validate_attribute(abstract, 'rts_profile.abstract',
                           expected_type=[str, unicode], required=True)
        self._abstract = abstract

    @property
    def creation_date(self):
        '''The date this RT system was first created.

        Usually set automatically by the tool that created the system.

        '''
        return self._creation_date

    @creation_date.setter
    def creation_date(self, creation_date):
        validate_attribute(creation_date, 'rts_profile.creationDate',
                           expected_type=[str, unicode], required=True)
        self._creation_date = creation_date

    @property
    def update_date(self):
        '''The date this RT system was most recently updated.

        Usually set automatically by the tool that created the system.

        '''
        return self._update_date

    @update_date.setter
    def update_date(self, update_date):
        validate_attribute(update_date, 'rts_profile.updateDate',
                           expected_type=[str, unicode], required=True)
        self._update_date = update_date

    @property
    def version(self):
        '''Version of the RTSProfile specification this is in.'''
        return self._version

    @version.setter
    def version(self, version):
        validate_attribute(version, 'rts_profile.version',
                           expected_type=[str, unicode], required=True)
        self._version = version

    @property
    def components(self):
        '''Information about the components that make up the RT system.

        May be an empty list if there are no components. Members are of type
        @ref Component.

        '''
        return self._components

    @components.setter
    def components(self, components):
        validate_attribute(components, 'rts_profile.Components',
                           expected_type=list, required=True)
        self._components = components

    @property
    def groups(self):
        '''Information about the component groups in the RT system.

        May be an empty list if there are no groups. Members are of type @ref
        ComponentGroup.

        '''
        return self._groups

    @groups.setter
    def groups(self, groups):
        validate_attribute(groups, 'rts_profile.Groups',
                           expected_type=list, required=True)
        self._groups = groups

    @property
    def data_port_connectors(self):
        '''Connections between data ports in the RT system.

        Members are of type @ref DataPortConnector.

        '''
        return self._data_port_connectors

    @data_port_connectors.setter
    def data_port_connectors(self, data_port_connectors):
        validate_attribute(data_port_connectors,
                           'rts_profile.DataPortConnectors',
                           expected_type=list, required=True)
        self._data_port_connectors = data_port_connectors

    @property
    def service_port_connectors(self):
        '''Connections between service ports in the RT system.

        Members are of type @ref ServicePortConnector.

        '''
        return self._service_port_connectors

    @service_port_connectors.setter
    def service_port_connectors(self, service_port_connectors):
        validate_attribute(service_port_connectors,
                           'rts_profile.ServicePortConnectors',
                           expected_type=list, required=True)
        self._service_port_connectors = service_port_connectors

    @property
    def startup(self):
        '''Ordering and conditions for when the RT system is started.'''
        return self._startup

    @startup.setter
    def startup(self, startup):
        validate_attribute(startup, 'rts_profile.StartUp',
                           expected_type=StartUp, required=False)
        self._startup = startup

    @property
    def shutdown(self):
        '''Ordering and conditions for when the RT system is shut down.'''
        return self._shutdown

    @shutdown.setter
    def shutdown(self, shutdown):
        validate_attribute(shutdown, 'rts_profile.ShutDown',
                           expected_type=ShutDown, required=False)
        self._shutdown = shutdown

    @property
    def activation(self):
        '''Ordering and conditions for when the RT system is activated.'''
        return self._activation

    @activation.setter
    def activation(self, activation):
        validate_attribute(activation, 'rts_profile.Activation',
                           expected_type=Activation, required=False)
        self._activation = activation

    @property
    def deactivation(self):
        '''Ordering and conditions for when the RT system is deactivated.'''
        return self._deactivation

    @deactivation.setter
    def deactivation(self, deactivation):
        validate_attribute(deactivation, 'rts_profile.Deactivation',
                           expected_type=Deactivation, required=False)
        self._deactivation = deactivation

    @property
    def resetting(self):
        '''Ordering and conditions for when the RT system is reset.'''
        return self._resetting

    @resetting.setter
    def resetting(self, resetting):
        validate_attribute(resetting, 'rts_profile.Resetting',
                           expected_type=Resetting, required=False)
        self._resetting = resetting

    @property
    def initializing(self):
        '''Ordering and conditions for when the RT system is initialised.'''
        return self._initializing

    @initializing.setter
    def initializing(self, initializing):
        validate_attribute(initializing, 'rts_profile.Initializing',
                           expected_type=Initialize, required=False)
        self._initializing = initializing

    @property
    def finalizing(self):
        '''Ordering and conditions for when the RT system is finalised.'''
        return self._finalizing

    @finalizing.setter
    def finalizing(self, finalizing):
        validate_attribute(finalizing, 'rts_profile.Finalizing',
                           expected_type=Finalize, required=False)
        self._finalizing = finalizing

    @property
    def comment(self):
        '''Comment about the system.

        A brief comment about the system. May or may not be displayed in other
        tools. May be empty.

        Part of the extended profile.

        '''
        return self._comment

    @comment.setter
    def comment(self, comment):
        validate_attribute(comment, 'rtsprofile.ext.comment',
                           expected_type=[str, unicode], required=False)
        self._comment = comment

    @property
    def version_up_log(self):
        '''Log entries for new versions.

        When an update to a system is made, the log entry describing changes is
        stored in this value.

        Part of the extended profile.

        '''
        return self._version_up_log

    @version_up_log.setter
    def version_up_log(self, version_up_log):
        validate_attribute(version_up_log, 'rtsprofile.ext.VersionUpLog',
                           expected_type=list, required=False)
        self._version_up_log = version_up_log

    @property
    def properties(self):
        '''Miscellaneous properties.

        Stores key/value pair properties.

        Part of the extended profile.

        '''
        return self._properties

    @properties.setter
    def properties(self, properties):
        validate_attribute(properties, 'rtsprofile.ext.Properties',
                           expected_type=dict, required=False)
        self._properties = properties

    ###########################################################################
    # API functions

    def find_comp_by_target(self, target):
        '''Finds a component using a TargetComponent or one of its subclasses.

        @param A @ref TargetComponent object or subclass of @ref
        TargetComponent.
        @return A Component object matching the target.
        @raises MissingComponentError

        '''
        for comp in self._components:
            if comp.id == target.component_id and \
                    comp.instance_name == target.instance_name:
                return comp
        raise MissingComponentError

    def optional_data_connections(self):
        '''Finds all data connections in which one or more components are not
        required.

        If all the components involved in a connection are required, that
        connection is also required. If one or more are not required, that
        connection is optional.

        '''
        result = []
        for conn in self._data_port_connectors:
            source_comp = self.find_comp_by_target(conn.source_data_port)
            target_comp = self.find_comp_by_target(conn.target_data_port)
            if not source_comp.is_required or not target_comp.is_required:
                result.append(conn)
        return result

    def optional_service_connections(self):
        '''Finds all service connections in which one or more components are
        not required.

        If all the components involved in a connection are required, that
        connection is also required. If one or more are not required, that
        connection is optional.

        '''
        result = []
        for conn in self._service_port_connectors:
            source_comp = self.find_comp_by_target(conn.source_service_port)
            target_comp = self.find_comp_by_target(conn.target_service_port)
            if not source_comp.is_required or not target_comp.is_required:
                result.append(conn)
        return result

    def required_data_connections(self):
        '''Finds all data connections in which all components are required.

        If all the components involved in a connection are required, that
        connection is also required. If one or more are not required, that
        connection is optional.

        '''
        result = []
        for conn in self._data_port_connectors:
            source_comp = self.find_comp_by_target(conn.source_data_port)
            target_comp = self.find_comp_by_target(conn.target_data_port)
            if source_comp.is_required and target_comp.is_required:
                result.append(conn)
        return result

    def required_service_connections(self):
        '''Finds all service connections in which all components are required.

        If all the components involved in a connection are required, that
        connection is also required. If one or more are not required, that
        connection is optional.

        '''
        result = []
        for conn in self._service_port_connectors:
            source_comp = self.find_comp_by_target(conn.source_service_port)
            target_comp = self.find_comp_by_target(conn.target_service_port)
            if source_comp.is_required and target_comp.is_required:
                result.append(conn)
        return result

    ###########################################################################
    # XML

    def parse_from_xml(self, xml_spec):
        '''Parse a string or file containing an XML specification.'''
        if type(xml_spec) == str or type(xml_spec) == unicode:
            dom = xml.dom.minidom.parseString(xml_spec)
        else:
            dom = xml.dom.minidom.parse(xml_spec)
        self._parse_xml(dom)
        dom.unlink()

    def save_to_xml(self):
        '''Save this RtsProfile into an XML-formatted string.'''
        xml_obj = self._to_xml_dom()
        return xml_obj.toprettyxml(indent='    ')

    ###########################################################################
    # YAML

    def parse_from_yaml(self, yaml_spec):
        '''Parse a string or file containing a YAML specification.'''
        self._parse_yaml(yaml.safe_load(yaml_spec))

    def save_to_yaml(self):
        '''Save this RtsProfile into a YAML-formatted string.'''
        return self._to_yaml()

    ###########################################################################
    # Internal functions

    def _parse_xml(self, dom):
        self._reset()
        root = dom.documentElement
        # Get the attributes
        self.id = root.getAttributeNS(RTS_NS, 'id')
        self.abstract = root.getAttributeNS(RTS_NS, 'abstract')
        self.creation_date = root.getAttributeNS(RTS_NS, 'creationDate')
        self.update_date = root.getAttributeNS(RTS_NS, 'updateDate')
        self.version = root.getAttributeNS(RTS_NS, 'version')
        self.comment = root.getAttributeNS(RTS_EXT_NS, 'comment')
        # Parse the children
        for c in root.getElementsByTagNameNS(RTS_NS, 'Components'):
            self._components.append(Component().parse_xml_node(c))
        for c in root.getElementsByTagNameNS(RTS_NS, 'Groups'):
            self._groups.append(ComponentGroup().parse_xml_node(c))
        for c in root.getElementsByTagNameNS(RTS_NS, 'DataPortConnectors'):
            self._data_port_connectors.append(DataPortConnector().parse_xml_node(c))
        for c in root.getElementsByTagNameNS(RTS_NS, 'ServicePortConnectors'):
            self._service_port_connectors.append(ServicePortConnector().parse_xml_node(c))
        # These children should have zero or one
        c = root.getElementsByTagNameNS(RTS_NS, 'StartUp')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('StartUp')
            self._startup = StartUp().parse_xml_node(c[0])
        c = root.getElementsByTagNameNS(RTS_NS, 'ShutDown')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('ShutDown')
            self._shutdown = ShutDown().parse_xml_node(c[0])
        c = root.getElementsByTagNameNS(RTS_NS, 'Activation')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('Activation')
            self._activation = Activation().parse_xml_node(c[0])
        c = root.getElementsByTagNameNS(RTS_NS, 'Deactivation')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('Deactivation')
            self._deactivation = Deactivation().parse_xml_node(c[0])
        c = root.getElementsByTagNameNS(RTS_NS, 'Resetting')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('Resetting')
            self._resetting = Resetting().parse_xml_node(c[0])
        c = root.getElementsByTagNameNS(RTS_NS, 'Initializing')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('Initializing')
            self._initializing = Initializing().parse_xml_node(c[0])
        c = root.getElementsByTagNameNS(RTS_NS, 'Finalizing')
        if c.length > 0:
            if c.length > 1:
                raise InvalidRtsProfileNodeError('Finalizing')
            self._finalizing = Finalizing().parse_xml_node(c[0])
        # Extended profile children
        for c in root.getElementsByTagNameNS(RTS_EXT_NS, 'VersionUpLog'):
            if c.nodeType == c.TEXT_NODE:
                self._version_up_log.append(c.data)
            else:
                print >>sys.stderr, 'Warning: bad VersionUpLog node type.'
        for c in get_direct_child_elements_xml(root, prefix=RTS_EXT_NS,
                                               local_name='Properties'):
            name, value = parse_properties_xml(c)
            self._properties[name] = value

    def _parse_yaml(self, spec):
        self._reset()
        if not 'rtsProfile' in spec:
            raise RtsProfileError('Missing root node.')
        root = spec['rtsProfile']
        # Get the attributes
        self.id = root['id']
        if 'abstract' in root:
            self.abstract = root['abstract']
        self.creation_date = '{year:04}-{month:02}-{day:02}T{hour:02}:\
{minute:02}:{second:02}'.format(**root['creationDate'])
        self.update_date = '{year:04}-{month:02}-{day:02}T{hour:02}:\
{minute:02}:{second:02}'.format(**root['updateDate'])
        self.version = root['version']
        if RTS_EXT_NS_YAML + 'comment' in root:
            self.comment = root[RTS_EXT_NS_YAML + 'comment']
        # Parse the children
        if 'components' in root:
            for c in root['components']:
                self._components.append(Component().parse_yaml(c))
        if 'groups' in root:
            for c in root['groups']:
                self._groups.append(ComponentGroup().parse_yaml(c))
        if 'dataPortConnectors' in root:
            for c in root['dataPortConnectors']:
                self._data_port_connectors.append(DataPortConnector().parse_yaml(c))
        if 'servicePortConnectors' in root:
            for c in root['servicePortConnectors']:
                self._service_port_connectors.append(ServicePortConnector().parse_yaml(c))
        # Singular children
        if 'startUp' in root:
            self._startup = StartUp().parse_yaml(root['startUp'])
        if 'shutDown' in root:
            self._shutdown = ShutDown().parse_yaml(root['shutDown'])
        if 'activation' in root:
            self._activation = Activation().parse_yaml(root['activation'])
        if 'deactivation' in root:
            self._deactivation = Deactivation().parse_yaml(root['deactivation'])
        if 'resetting' in root:
            self._resetting = Resetting().parse_yaml(root['resetting'])
        if 'initializing' in root:
            self._initializing = Initializing().parse_yaml(root['initializing'])
        if 'finalizing' in root:
            self._finalizing = Finalizing().parse_yaml(root['finalizing'])
        # Extended profile children
        if RTS_EXT_NS_YAML + 'versionUpLogs' in root:
            for c in root[RTS_EXT_NS_YAML + 'versionUpLogs']:
                self._version_up_log.append(c)
        if RTS_EXT_NS_YAML + 'properties' in root:
            for p in root[RTS_EXT_NS_YAML + 'properties']:
                if 'value' in p:
                    value = p['value']
                else:
                    value = None
                self._properties[p['name']] = value

    def _reset(self):
        # Clears all values in the class in preparation for parsing an
        # XML file.
        # Attributes
        self._id = ''
        self._abstract = None
        self._creation_date = datetime(MINYEAR, 1, 1)
        self._update_date = datetime(MINYEAR, 1, 1)
        self._version = ''
        # Children
        self._components = []
        self._groups = []
        self._data_port_connectors = []
        self._service_port_connectors = []
        self._startup = None
        self._shutdown = None
        self._activation = None
        self._deactivation = None
        self._resetting = None
        self._initializing = None
        self._finalizing = None
        # Extended spec
        self._comment = ''
        self._version_up_log = ''
        self._properties = {}

    def _to_dict(self):
        # This converts an RTSProfile object into a dictionary. The typical use
        # for this is to then dump that dictionary as YAML.
        # We need to do this because the RtsProfile object hierarchy does not
        # correspond directly to the YAML object hierarchy.
        prof = {'id': self.id,
                'abstract': self.abstract,
                'creationDate': date_to_dict(self.creation_date),
                'updateDate': date_to_dict(self.update_date),
                'version': self.version}
        if self.comment:
            prof[RTS_EXT_NS_YAML + 'comment'] = self.comment

        components = []
        for c in self.components:
            components.append(c.to_dict())
        if components:
            prof['components'] = components
        groups = []
        for g in self.groups:
            groups.append(g.to_dict())
        if groups:
            prof['groups'] = groups
        d_connectors = []
        for c in self.data_port_connectors:
            d_connectors.append(c.to_dict())
        if d_connectors:
            prof['dataPortConnectors'] = d_connectors
        s_connectors = []
        for c in self.service_port_connectors:
            s_connectors.append(c.to_dict())
        if s_connectors:
            prof['servicePortConnectors'] = s_connectors

        if self.startup:
            prof['startUp'] = self.startup.to_dict()
        if self.shutdown:
            prof['shutDown'] = self.shutdown.to_dict()
        if self.activation:
            prof['activation'] = self.activation.to_dict()
        if self.deactivation:
            prof['deactivation'] = self.deactivation.to_dict()
        if self.resetting:
            prof['resetting'] = self.resetting.to_dict()
        if self.initializing:
            prof['initializing'] = self.initializing.to_dict()
        if self.finalizing:
            prof['finalizing'] = self.finalizing.to_dict()

        log = []
        for l in self.version_up_log:
            log.append(l)
        if log:
            prof[RTS_EXT_NS_YAML + 'versionUpLogs'] = log
        props = []
        for name in self.properties:
            p = {'name': name}
            if self.properties[name]:
                p['value'] = str(self.properties[name])
            props.append(p)
        if props:
            prof[RTS_EXT_NS_YAML + 'properties'] = props

        return {'rtsProfile': prof}

    def _to_xml_dom(self):
        impl = xml.dom.minidom.getDOMImplementation()
        doc = impl.createDocument(RTS_NS, RTS_NS_S + 'RtsProfile', None)
        doc.documentElement.setAttribute('xmlns:rts', RTS_NS)
        doc.documentElement.setAttribute('xmlns:rtsExt', RTS_EXT_NS)
        doc.documentElement.setAttribute('xmlns:xsi',
                'http://www.w3.org/2001/XMLSchema-instance')

        doc.documentElement.setAttributeNS(RTS_NS, RTS_NS_S + 'id', self.id)
        doc.documentElement.setAttributeNS(RTS_NS, RTS_NS_S + 'abstract',
                                           self.abstract)
        doc.documentElement.setAttributeNS(RTS_NS, RTS_NS_S + 'creationDate',
                                           self.creation_date)
        doc.documentElement.setAttributeNS(RTS_NS, RTS_NS_S + 'updateDate',
                                           self.update_date)
        doc.documentElement.setAttributeNS(RTS_NS, RTS_NS_S + 'version',
                                           str(self.version))
        if self.comment:
            doc.documentElement.setAttributeNS(RTS_EXT_NS,
                                               RTS_EXT_NS_S + 'comment',
                                               self.comment)
        for c in self.components:
            new_comp_element = doc.createElementNS(RTS_NS,
                                                   RTS_NS_S + 'Components')
            c.save_xml(doc, new_comp_element)
            doc.documentElement.appendChild(new_comp_element)
        for g in self.groups:
            new_group_element = doc.createElementNS(RTS_NS,
                                                    RTS_NS_S + 'Groups')
            g.save_xml(doc, new_group_element)
            doc.documentElement.appendChild(new_group_element)
        for dc in self.data_port_connectors:
            new_conn_element = doc.createElementNS(RTS_NS,
                    RTS_NS_S + 'DataPortConnectors')
            dc.save_xml(doc, new_conn_element)
            doc.documentElement.appendChild(new_conn_element)
        for sc in self.service_port_connectors:
            new_conn_element = doc.createElementNS(RTS_NS,
                    RTS_NS_S + 'ServicePortConnectors')
            sc.save_xml(doc, new_conn_element)
            doc.documentElement.appendChild(new_conn_element)
        if self.startup:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'StartUp')
            self.startup.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        if self.shutdown:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'ShutDown')
            self.shutdown.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        if self.activation:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'Activation')
            self.activation.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        if self.deactivation:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'Deactivation')
            self.deactivation.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        if self.resetting:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'Resetting')
            self.resetting.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        if self.initializing:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'Initializing')
            self.initializing.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        if self.finalizing:
            new_cond = doc.createElementNS(RTS_NS, RTS_NS_S + 'Finalizing')
            self.finalizing.save_xml(doc, new_cond)
            doc.documentElement.appendChild(new_cond)
        for vl in self.version_up_log:
            new_vl_element = doc.createElementNS(RTS_EXT_NS,
                                                 RTS_EXT_NS_S + 'VersionUpLog')
            new_text_node = doc.createTextNode(vl)
            new_vl_element.appendChild(new_text_node)
            doc.documentElement.appendChild(new_vl_element)
        for p in self.properties:
            new_prop_element = doc.createElementNS(RTS_EXT_NS,
                                                   RTS_EXT_NS_S + 'Properties')
            properties_to_xml(new_prop_element, p, self.properties[p])
            doc.documentElement.appendChild(new_prop_element)
        return doc

    def _to_yaml(self):
        return yaml.safe_dump(self._to_dict())


# vim: tw=79

