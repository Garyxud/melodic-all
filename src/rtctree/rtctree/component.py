# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtctree

Copyright (C) 2009-2014
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

Object representing a component node in the tree.

'''


import RTC
import SDOPackage
import time
import uuid

from rtctree.config_set import ConfigurationSet
from rtctree.exceptions import *
from rtctree.exec_context import ExecutionContext
from rtctree.node import TreeNode
import rtctree.sdo
from rtctree.ports import parse_port
from rtctree.utils import build_attr_string, nvlist_to_dict, dict_to_nvlist


##############################################################################
## Component node object

class Component(TreeNode):
    '''Node representing a component on a name server.

    Component nodes can occur below name server and directory nodes. They
    cannot contain any children.

    Component nodes store all the properties of a component, as well as
    references to the actual objects and object types used in the component.

    This node can be made dynamic by setting the 'dynamic' property to True.
    The following callbacks are available on this node type when it is dynamic:

    - rtc_status(ec_handle, status)
      The component's state has changed in the specified execution context. The
      new status is one of Component.INACTIVE, Component.ACTIVE,
      Component.ERROR, Component.UNKNOWN and Component.CREATED.
    - component_profile(items)
      One or more members of the component's profiles have been updated. The
      updated items are listed in the "items" argument.
    - ec_event(ec_handle, event)
      A change in one of the attached execution contexts has occurred. The
      event is one of Component.EC_ATTACHED, Component.EC_DETACHED,
      Component.EC_RATE_CHANGED, Component.EC_STARTUP, and
      Component.EC_SHUTDOWN.
    - port_event(port_name, event)
      A change on the specified port has occurred. The event is one of
      Component.PORT_ADD, Component.PORT_REMOVE, Component.PORT_CONNECT and
      Component.PORT_DISCONNECT.
    - config_event(config_set_name, event)
      A change in the component's configuration sets has occurred. The event is
      one of Component.CFG_UPDATE_SET, Component.CFG_UPDATE_PARAM,
      Component.CFG_UPDATE_PARAM_IN_ACTIVE, Component.CFG_ADD_SET,
      Component.CFG_REMOVE_SET and Component.CFG_ACTIVATE_SET.
    - heartbeat(time)
      A heartbeat was received from the component. The time the beat was
      received is passed.

    '''
    def __init__(self, name=None, parent=None, obj=None, *args, **kwargs):
        '''Constructor.

        @param name Name of this component (i.e. its entry in the path).
        @param parent The parent node of this node, if any.
        @param obj The CORBA LightweightRTObject object to wrap.

        '''
        self._obj = obj
        self._obs = None
        self._obs_id = None
        self._loggers = {}
        self._last_heartbeat = time.time() # RTC is alive at construction time
        super(Component, self).__init__(name=name, parent=parent,
                                        *args, **kwargs)
        self._set_events(['rtc_status', 'component_profile', 'ec_event',
            'port_event', 'config_event', 'heartbeat'])
        self._reset_data()
        self._parse_profile()

    def reparse(self):
        '''Reparse the component's information.

        This will cause a delayed reparse of most information. This means that
        a piece of information, such as the list of ports, will be cleared and
        remain empty until it is next requested, at which point a fresh list
        will be retrieved from the component.

        If you only want to reparse a specific piece of information, use one of
        the reparse_X() methods.

        '''
        self._reset_data()
        self._parse_profile()

    def reparse_conf_sets(self):
        '''Delayed reparse configuration sets.'''
        self._reset_conf_sets()

    def reparse_ecs(self):
        '''Delayed reparse all execution contexts.'''
        self.reparse_owned_ecs()
        self.reparse_participating_ecs()

    def reparse_owned_ecs(self):
        '''Delayed reparse only owned execution contexts.'''
        self._reset_owned_ecs()

    def reparse_participating_ecs(self):
        '''Delayed reparse only participating execution contexts.'''
        self._reset_participating_ecs()

    def reparse_ports(self):
        '''Delayed reparse ports.'''
        self._reset_ports()

    def reparse_profile(self):
        '''Delayed reparse the component's profile.'''
        self._parse_profile()

    ###########################################################################
    # Component information

    @property
    def category(self):
        '''The category in which the component belongs.'''
        with self._mutex:
            return self._category

    @property
    def description(self):
        '''The component's description.'''
        with self._mutex:
            return self._description

    @property
    def instance_name(self):
        '''Instance name of the component.'''
        with self._mutex:
            return self._instance_name

    @property
    def parent_object(self):
        '''The name of the component's parent object (typically another
        component), if it has one.

        '''
        with self._mutex:
            return self._parent_obj

    @property
    def properties(self):
        '''The component's extra properties dictionary.'''
        with self._mutex:
            return self._properties

    @property
    def type_name(self):
        '''Type name of the component.'''
        with self._mutex:
            return self._type_name

    @property
    def vendor(self):
        '''The component's vendor.'''
        with self._mutex:
            return self._vendor

    @property
    def version(self):
        '''The component's version.'''
        with self._mutex:
            return self._version

    ###########################################################################
    # Composite component information and management

    def add_members(self, rtcs):
        '''Add other RT Components to this composite component as members.

        This component must be a composite component.

        '''
        if not self.is_composite:
            raise NotCompositeError(self.name)
        for rtc in rtcs:
            if self.is_member(rtc):
                raise AlreadyInCompositionError(self.name, rtc.instance_name)
        org = self.organisations[0].obj
        org.add_members([x.object for x in rtcs])
        # Force a reparse of the member information
        self._orgs = []

    def remove_members(self, rtcs):
        '''Remove other RT Components from this composite component.

        rtcs is a list of components to remove. Each element must be either an
        rtctree.Component object or a string containing a component's instance
        name. rtctree.Component objects are more reliable.

        This component must be a composite component.

        '''
        if not self.is_composite:
            raise NotCompositeError(self.name)
        org = self.organisations[0].obj
        members = org.get_members()
        for rtc in rtcs:
            if type(rtc) == str:
                rtc_name = rtc
            else:
                rtc_name = rtc.instance_name
            # Check if the RTC actually is a member
            if not self.is_member(rtc):
                raise NotInCompositionError(self.name, rtc_name)
            # Remove the RTC from the composition
            org.remove_member(rtc_name)
        # Force a reparse of the member information
        self._orgs = []

    @property
    def composite_parent(self):
        '''The parent component in the composition.

        None if this component is not a member of a composition.

        '''
        return None

    @property
    def is_composite(self):
        '''Is the component a composite component.'''
        return self._obj.get_owned_organizations() != []

    @property
    def is_composite_member(self):
        '''Is the component a member of a composite component.'''
        return self._obj.get_organizations() != []

    def is_member(self, rtc):
        '''Is the given component a member of this composition?

        rtc may be a Component object or a string containing a component's
        instance name. Component objects are more reliable.

        Returns False if the given component is not a member of this
        composition.

        Raises NotCompositeError if this component is not a composition.

        '''
        if not self.is_composite:
            raise NotCompositeError(self.name)
        members = self.organisations[0].obj.get_members()
        if type(rtc) is str:
            for m in members:
                if m.get_component_profile().instance_name == rtc:
                    return True
        else:
            for m in members:
                if m._is_equivalent(rtc.object):
                    return True
        return False

    @property
    def members(self):
        '''Member components if this component is composite.'''
        with self._mutex:
            if not self._members:
                self._members = {}
                for o in self.organisations:
                    # TODO: Search for these in the tree
                    self._members[o.org_id] = o.obj.get_members()
        return self._members

    @property
    def organisations(self):
        '''The organisations of this composition.'''
        class Org:
            def __init__(self, sdo_id, org_id, members, obj):
                self.sdo_id = sdo_id
                self.org_id = org_id
                self.members = members
                self.obj = obj

        with self._mutex:
            if not self._orgs:
                for org in self._obj.get_owned_organizations():
                    owner = org.get_owner()
                    if owner:
                        sdo_id = owner._narrow(SDOPackage.SDO).get_sdo_id()
                    else:
                        sdo_id = ''
                    org_id = org.get_organization_id()
                    members = [m.get_sdo_id() for m in org.get_members()]
                    self._orgs.append(Org(sdo_id, org_id, members, org))
        return self._orgs

    @property
    def org_ids(self):
        '''The organisation IDs of this composition.'''
        return [sdo.get_organization_id() for sdo in \
                self._obj.get_owned_organizations()]

    @property
    def parent_org_ids(self):
        '''The organisation IDs of the compositions this RTC belongs to.'''
        return [sdo.get_organization_id() for sdo in \
                self._obj.get_organizations() if sdo]

    @property
    def parent_org_sdo_ids(self):
        '''The SDO IDs of the compositions this RTC belongs to.'''
        return [sdo.get_owner()._narrow(SDOPackage.SDO).get_sdo_id() \
                for sdo in self._obj.get_organizations() if sdo]

    @property
    def parent_organisations(self):
        '''The organisations this RTC belongs to.'''
        class ParentOrg:
            def __init__(self, sdo_id, org_id):
                self.sdo_id = sdo_id
                self.org_id = org_id

        with self._mutex:
            if not self._parent_orgs:
                for sdo in self._obj.get_organizations():
                    if not sdo:
                        continue
                    owner = sdo.get_owner()
                    if owner:
                        sdo_id = owner._narrow(SDOPackage.SDO).get_sdo_id()
                    else:
                        sdo_id = ''
                    org_id = sdo.get_organization_id()
                    self._parent_orgs.append(ParentOrg(sdo_id, org_id))
        return self._parent_orgs

    ###########################################################################
    # State management

    def exit(self):
        '''Stop the component's execution contexts and finalise it.

        This will have flow-on effects to any other components using this
        component's execution contexts and any child components.

        @return The result of attempting to exit.

        '''
        with self._mutex:
            return self._obj.exit()

    def activate_in_ec(self, ec_index):
        '''Activate this component in an execution context.

        @param ec_index The index of the execution context to activate in.
                        This index is into the total array of contexts, that
                        is both owned and participating contexts. If the value
                        of ec_index is greater than the length of
                        @ref owned_ecs, that length is subtracted from
                        ec_index and the result used as an index into
                        @ref participating_ecs.

        '''
        with self._mutex:
            if ec_index >= len(self.owned_ecs):
                ec_index -= len(self.owned_ecs)
                if ec_index >= len(self.participating_ecs):
                    raise BadECIndexError(ec_index)
                ec = self.participating_ecs[ec_index]
            else:
                ec = self.owned_ecs[ec_index]
            ec.activate_component(self._obj)

    def deactivate_in_ec(self, ec_index):
        '''Deactivate this component in an execution context.

        @param ec_index The index of the execution context to deactivate in.
                        This index is into the total array of contexts, that
                        is both owned and participating contexts. If the value
                        of ec_index is greater than the length of
                        @ref owned_ecs, that length is subtracted from
                        ec_index and the result used as an index into
                        @ref participating_ecs.

        '''
        with self._mutex:
            if ec_index >= len(self.owned_ecs):
                ec_index -= len(self.owned_ecs)
                if ec_index >= len(self.participating_ecs):
                    raise BadECIndexError(ec_index)
                ec = self.participating_ecs[ec_index]
            else:
                ec = self.owned_ecs[ec_index]
            ec.deactivate_component(self._obj)

    def exit(self):
        '''Make a component exit.

        This function will make the component exit, shutting down its CORBA
        object in the process. It will not remove the node from the tree; a
        reparse is necessary to do that.

        '''
        self._obj.exit()

    def get_ec(self, ec_handle):
        '''Get a reference to the execution context with the given handle.

        @param ec_handle The handle of the execution context to look for.
        @type ec_handle str
        @return A reference to the ExecutionContext object corresponding to
        the ec_handle.
        @raises NoECWithHandleError

        '''
        with self._mutex:
            for ec in self.owned_ecs:
                if ec.handle == ec_handle:
                    return ec
            for ec in self.participating_ecs:
                if ec.handle == ec_handle:
                    return ec
            raise NoECWithHandleError


    def get_ec_index(self, ec_handle):
        '''Get the index of the execution context with the given handle.

        @param ec_handle The handle of the execution context to look for.
        @type ec_handle str
        @return The index into the owned + participated arrays, suitable for
        use in methods such as @ref activate_in_ec, or -1 if the EC was not
        found.
        @raises NoECWithHandleError

        '''
        with self._mutex:
            for ii, ec in enumerate(self.owned_ecs):
                if ec.handle == ec_handle:
                    return ii
            for ii, ec in enumerate(self.participating_ecs):
                if ec.handle == ec_handle:
                    return ii + len(self.owned_ecs)
            raise NoECWithHandleError

    def get_state_string(self, add_colour=True):
        '''Get the state of this component as an optionally-coloured string.

        @param add_colour If True, ANSI colour codes will be added to the
                          string.
        @return A string describing the state of this component.

        '''
        with self._mutex:
            if self.state == self.INACTIVE:
                result = 'Inactive', ['bold', 'blue']
            elif self.state == self.ACTIVE:
                result = 'Active', ['bold', 'green']
            elif self.state == self.ERROR:
                result = 'Error', ['bold', 'white', 'bgred']
            elif self.state == self.UNKNOWN:
                result = 'Unknown', ['bold', 'red']
            elif self.state == self.CREATED:
                result = 'Created', ['reset']
        if add_colour:
            return build_attr_string(result[1], supported=add_colour) + \
                    result[0] + build_attr_string('reset', supported=add_colour)
        else:
            return result[0]

    def get_state_in_ec_string(self, ec_index, add_colour=True):
        '''Get the state of the component in an execution context as a string.

        @param ec_index The index of the execution context to check the state
                        in. This index is into the total array of contexts,
                        that is both owned and participating contexts. If the
                        value of ec_index is greater than the length of @ref
                        owned_ecs, that length is subtracted from ec_index and
                        the result used as an index into @ref
                        participating_ecs.

        '''
        with self._mutex:
            if ec_index >= len(self.owned_ecs):
                ec_index -= len(self.owned_ecs)
                if ec_index >= len(self.participating_ecs):
                    raise BadECIndexError(ec_index)
                state = self.participating_ec_states[ec_index]
            else:
                state = self.owned_ec_states[ec_index]
        if state == self.INACTIVE:
            result = 'Inactive', ['bold', 'blue']
        elif state == self.ACTIVE:
            result = 'Active', ['bold', 'green']
        elif state == self.ERROR:
            result = 'Error', ['bold', 'white', 'bgred']
        elif state == self.UNKNOWN:
            result = 'Unknown', ['bold', 'red']
        elif state == self.CREATED:
            result = 'Created', ['reset']
        if add_colour:
            return build_attr_string(result[1], supported=add_colour) + \
                    result[0] + build_attr_string('reset',
                    supported=add_colour)
        else:
            return result[0]

    def reset_in_ec(self, ec_index):
        '''Reset this component in an execution context.

        @param ec_index The index of the execution context to reset in. This
                        index is into the total array of contexts, that is both
                        owned and participating contexts. If the value of
                        ec_index is greater than the length of @ref owned_ecs,
                        that length is subtracted from ec_index and the result
                        used as an index into @ref participating_ecs.

        '''
        with self._mutex:
            if ec_index >= len(self.owned_ecs):
                ec_index -= len(self.owned_ecs)
                if ec_index >= len(self.participating_ecs):
                    raise BadECIndexError(ec_index)
                ec = self.participating_ecs[ec_index]
            else:
                ec = self.owned_ecs[ec_index]
            ec.reset_component(self._obj)

    def state_in_ec(self, ec_index):
        '''Get the state of the component in an execution context.

        @param ec_index The index of the execution context to check the state
                        in. This index is into the total array of contexts,
                        that is both owned and participating contexts. If the
                        value of ec_index is greater than the length of @ref
                        owned_ecs, that length is subtracted from ec_index and
                        the result used as an index into @ref
                        participating_ecs.

        '''
        with self._mutex:
            if ec_index >= len(self.owned_ecs):
                ec_index -= len(self.owned_ecs)
                if ec_index >= len(self.participating_ecs):
                    raise BadECIndexError(ec_index)
                return self.participating_ec_states[ec_index]
            else:
                return self.owned_ec_states[ec_index]

    def refresh_state_in_ec(self, ec_index):
        '''Get the up-to-date state of the component in an execution context.

        This function will update the state, rather than using the cached
        value. This may take time, if the component is executing on a remote
        node.

        @param ec_index The index of the execution context to check the state
                        in. This index is into the total array of contexts,
                        that is both owned and participating contexts. If the
                        value of ec_index is greater than the length of @ref
                        owned_ecs, that length is subtracted from ec_index and
                        the result used as an index into @ref
                        participating_ecs.

        '''
        with self._mutex:
            if ec_index >= len(self.owned_ecs):
                ec_index -= len(self.owned_ecs)
                if ec_index >= len(self.participating_ecs):
                    raise BadECIndexError(ec_index)
                state = self._get_ec_state(self.participating_ecs[ec_index])
                self.participating_ec_states[ec_index] = state
            else:
                state = self._get_ec_state(self.owned_ecs[ec_index])
                self.owned_ec_states[ec_index] = state
            return state

    @property
    def alive(self):
        '''Is this component alive?'''
        with self._mutex:
            if self.exec_contexts:
                for ec in self.exec_contexts:
                    if self._obj.is_alive(ec):
                        return True
        return False

    @property
    def owned_ec_states(self):
        '''The state of each execution context this component owns.'''
        with self._mutex:
            if not self._owned_ec_states:
                if self.owned_ecs:
                    states = []
                    for ec in self.owned_ecs:
                        states.append(self._get_ec_state(ec))
                    self._owned_ec_states = states
                else:
                    self._owned_ec_states = []
        return self._owned_ec_states

    @property
    def owned_ecs(self):
        '''A list of the execution contexts owned by this component.'''
        with self._mutex:
            if not self._owned_ecs:
                self._owned_ecs = [ExecutionContext(ec,
                    self._obj.get_context_handle(ec)) \
                    for ec in self._obj.get_owned_contexts()]
        return self._owned_ecs

    @property
    def participating_ec_states(self):
        '''The state of each execution context this component is participating
        in.

        '''
        with self._mutex:
            if not self._participating_ec_states:
                if self.participating_ecs:
                    states = []
                    for ec in self.participating_ecs:
                        states.append(self._get_ec_state(ec))
                    self._participating_ec_states = states
                else:
                    self._participating_ec_states = []
        return self._participating_ec_states

    @property
    def participating_ecs(self):
        '''A list of the execution contexts this component is participating in.

        '''
        with self._mutex:
            if not self._participating_ecs:
                self._participating_ecs = [ExecutionContext(ec,
                                    self._obj.get_context_handle(ec)) \
                             for ec in self._obj.get_participating_contexts()]
        return self._participating_ecs

    @property
    def plain_state_string(self):
        '''The state of this component as a string without colour.'''
        return self.get_state_string(add_colour=False)

    @property
    def state(self):
        '''The merged state of all the execution context states, which can be
        used as the overall state of this component.

        The order of precedence is:
            Error > Active > Inactive > Created > Unknown

        '''
        def merge_state(current, new):
            if new == self.ERROR:
                return self.ERROR
            elif new == self.ACTIVE and current != self.ERROR:
                return self.ACTIVE
            elif new == self.INACTIVE and \
                    current not in [self.ACTIVE, self.ERROR]:
                return self.INACTIVE
            elif new == self.CREATED and \
                    current not in [self.ACTIVE, self.ERROR, self.INACTIVE]:
                return self.CREATED
            elif current not in [self.ACTIVE, self.ERROR, self.INACTIVE,
                                 self.CREATED]:
                return self.UNKNOWN
            return current

        with self._mutex:
            if not self.owned_ec_states and not self.participating_ec_states:
                return self.UNKNOWN
            merged_state = self.CREATED
            if self.owned_ec_states:
                for ec_state in self.owned_ec_states:
                    merged_state = merge_state(merged_state, ec_state)
            if self.participating_ec_states:
                for ec_state in self.participating_ec_states:
                    merged_state = merge_state(merged_state, ec_state)
            return merged_state

    @property
    def state_string(self):
        '''The state of this component as a colourised string.'''
        return self.get_state_string()

    ###########################################################################
    # Port management

    def disconnect_all(self):
        '''Disconnect all connections to all ports of this component.'''
        with self._mutex:
            for p in self.ports:
                p.disconnect_all()

    def get_port_by_name(self, port_name):
        '''Get a port of this component by name.'''
        with self._mutex:
            for p in self.ports:
                if p.name == port_name:
                    return p
            return None

    def get_port_by_ref(self, port_ref):
        '''Get a port of this component by reference to a CORBA PortService
        object.

        '''
        with self._mutex:
            for p in self.ports:
                if p.object._is_equivalent(port_ref):
                    return p
            return None

    def has_port_by_name(self, port_name):
        '''Check if this component has a port by the given name.'''
        with self._mutex:
            if self.get_port_by_name(port_name):
                return True
            return False

    def has_port_by_ref(self, port_ref):
        '''Check if this component has a port by the given reference to a CORBA
        PortService object.

        '''
        with self._mutex:
            if self.get_port_by_ref(self, port_ref):
                return True
            return False

    @property
    def connected_inports(self):
        '''The list of all input ports belonging to this component that are
        connected to one or more other ports.

        '''
        return [p for p in self.ports \
                if p.__class__.__name__ == 'DataInPort' and p.is_connected]

    @property
    def connected_outports(self):
        '''The list of all output ports belonging to this component that are
        connected to one or more other ports.

        '''
        return [p for p in self.ports \
                    if p.__class__.__name__ == 'DataOutPort' \
                    and p.is_connected]

    @property
    def connected_ports(self):
        '''The list of all ports belonging to this component that are connected
        to one or more other ports.

        '''
        return [p for p in self.ports if p.is_connected]

    @property
    def connected_svcports(self):
        '''The list of all service ports belonging to this component that are
        connected to one or more other ports.

        '''
        return [p for p in self.ports \
                if p.__class__.__name__ == 'CorbaPort' and p.is_connected]

    @property
    def inports(self):
        '''The list of all input ports belonging to this component.'''
        return [p for p in self.ports if p.__class__.__name__ == 'DataInPort']

    @property
    def outports(self):
        '''The list of all output ports belonging to this component.'''
        return [p for p in self.ports if p.__class__.__name__ == 'DataOutPort']

    @property
    def ports(self):
        '''The list of all ports belonging to this component.'''
        with self._mutex:
            if not self._ports:
                self._ports = [parse_port(port, self) \
                               for port in self._obj.get_ports()]
        return self._ports

    @property
    def svcports(self):
        '''The list of all service ports belonging to this component.'''
        return [p for p in self.ports if p.__class__.__name__ == 'CorbaPort']

    ###########################################################################
    # Node functionality

    @property
    def heartbeat_time(self):
        '''The time of the last heartbeat.

        Updated only when the node is dynamic.

        '''
        return self._last_heartbeat

    @property
    def is_component(self):
        '''Is this node a component?'''
        return True

    @property
    def loggers(self):
        '''Returns the list of logger IDs attached to this component.'''
        return self._loggers.keys()

    @property
    def object(self):
        '''The LightweightRTObject this object wraps.'''
        with self._mutex:
            return self._obj

    def add_logger(self, cb, level='NORMAL', filters='ALL'):
        '''Add a callback to receive log events from this component.

        @param cb The callback function to receive log events. It must have the
            signature cb(name, time, source, level, message), where name is the
            name of the component the log record came from, time is a
            floating-point time stamp, source is the name of the logger that
            provided the log record, level is the log level of the record and
            message is a text string.
        @param level The maximum level of log records to receive.
        @param filters Filter the objects from which to receive log messages.
        @return An ID for this logger. Use this ID in future operations such as
                removing this logger.
        @raises AddLoggerError

        '''
        with self._mutex:
            obs = rtctree.sdo.RTCLogger(self, cb)
            uuid_val = uuid.uuid4()
            intf_type = obs._this()._NP_RepositoryId
            props = {'logger.log_level': level,
                    'logger.filter': filters}
            props = dict_to_nvlist(props)
            sprof = SDOPackage.ServiceProfile(id=uuid_val.get_bytes(),
                    interface_type=intf_type, service=obs._this(),
                    properties=props)
            conf = self.object.get_configuration()
            res = conf.add_service_profile(sprof)
            if res:
                self._loggers[uuid_val] = obs
                return uuid_val
            raise AddLoggerError(self.name)

    def remove_logger(self, cb_id):
        '''Remove a logger.

        @param cb_id The ID of the logger to remove.
        @raises NoLoggerError

        '''
        if cb_id not in self._loggers:
            raise NoLoggerError(cb_id, self.name)
        conf = self.object.get_configuration()
        res = conf.remove_service_profile(cb_id.get_bytes())
        del self._loggers[cb_id]

    ###########################################################################
    # Configuration set management

    def activate_conf_set(self, set_name):
        '''Activate a configuration set by name.

        @raises NoSuchConfSetError

        '''
        with self._mutex:
            if not set_name in self.conf_sets:
                raise NoSuchConfSetError(set_name)
            self._conf.activate_configuration_set(set_name)

    def set_conf_set_value(self, set_name, param, value):
        '''Set a configuration set parameter value.

        @param set_name The name of the configuration set the destination
                        parameter is in.
        @param param The name of the parameter to set.
        @param value The new value for the parameter.
        @raises NoSuchConfSetError, NoSuchConfParamError

        '''
        with self._mutex:
            if not set_name in self.conf_sets:
                raise NoSuchConfSetError(set_name)
            if not self.conf_sets[set_name].has_param(param):
                raise NoSuchConfParamError(param)
            self.conf_sets[set_name].set_param(param, value)
            self._conf.set_configuration_set_values(\
                    self.conf_sets[set_name].object)

    @property
    def active_conf_set(self):
        '''The currently-active configuration set.'''
        with self._mutex:
            if not self.conf_sets:
                return None
            if not self._active_conf_set:
                return None
            return self.conf_sets[self._active_conf_set]

    @property
    def active_conf_set_name(self):
        '''The name of the currently-active configuration set.'''
        with self._mutex:
            if not self.conf_sets:
                return ''
            if not self._active_conf_set:
                return ''
            return self._active_conf_set

    @property
    def conf_sets(self):
        '''The dictionary of configuration sets in this component, if any.'''
        with self._mutex:
            if not self._conf_sets:
                self._parse_configuration()
        return self._conf_sets

    ###########################################################################
    # Internal API

    def _add_child(self):
        # Components cannot contain children.
        raise CannotHoldChildrenError

    def _config_event(self, name, event):
        with self._mutex:
            if self._conf_sets:
                if event == self.CFG_UPDATE_SET:
                    # A configuration set has been updated
                    cs = self._conf.get_configuration_set(name)
                    self._conf_sets[name]._reload(cs, cs.description,
                            nvlist_to_dict(cs.configuration_data))
                elif event == self.CFG_UPDATE_PARAM:
                    # A parameter in a configuration set has been changed
                    cset, param = name.split('.')
                    cs = self._conf.get_configuration_set(cset)
                    data = nvlist_to_dict(cs.configuration_data)
                    self._conf_sets[cset].set_param(param, data[param])
                elif event == self.CFG_ADD_SET:
                    # A new configuration set has been added
                    cs = self._conf.get_configuration_set(name)
                    self._conf_sets[name] = ConfigurationSet(self, cs,
                            cs.description,
                            nvlist_to_dict(cs.configuration_data))
                elif event == self.CFG_REMOVE_SET:
                    # Remove the configuration set
                    del self._conf_sets[name]
                elif event == self.CFG_ACTIVATE_SET:
                    # Change the active configuration set
                    self._active_conf_set = name
        # Call callbacks outside the mutex
        self._call_cb('config_event', (name, event))

    def _enable_dynamic(self, enable=True):
        if enable:
            obs = rtctree.sdo.RTCObserver(self)
            uuid_val = uuid.uuid4().get_bytes()
            intf_type = obs._this()._NP_RepositoryId
            props = dict_to_nvlist({'heartbeat.enable': 'YES',
                'heartbeat.interval': '1.0',
                'observed_status': 'ALL'})
            sprof = SDOPackage.ServiceProfile(id=uuid_val,
                    interface_type=intf_type, service=obs._this(),
                    properties=props)
            conf = self.object.get_configuration()
            res = conf.add_service_profile(sprof)
            if res:
                self._dynamic = True
                self._obs = obs
                self._obs_id = uuid_val
                # If we could set an observer, the component is alive
                self._last_heartbeat = time.time()
        else: # Disable
            conf = self.object.get_configuration()
            res = conf.remove_service_profile(self._obs_id)
            if res:
                self._dynamic = False
                self._obs = None
                self._obs_id = None

    def _ec_event(self, ec_handle, event):
        def get_ec(ec_handle):
            tgt_ec = None
            loc = None
            if self._owned_ecs:
                for ec in self._owned_ecs:
                    if ec.handle == ec_handle:
                        tgt_ec = ec
                        loc = self._owned_ecs
                        break
            if not del_ec and self._participating_ecs:
                for ec in self._participating_ecs:
                    if ec.handle == ec_handle:
                        tgt_ec = ec
                        loc = self._participating_ecs
                        break
            return tgt_ec, loc

        with self._mutex:
            if event == self.EC_ATTACHED:
                # New EC has been attached
                self._participating_ecs.append(ExecutionContext(
                    self._obj.get_context(ec_handle), ec_handle))
            elif event == self.EC_DETACHED:
                # An EC has been detached; delete the local facade
                # if ec is not None, the corresponding EC has a local
                # facade that needs deleting
                ec, loc = get_ec(ec_handle)
                if ec:
                    loc.remove(ec)
            elif event == self.EC_RATE_CHANGED:
                # Nothing to do
                pass
            elif event == self.EC_STARTUP:
                ec, loc = get_ec(ec_handle)
                if ec:
                    ec._set_running(True)
            elif event == self.EC_SHUTDOWN:
                ec, loc = get_ec(ec_handle)
                if ec:
                    ec._set_running(False)
        # Call callbacks outside the mutex
        self._call_cb('ec_event', (ec_handle, state))

    def _get_ec_state(self, ec):
        # Get the state of this component in an EC and return the enum value.
        if self._obj.is_alive(ec._obj):
            ec_state = ec.get_component_state(self._obj)
            if ec_state == RTC.ACTIVE_STATE:
                return self.ACTIVE
            elif ec_state == RTC.ERROR_STATE:
                return self.ERROR
            elif ec_state == RTC.INACTIVE_STATE:
                return self.INACTIVE
            else:
                return self.UNKNOWN
        else:
            return self.CREATED

    def _heartbeat(self):
        # Received a heart beat
        self._last_heartbeat = time.time()
        self._call_cb('heartbeat', self._last_heartbeat)

    def _parse_configuration(self):
        # Parse the component's configuration sets
        with self._mutex:
            self._conf = self.object.get_configuration()
            self._conf_sets = {}
            for cs in self._conf.get_configuration_sets():
                self._conf_sets[cs.id] = ConfigurationSet(self, cs, cs.description,
                        nvlist_to_dict(cs.configuration_data))
            try:
                self._active_conf_set = self._conf.get_active_configuration_set().id
            except SDOPackage.NotAvailable:
                self._active_conf_set = ''

    def _parse_profile(self):
        # Parse the component's profile
        with self._mutex:
            profile = self._obj.get_component_profile()
            self._instance_name = profile.instance_name
            self._type_name = profile.type_name
            self._description = profile.description
            self._version = profile.version
            self._vendor = profile.vendor
            self._category = profile.category
            if profile.parent:
                self._parent_obj = \
                        profile.parent.get_component_profile().instance_name
            else:
                self._parent_obj = ''
            self._properties = nvlist_to_dict(profile.properties)

    def _port_event(self, port_name, event):
        def get_port_obj(port_name):
            for p_obj in self._obj.get_ports():
                prof = p_obj.get_port_profile()
                if prof.name == port_name:
                    return p_obj
            raise ValueError(port_name)

        with self._mutex:
            if self._ports:
                if event == self.PORT_ADD:
                    # New port
                    p_obj = get_port_obj(port_name)
                    self._ports.append(parse_port(p_obj, self))
                elif event == self.PORT_REMOVE:
                    # Port removed
                    p = self.get_port_by_name(port_name)
                    self._ports.remove(p)
                elif event == self.PORT_CONNECT:
                    # A port has a new connection
                    p = self.get_port_by_name(port_name)
                    p.reparse_connections()
                elif event == self.PORT_DISCONNECT:
                    # A port has had a connection removed
                    p = self.get_port_by_name(port_name)
                    p.reparse_connections()
        # Call callbacks outside the mutex
        self._call_cb('port_event', (port_name, event))

    def _profile_update(self, items):
        # Reparse the profile
        self._parse_profile()
        # Call callbacks outside the mutex
        self._call_cb('component_profile', items)

    def _reset_conf_sets(self):
        with self._mutex:
            self._conf_sets = None
            self._active_conf_set = None

    def _reset_data(self):
        self._reset_owned_ecs()
        self._reset_participating_ecs()
        self._reset_ports()
        self._reset_conf_sets()
        self._reset_composite()

    def _reset_owned_ecs(self):
        with self._mutex:
            self._owned_ecs = None
            self._owned_ec_states = None

    def _reset_owned_ec_states(self):
        with self._mutex:
            self._owned_ec_states = None

    def _reset_participating_ecs(self):
        with self._mutex:
            self._participating_ecs = None
            self._participating_ec_states = None

    def _reset_participating_ec_states(self):
        with self._mutex:
            self._participating_ec_states = None

    def _reset_ports(self):
        with self._mutex:
            self._ports = None

    def _reset_composite(self):
        with self._mutex:
            self._orgs = []
            self._parent_orgs = []
            self._members = {}

    def _set_state_in_ec(self, ec_handle, state):
        # Forcefully set the state of this component in an EC
        with self._mutex:
            if ec_handle >= len(self.owned_ecs):
                ec_handle -= len(self.owned_ecs)
                if ec_handle >= len(self.participating_ecs):
                    raise BadECIndexError(ec_handle)
                self.participating_ec_states[ec_handle] = state
            else:
                self.owned_ec_states[ec_handle] = state
        # Call callbacks outside the mutex
        self._call_cb('rtc_status', (ec_handle, state))

    # Constant for a component in the inactive state
    INACTIVE = 1
    # Constant for a component in the active state
    ACTIVE = 2
    # Constant for a component in the error state
    ERROR = 3
    # Constant for a component in an unknown state
    UNKNOWN = 4
    # Constant for a component in the created state
    CREATED = 5

    # Constant for execution context event "attached"
    EC_ATTACHED = 11
    # Constant for execution context event "detached"
    EC_DETACHED = 12
    # Constant for execution context event "rate_changed"
    EC_RATE_CHANGED = 13
    # Constant for execution context event "startup"
    EC_STARTUP = 14
    # Constant for execution context event "shutdown"
    EC_SHUTDOWN = 15

    # Constant for port event 'add'
    PORT_ADD = 21
    # Constant for port event 'remove'
    PORT_REMOVE = 22
    # Constant for port event 'connect'
    PORT_CONNECT = 23
    # Constant for port event 'disconnect'
    PORT_DISCONNECT = 24

    # Constant for configuration set event 'update_set'
    CFG_UPDATE_SET = 31
    # Constant for configuration set event 'update_param'
    CFG_UPDATE_PARAM = 32
    # Constant for configuration set event 'set_active_set'
    CFG_SET_SET = 33
    # Constant for configuration set event 'add_set'
    CFG_ADD_SET = 34
    # Constant for configuration set event 'remove_set'
    CFG_REMOVE_SET = 35
    # Constant for configuration set event 'activate_set'
    CFG_ACTIVATE_SET = 36


# vim: tw=79

