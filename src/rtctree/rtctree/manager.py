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

Object representing a manager node in the tree.

'''


from omniORB import CORBA, TRANSIENT_ConnectFailed, UNKNOWN_UserException
import os.path
import sys

from rtctree.component import Component
from rtctree.exceptions import FailedToLoadModuleError, \
                               FailedToCreateComponentError, \
                               FailedToDeleteComponentError, \
                               FailedToSetConfigurationError, \
                               FailedToAddMasterManagerError, \
                               FailedToRemoveMasterManagerError, \
                               FailedToAddSlaveManagerError, \
                               FailedToRemoveSlaveManagerError
from rtctree.node import TreeNode
from rtctree.utils import nvlist_to_dict
import RTC


##############################################################################
## Manager node object

class Manager(TreeNode):
    '''Node representing a manager on a name server.

    Manager nodes can occur below name server and directory nodes. They may
    store child components and child managers. They can be used to add and
    remove new components and managers to the tree at run time.

    '''
    def __init__(self, name=None, parent=None, obj=None, *args, **kwargs):
        '''Constructor. Calls the TreeNode constructor.'''
        super(Manager, self).__init__(name=name, parent=parent, *args,
                                      **kwargs)
        self._obj = obj
        self._parse()

    ##########################################################################
    # Module and component management

    def create_component(self, module_name):
        '''Create a component out of a loaded module.

        Turns a previously-loaded shared module into a component in the
        manager. This will invalidate any objects that are children of this
        node.

        The @ref module_name argument can contain options that set various
        properties of the new component. These must be appended to the module
        name, prefixed by a question mark for each property, in key=value
        format. For example, to change the instance name of the new component,
        append '?instance_name=new_name' to the module name.

        @param module_name Name of the module to turn into a component.
        @raises FailedToCreateComponentError

        '''
        with self._mutex:
            if not self._obj.create_component(module_name):
                raise FailedToCreateComponentError(module_name)
            # The list of child components will have changed now, so it must be
            # reparsed.
            self._parse_component_children()

    def delete_component(self, instance_name):
        '''Delete a component.

        Deletes the component specified by @ref instance_name from the manager.
        This will invalidate any objects that are children of this node.

        @param instance_name The instance name of the component to delete.
        @raises FailedToDeleteComponentError

        '''
        with self._mutex:
            if self._obj.delete_component(instance_name) != RTC.RTC_OK:
                raise FailedToDeleteComponentError(instance_name)
            # The list of child components will have changed now, so it must be
            # reparsed.
            self._parse_component_children()

    def load_module(self, path, init_func):
        '''Load a shared library.

        Call this function to load a shared library (DLL file under Windows,
        shared object under UNIX) into the manager.

        @param path The path to the shared library.
        @param init_func The name entry function in the library.
        @raises FailedToLoadModuleError

        '''
        try:
            with self._mutex:
                if self._obj.load_module(path, init_func) != RTC.RTC_OK:
                    raise FailedToLoadModuleError(path)
        except CORBA.UNKNOWN, e:
            if e.args[0] == UNKNOWN_UserException:
                raise FailedToLoadModuleError(path, 'CORBA User Exception')
            else:
                raise

    def unload_module(self, path):
        '''Unload a loaded shared library.

        Call this function to remove a shared library (e.g. a component) that
        was previously loaded.

        @param path The path to the shared library.
        @raises FailedToUnloadModuleError

        '''
        with self._mutex:
            if self._obj.unload_module(path) != RTC.RTC_OK:
                raise FailedToUnloadModuleError(path)

    @property
    def components(self):
        '''The list of components in this manager, if any.

        This information can also be found by listing the children of this node
        that are of type @ref Component. That method is more useful as it returns
        the tree entries for the components.

        '''
        with self._mutex:
            if not self._components:
                self._components = [c for c in self.children if c.is_component]
        return self._components

    @property
    def factory_profiles(self):
        '''The factory profiles of all loaded modules.'''
        with self._mutex:
            if not self._factory_profiles:
                self._factory_profiles = []
                for fp in self._obj.get_factory_profiles():
                    self._factory_profiles.append(nvlist_to_dict(fp.properties))
        return self._factory_profiles

    ##########################################################################
    # Manager configuration

    def set_config_parameter(self, param, value):
        '''Set a configuration parameter of the manager.

        @param The parameter to set.
        @value The new value for the parameter.
        @raises FailedToSetConfigurationError

        '''
        with self._mutex:
            if self._obj.set_configuration(param, value) != RTC.RTC_OK:
                raise FailedToSetConfigurationError(param, value)
            # Force a reparse of the configuration
            self._configuration = None

    @property
    def configuration(self):
        '''The configuration dictionary of the manager.'''
        with self._mutex:
            if not self._configuration:
                self._configuration = nvlist_to_dict(self._obj.get_configuration())
        return self._configuration

    @property
    def profile(self):
        '''The manager's profile.'''
        with self._mutex:
            if not self._profile:
                profile = self._obj.get_profile()
                self._profile = nvlist_to_dict(profile.properties)
        return self._profile

    ##########################################################################
    # Undocumented functions

    def fork(self):
        '''Fork the manager.'''
        with self._mutex:
            self._obj.fork()

    def shutdown(self):
        '''Shut down the manager.'''
        with self._mutex:
            self._obj.shutdown()

    def restart(self):
        '''Restart the manager.'''
        with self._mutex:
            self._obj.restart()

    ##########################################################################
    # Node functionality

    @property
    def is_directory(self):
        '''Is this node a directory?'''
        return True

    @property
    def is_manager(self):
        '''Is this node a manager?'''
        return True


    @property
    def object(self):
        '''The RTM::Manager object that this node contains.'''
        with self._mutex:
            return self._obj

    @property
    def is_master(self):
        '''Is this manager node a master manager?

        Master managers have a direct presence on the name server. Slave
        managers are only present as children of other managers.

        '''
        with self._mutex:
            return self._obj.is_master()

    @property
    def loadable_modules(self):
        '''The list of loadable module profile dictionaries.'''
        with self._mutex:
            if not self._loadable_modules:
                self._loadable_modules = []
                for mp in self._obj.get_loadable_modules():
                    self._loadable_modules.append(nvlist_to_dict(mp.properties))
        return self._loadable_modules

    @property
    def loaded_modules(self):
        '''The list of loaded module profile dictionaries.'''
        with self._mutex:
            if not self._loaded_modules:
                self._loaded_modules = []
                for mp in self._obj.get_loaded_modules():
                    self._loaded_modules.append(nvlist_to_dict(mp.properties))
        return self._loaded_modules

    @property
    def masters(self):
        '''The list of master managers of this manager, if any.

        If this manager is a master, this list will be empty.

        '''
        with self._mutex:
            if not self._masters:
                raise NotImplementedError
        return self._masters

    @property
    def slaves(self):
        '''The list of slave managers of this manager, if any.

        This information can also be found by listing the children of this node
        that are of type @ref Manager.

        '''
        with self._mutex:
            if not self._slaves:
                self._slaves = [c for c in self.children if c.is_manager]
        return self._slaves

    ##########################################################################
    # Internal functions

    def _add_master(self, new_master):
        # Add a new master to this manager. A slave manager can have multiple
        # masters. new_master should be a rtctree.manager.Manager object.
        with self._mutex:
            if self._obj.add_master_manager(new_master.object) != RTC.RTC_OK:
                raise FailedToAddMasterManagerError

    def _add_slave(self, new_slave):
        # Add a slave to this manager. Master managers can hold slave managers,
        # which appear as child nodes in the tree. It will appear in the tree
        # as a new child node of this manager's node if the tree is reparsed.
        # new_slave should be a rtctree.manager.Manager object.
        with self._mutex:
            if self._obj.add_save_manager(new_slave.object) != RTC.RTC_OK:
                raise FailedToAddSlaveManagerError(self.name, new_slave.name)

    def _parse(self):
        # Nearly everything is delay-parsed when it is first accessed.
        with self._mutex:
            self._components = None
            self._configuration = None
            self._profile = None
            self._factory_profiles = None
            self._loadable_modules = None
            self._loaded_modules = None
            self._masters = None
            self._slaves = None
            self._parse_children()

    def _parse_children(self):
        # Parses child managers and components.
        with self._mutex:
            self._parse_component_children()
            self._parse_manager_children()

    def _parse_component_children(self):
        # Parses the list returned by _obj.get_components into child nodes.
        with self._mutex:
            try:
                comps = self._obj.get_components()
            except CORBA.BAD_PARAM, e:
                print >>sys.stderr, '{0}: {1}'.format(
                        os.path.basename(sys.argv[0]), e)
                return
            for c in comps:
                # Get the instance profile - this will be the node's name
                profile = c.get_component_profile()
                instance_name = profile.instance_name
                # Create and store the new leaf node
                leaf = Component(instance_name + '.rtc', self, c)
                self._add_child(leaf)

    def _parse_manager_children(self):
        # Parses the list returned by _obj.get_slave_managers into child nodes.
        with self._mutex:
            try:
                mgrs = self._obj.get_slave_managers()
            except CORBA.BAD_OPERATION:
                # This manager does not support slave managers; ignore
                return
            index = 0
            for m in mgrs:
                # Add each slave manager as a child node.
                try:
                    props = nvlist_to_dict(m.get_profile().properties)
                except CORBA.TRANSIENT, e:
                    if e.args[0] == TRANSIENT_ConnectFailed:
                        print >>sys.stderr, '{0}: Warning: zombie slave of '\
                                'manager {1} found'.format(sys.argv[0],
                                        self.name)
                        continue
                    else:
                        raise
                if 'name' in props:
                    name = props['name']
                else:
                    name = 'slave{0}'.format(index)
                    index += 1
                leaf = Manager(name, self, m)
                self._add_child(leaf)

    def _remove_master(self, master):
        # Remove a new master from this manager. A slave manager can have multiple
        # masters. new_master should be a rtctree.manager.Manager object.
        with self._mutex:
            if self._obj.remove_master_manager(master.object) != RTC.RTC_OK:
                raise FailedToRemoveMasterManagerError

    def _remove_slave(self, slave):
        # Remove a slave from this manager. Master managers can hold slave
        # managers, which appear as child nodes in the tree. slave should be a
        # rtctree.manager.Manager object.
        with self._mutex:
            if self._obj.remove_slave_manager(slave.object) != RTC.RTC_OK:
                raise FailedToRemoveSlaveManagerError(self.name, slave.name)

    def _set_parent(self, new_parent):
        # When setting the parent of a manager node, we need to tell wrapped
        # object that it has a new master. If our old parent was a master, then
        # we need to remove ourselves from that one.
        # Note that rtctree assumes a singly-linked hierarchy of managers.
        with self._mutex:
            if self.parent:
                if self.parent.is_manager:
                    self.parent._remove_slave(self)
                    self._remove_master(self.parent)
            self._add_master(new_parent)
            new_parent._add_slave(self)
            self.parent = new_parent


# vim: tw=79

