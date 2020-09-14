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

Object representing a directory node in the tree.

'''


from copy import deepcopy
import CosNaming
from omniORB import URI, CORBA, TRANSIENT_ConnectFailed
import sys

from rtctree.component import Component
from rtctree.exceptions import BadPathError
from rtctree.manager import Manager
from rtctree.node import TreeNode
from rtctree.options import Options
from rtctree.unknown import Unknown
from rtctree.zombie import Zombie
from rtctree.utils import filtered, trim_filter
import RTC
import RTM


##############################################################################
## Directory node object

class Directory(TreeNode):
    '''Node representing a naming context on a name server.

    Name servers contain contexts (including the root context) and objects. For
    us, contexts are directories and objects are managers and components. A
    directory context may specialise as a name server context, in which case
    it represents the root context of a name server.

    '''
    def __init__(self, name=None, parent=None, children=None, filter=[], *args,
            **kwargs):
        '''Constructor. Calls the TreeNode constructor.'''
        super(Directory, self).__init__(name=name, parent=parent,
                children=children, filter=filter, *args, **kwargs)

    def reparse(self):
        '''Reparse all children of this directory.

        This effectively rebuilds the tree below this node.

        This operation takes an unbounded time to complete; if there are a lot
        of objects registered below this directory's context, they will all
        need to be parsed.

        '''
        self._remove_all_children()
        self._parse_context(self._context, self.orb)

    def unbind(self, name):
        '''Unbind an object from the context represented by this directory.

        Warning: this is a dangerous operation. You may unlink an entire
        section of the tree and be unable to recover it. Be careful what you
        unbind.

        The name should be in the format used in paths. For example,
        'manager.mgr' or 'ConsoleIn0.rtc'.

        '''
        with self._mutex:
            id, sep, kind = name.rpartition('.')
            if not id:
                id = kind
                kind = ''
            name = CosNaming.NameComponent(id=str(id), kind=str(kind))
            try:
                self.context.unbind([name])
            except CosNaming.NamingContext.NotFound:
                raise BadPathError(name)

    @property
    def context(self):
        '''The object representing this naming context.'''
        with self._mutex:
            return self._context

    @property
    def is_directory(self):
        '''Is this node a directory?'''
        return True

    def _parse_context(self, context, orb, filter=[]):
        with self._mutex:
            # Parse a naming context to fill in the children.
            self._context = context
            # Get the list of bindings from the context
            bindings, bindings_it = context.list(Options().\
                                        get_option('max_bindings'))
            for binding in bindings:
                # Process the bindings that are within max_bindings
                self._process_binding(binding, orb, filter)
            if bindings_it:
                # Handle the iterator containing the remaining bindings
                remaining, bindings = bindings_it.next_n(Options().\
                                            get_option('max_bindings'))
                while remaining:
                    for binding in bindings:
                        self._process_binding(binding, orb, filter)
                    remaining, binding = bindings_it.next_n(Options().\
                                                get_option('max_bindings'))
                bindings_it.destroy()

    def _process_binding(self, binding, orb, filter):
        if filtered([corba_name_to_string(binding.binding_name)], filter):
            # Do not pass anything which does not pass the filter
            return
        trimmed_filter = trim_filter(deepcopy(filter))
        with self._mutex:
            # Process a binding, creating the correct child type for it and
            # adding that child to this node's children.
            if binding.binding_type == CosNaming.nobject:
                # This is a leaf node; either a component or a manager.  The
                # specific type can be determined from the binding name kind.
                if binding.binding_name[0].kind == 'mgr':
                    name = corba_name_to_string(binding.binding_name)
                    obj = self._context.resolve(binding.binding_name)
                    if not obj:
                        leaf = Zombie(name, self)
                        return
                    obj = obj._narrow(RTM.Manager)
                    try:
                        leaf = Manager(name, self, obj, dynamic=self.dynamic)
                    except CORBA.OBJECT_NOT_EXIST:
                        # Manager zombie
                        leaf = Zombie(name, self)
                    except CORBA.TRANSIENT:
                        # Manager zombie
                        leaf = Zombie(name, self)
                    self._add_child(leaf)
                elif binding.binding_name[0].kind == 'rtc':
                    name = corba_name_to_string(binding.binding_name)
                    obj = self._context.resolve(binding.binding_name)
                    try:
                        obj = obj._narrow(RTC.RTObject)
                    except CORBA.TRANSIENT, e:
                        if e.args[0] == TRANSIENT_ConnectFailed:
                            self._add_child(Zombie(name, self))
                            return
                        else:
                            raise
                    except CORBA.OBJECT_NOT_EXIST:
                        self._add_child(Zombie(name, self))
                        return
                    try:
                        leaf = Component(name, self, obj, dynamic=self.dynamic)
                    except CORBA.OBJECT_NOT_EXIST:
                        # Component zombie
                        leaf = Zombie(name, self, dynamic=self.dynamic)
                    except CORBA.TRANSIENT, e:
                        if e.args[0] == TRANSIENT_ConnectFailed:
                            self._add_child(Zombie(name, self))
                            return
                        else:
                            raise
                    self._add_child(leaf)
                else:
                    # Unknown type - add a plain node
                    name = corba_name_to_string(binding.binding_name)
                    obj = self._context.resolve(binding.binding_name)
                    leaf = Unknown(name, self, obj)
                    self._add_child(leaf)
            else:
                # This is a context, and therefore a subdirectory.
                subdir_name = corba_name_to_string(binding.binding_name)
                subdir = Directory(subdir_name, self, filter=trimmed_filter,
                        dynamic=self.dynamic)
                subdir_context = self._context.resolve(binding.binding_name)
                subdir_context = subdir_context._narrow(CosNaming.NamingContext)
                subdir._parse_context(subdir_context, orb,
                        filter=trimmed_filter)
                self._add_child(subdir)


def corba_name_to_string(name):
    '''Convert a CORBA CosNaming.Name to a string.'''
    parts = []
    if type(name) is not list and type(name) is not tuple:
        raise NotCORBANameError(name)
    if len(name) == 0:
        raise NotCORBANameError(name)

    for nc in name:
        if not nc.kind:
            parts.append(nc.id)
        else:
            parts.append('{0}.{1}'.format(nc.id, nc.kind))
    return '/'.join(parts)


# vim: tw=79

