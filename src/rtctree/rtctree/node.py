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

Object representing a generic node in the tree.

'''


import threading

from rtctree.exceptions import NotRelatedError, NoSuchEventError


##############################################################################
## Base node object

class TreeNode(object):
    '''Base class for a node in the tree.

    Do not create this class directly. Create objects using a suitable child
    class of this class.

    '''
    def __init__(self, name=None, parent=None, children=None, filter=[],
            dynamic=False, *args, **kwargs):
        '''Constructor.

        @param name Name of this node (i.e. its entry in the path).
        @param parent The parent node of this node, if any.
        @param children If the list of children is already known, put it here.
        @param filter A list of paths to filter by.
        @param dynamic Enable dynamic features such as observers on this node
                       and any children it creates.

        '''
        super(TreeNode, self).__init__(*args, **kwargs)
        self._mutex = threading.RLock()
        self._name = name
        self._parent = parent
        if children:
            self._children = children
        else:
            self._children = {}
        self._cbs = {}
        self._dynamic = dynamic
        if dynamic:
            self._enable_dynamic(dynamic)

    def __str__(self):
        '''Get this node as a string.'''
        with self._mutex:
            indent = ''.rjust(self.depth)
            result = '{0}{1}, {2}\n'.format(indent, self._name, self._children)
            for child in self._children:
                result += str(self._children[child])
        return result

    def add_callback(self, event, cb, args=None):
        '''Add a callback to this node.

        Callbacks are called when the specified event occurs. The available
        events depends on the specific node type. Args should be a value to
        pass to the callback when it is called. The callback should be of the
        format:

        def callback(node, value, cb_args):

        where node will be the node that called the function, value is the
        relevant information for the event, and cb_args are the arguments you
        registered with the callback.

        '''
        if event not in self._cbs:
            raise exceptions.NoSuchEventError
        self._cbs[event] = [(cb, args)]

    def get_node(self, path):
        '''Get a child node of this node, or this node, based on a path.

        @param path A list of path elements pointing to a node in the tree.
                    For example, ['/', 'localhost', 'dir.host']. The first
                    element in this path should be this node's name.
        @return The node pointed to by @ref path, or None if the path does not
                point to a node in the tree below this node.

        '''
        with self._mutex:
            if path[0] == self._name:
                if len(path) == 1:
                    return self
                elif path[1] in self._children:
                    return self._children[path[1]].get_node(path[1:])
                else:
                    return None
            else:
                return None

    def has_path(self, path):
        '''Check if a path exists below this node.

        @param path A list of path elements pointing to a node in the tree.
                    For example, ['/', 'localhost', 'dir.host']. The first
                    element in this path should be this node's name.
        @return True if the path points to a node in the tree below this node,
                or this node itself (for paths one element long). False
                otherwise.

        '''
        with self._mutex:
            if path[0] == self._name:
                if len(path) == 1:
                    return True
                elif path[1] in self._children:
                    return self._children[path[1]].has_path(path[1:])
                else:
                    return False
            else:
                return False

    def is_child(self, other_node):
        '''Is @ref other_node a child of this node?'''
        with self._mutex:
            return other_node in self._children

    def is_parent(self, other_node):
        '''Is @ref other_node the parent of this note?'''
        return other_node == self.parent

    def iterate(self, func, args=None, filter=[]):
        '''Call a function on this node, and recursively all its children.

        This is a depth-first iteration.

        @param func The function to call. Its declaration must be
                    'def blag(node, args)', where 'node' is the current node
                    in the iteration and args is the value of @ref args.
        @param args Extra arguments to pass to the function at each iteration.
                    Pass multiple arguments in as a tuple.
        @param filter A list of filters to apply before calling func for each
                      node in the iteration. If the filter is not True,
                      @ref func will not be called for that node. Each filter
                      entry should be a string, representing one of the is_*
                      properties (is_component, etc), or a function object.
        @return The results of the calls to @ref func in a list.

        '''
        with self._mutex:
            result = []
            if filter:
                filters_passed = True
                for f in filter:
                    if type(f) == str:
                        if not eval('self.' + f):
                            filters_passed = False
                            break
                    else:
                        if not f(self):
                            filters_passed = False
                            break
                if filters_passed:
                    result = [func(self, args)]
            else:
                result = [func(self, args)]
            for child in self._children:
                result += self._children[child].iterate(func, args, filter)
        return result

    def rem_callback(self, event, cb):
        '''Remove a callback from this node.

        The callback is removed from the specified event.

        @param cb The callback function to remove.

        '''
        if event not in self._cbs:
            raise exceptions.NoSuchEventError(self.name, event)
        c = [(x[0], x[1]) for x in self._cbs[event]]
        if not c:
            raise exceptions.NoCBError(self.name, event, cb)
        self._cbs[event].remove(c[0])

    @property
    def children(self):
        '''The child nodes of this node (if any).'''
        with self._mutex:
            return self._children.values()

    @property
    def children_names(self):
        '''A list of the names of the child nodes of this node (if any).'''
        with self._mutex:
            return self._children.keys()

    @property
    def depth(self):
        '''The depth of this node in the tree.

        The root node is depth 0.

        '''
        with self._mutex:
            if self._parent:
                return len(self.full_path) - 1
            else:
                return 0

    @property
    def dynamic(self):
        '''Get and change the dynamic setting of this node.'''
        with self._mutex:
            return self._dynamic

    @dynamic.setter
    def dynamic(self, dynamic):
        with self._mutex:
            if self._dynamic and not dynamic:
                # Disable dynamism
                self._enable_dynamic(False)
            elif not self._dynamic and dynamic:
                # Enable dynamism
                self._enable_dynamic(True)

    @property
    def full_path(self):
        '''The full path of this node.'''
        with self._mutex:
            if self._parent:
                return self._parent.full_path + [self._name]
            else:
                return [self._name]

    @property
    def full_path_str(self):
        '''The full path of this node as a string.'''
        with self._mutex:
            if self._parent:
                if self._parent._name == '/':
                    return self._parent.full_path_str + self._name
                else:
                    return self._parent.full_path_str + '/' + self._name
            else:
                return self._name

    @property
    def is_component(self):
        '''Is this node a component?'''
        return False

    @property
    def is_directory(self):
        '''Is this node a directory?'''
        with self._mutex:
            if self._name == '/':
                return True
            return False

    @property
    def is_manager(self):
        '''Is this node a manager?'''
        return False

    @property
    def is_nameserver(self):
        '''Is this node a name server (specialisation of directory nodes)?'''
        return False

    @property
    def is_unknown(self):
        '''Is this node unknown?'''
        return False

    @property
    def is_zombie(self):
        '''Is this node a zombie?'''
        return False

    @property
    def name(self):
        '''The name of this node.'''
        with self._mutex:
            return self._name

    @property
    def nameserver(self):
        '''The name server of the node (i.e. its top-most parent below /).'''
        with self._mutex:
            if not self._parent:
                # The root node does not have a name server
                return None
            elif self._parent.name == '/':
                return self
            else:
                return self._parent.nameserver

    @property
    def orb(self):
        '''The ORB used to access this object.

        This property's value will be None if no object above this object is a
        name server.

        '''
        with self._mutex:
            if self._parent.name == '/':
                return None
            return self._parent.orb

    @property
    def parent(self):
        '''This node's parent, or None if no parent.'''
        with self._mutex:
            return self._parent

    def remove_child(self, child):
        # Remove a child from this node.
        with self._mutex:
            if child.name not in self._children:
                raise NotRelatedError(self.name, child.name)
            del self._children[child.name]

    @parent.setter
    def parent(self, new_parent):
        with self._mutex:
            if self._parent:
                # Make sure to unlink the tree as well
                self._parent.remove_child(self)
            self._parent = new_parent

    @property
    def parent_name(self):
        '''The name of this node's parent or an empty string if no parent.'''
        with self._mutex:
            if self._parent:
                return self._parent.name
            else:
                return ''

    @property
    def root(self):
        '''The root node of the tree this node is in.'''
        with self._mutex:
            if self._parent:
                return self._parent.root
            else:
                return self

    def _add_child(self, new_child):
        # Add a child to this node.
        with self._mutex:
            self._children[new_child._name] = new_child

    def _call_cb(self, event, value):
        if event not in self._cbs:
            raise exceptions.NoSuchEventError(self.name, event)
        for (cb, args) in self._cbs[event]:
            cb(self, value, args)

    def _enable_dynamic(self, enable=True):
        # Enable or disable dynamic features.
        # By default, do nothing.
        pass

    def _remove_all_children(self):
        # Remove all children from this node.
        self._children = {}

    def _set_events(self, events):
        self._cbs = {}
        for e in events:
            self._cbs[e] = []


# vim: tw=79

